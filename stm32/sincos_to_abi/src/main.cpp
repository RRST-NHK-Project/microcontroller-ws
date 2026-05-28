#include "stm32f3xx_hal.h"
#include <Arduino.h>
#include <math.h>

namespace {
    constexpr uint32_t kSampleHz = 50000;
    constexpr uint32_t kPpr = 1024;
    constexpr uint32_t kTotalCounts = kPpr * 4;

    constexpr uint8_t kPinSin = A0;
    constexpr uint8_t kPinCos = A1;
    constexpr uint8_t kPinA = D2;
    constexpr uint8_t kPinB = D3;
    constexpr uint8_t kPinZ = D4;

    constexpr uint32_t kZCenterCounts = 0;
    constexpr uint32_t kZWidthCounts = 4;
    constexpr bool kInvertDirection = false;

    constexpr bool kUseFastAtan2 = true;
    constexpr bool kEnableFilter = false;
    constexpr float kFilterAlpha = 0.2f;

    constexpr float kSinOffset = 2048.0f;
    constexpr float kCosOffset = 2048.0f;
    constexpr float kSinGain = 1.0f / 2048.0f;
    constexpr float kCosGain = 1.0f / 2048.0f;
    constexpr float kPhaseSkew = 0.0f;

    constexpr float kTwoPi = 6.283185307179586f;
    constexpr float kPhaseScale = static_cast<float>(kTotalCounts) / kTwoPi;

    ADC_HandleTypeDef g_adc1;
    DMA_HandleTypeDef g_dma_adc1;
    TIM_HandleTypeDef g_tim2;

    uint16_t g_adcBuffer[2];
    uint32_t g_lastPhase = 0;
    float g_sinFilt = 0.0f;
    float g_cosFilt = 0.0f;

    inline float fastAtan2f(float y, float x) {
        const float abs_y = fabsf(y) + 1.0e-10f;
        float r = 0.0f;
        float angle = 0.0f;

        if (x >= 0.0f) {
            r = (x - abs_y) / (x + abs_y);
            angle = 0.78539816339f;
        } else {
            r = (x + abs_y) / (abs_y - x);
            angle = 2.35619449019f;
        }

        angle += (0.1963f * r * r - 0.9817f) * r;
        return (y < 0.0f) ? -angle : angle;
    }

    inline float computeAngle(float sin_v, float cos_v) {
        if (kUseFastAtan2) {
            return fastAtan2f(sin_v, cos_v);
        }
        return atan2f(sin_v, cos_v);
    }

    inline void writePin(uint8_t pin, bool level) {
#if defined(digitalWriteFast)
        digitalWriteFast(pin, level ? HIGH : LOW);
#else
        digitalWrite(pin, level ? HIGH : LOW);
#endif
    }

    inline bool isWithinZWindow(uint32_t phase_q) {
        if (kZWidthCounts == 0 || kTotalCounts == 0) {
            return false;
        }

        uint32_t width = kZWidthCounts;
        if (width >= kTotalCounts) {
            width = kTotalCounts;
        }

        const uint32_t half = width / 2;
        const uint32_t dist = (phase_q + kTotalCounts - kZCenterCounts) % kTotalCounts;
        return (dist <= half) || (dist >= (kTotalCounts - half));
    }

    inline void updateOutputs(uint32_t phase_q) {
        uint32_t phase_dir = phase_q;
        if (kInvertDirection && phase_q != 0) {
            phase_dir = kTotalCounts - phase_q;
        }

        const uint8_t index = static_cast<uint8_t>(phase_dir & 0x3);
        const uint8_t a_state[4] = {0, 1, 1, 0};
        const uint8_t b_state[4] = {0, 0, 1, 1};

        writePin(kPinA, a_state[index] != 0);
        writePin(kPinB, b_state[index] != 0);
        writePin(kPinZ, isWithinZWindow(phase_q));
    }

    uint32_t timerInputClockHz() {
        uint32_t pclk = HAL_RCC_GetPCLK1Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
            pclk *= 2U;
        }
        return pclk;
    }

    void initTimer2(uint32_t sample_hz) {
        __HAL_RCC_TIM2_CLK_ENABLE();

        const uint32_t tim_clk = timerInputClockHz();
        const uint32_t prescaler = (tim_clk / 1000000U) - 1U;
        const uint32_t period = (1000000U / sample_hz) - 1U;

        g_tim2.Instance = TIM2;
        g_tim2.Init.Prescaler = prescaler;
        g_tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
        g_tim2.Init.Period = period;
        g_tim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        g_tim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        HAL_TIM_Base_Init(&g_tim2);

        TIM_MasterConfigTypeDef master_cfg = {};
        master_cfg.MasterOutputTrigger = TIM_TRGO_UPDATE;
        master_cfg.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        HAL_TIMEx_MasterConfigSynchronization(&g_tim2, &master_cfg);
        HAL_TIM_Base_Start(&g_tim2);
    }

    void initAdc1() {
        __HAL_RCC_ADC12_CLK_ENABLE();

        g_adc1.Instance = ADC1;
        g_adc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
        g_adc1.Init.Resolution = ADC_RESOLUTION_12B;
        g_adc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
        g_adc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
        g_adc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
        g_adc1.Init.LowPowerAutoWait = DISABLE;
        g_adc1.Init.ContinuousConvMode = DISABLE;
        g_adc1.Init.NbrOfConversion = 2;
        g_adc1.Init.DiscontinuousConvMode = DISABLE;
        g_adc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
        g_adc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
        g_adc1.Init.DMAContinuousRequests = ENABLE;
        g_adc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

        HAL_ADC_Init(&g_adc1);

        ADC_ChannelConfTypeDef channel_cfg = {};
        channel_cfg.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
        channel_cfg.SingleDiff = ADC_SINGLE_ENDED;
        channel_cfg.OffsetNumber = ADC_OFFSET_NONE;
        channel_cfg.Offset = 0;

        channel_cfg.Channel = ADC_CHANNEL_1;
        channel_cfg.Rank = ADC_REGULAR_RANK_1;
        HAL_ADC_ConfigChannel(&g_adc1, &channel_cfg);

        channel_cfg.Channel = ADC_CHANNEL_2;
        channel_cfg.Rank = ADC_REGULAR_RANK_2;
        HAL_ADC_ConfigChannel(&g_adc1, &channel_cfg);

        HAL_ADCEx_Calibration_Start(&g_adc1, ADC_SINGLE_ENDED);
    }

    void initAdcDma() {
        __HAL_RCC_DMA1_CLK_ENABLE();

        g_dma_adc1.Instance = DMA1_Channel1;
        g_dma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        g_dma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
        g_dma_adc1.Init.MemInc = DMA_MINC_ENABLE;
        g_dma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        g_dma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        g_dma_adc1.Init.Mode = DMA_CIRCULAR;
        g_dma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
        HAL_DMA_Init(&g_dma_adc1);

        __HAL_LINKDMA(&g_adc1, DMA_Handle, g_dma_adc1);

        HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    }

    void processSample(uint16_t raw_sin, uint16_t raw_cos) {
        float sin_v = (static_cast<float>(raw_sin) - kSinOffset) * kSinGain;
        float cos_v = (static_cast<float>(raw_cos) - kCosOffset) * kCosGain;

        cos_v += sin_v * kPhaseSkew;

        if (kEnableFilter) {
            g_sinFilt += (sin_v - g_sinFilt) * kFilterAlpha;
            g_cosFilt += (cos_v - g_cosFilt) * kFilterAlpha;
            sin_v = g_sinFilt;
            cos_v = g_cosFilt;
        }

        float angle = computeAngle(sin_v, cos_v);
        if (angle < 0.0f) {
            angle += kTwoPi;
        }

        uint32_t phase_q = static_cast<uint32_t>(angle * kPhaseScale);
        if (phase_q >= kTotalCounts) {
            phase_q = 0;
        }

        if (phase_q != g_lastPhase) {
            updateOutputs(phase_q);
            g_lastPhase = phase_q;
        }
    }
} // namespace

extern "C" void DMA1_Channel1_IRQHandler(void) {
    HAL_DMA_IRQHandler(&g_dma_adc1);
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance != ADC1) {
        return;
    }

    const uint16_t raw_sin = g_adcBuffer[0];
    const uint16_t raw_cos = g_adcBuffer[1];
    processSample(raw_sin, raw_cos);
}

void setup() {
    pinMode(kPinSin, INPUT_ANALOG);
    pinMode(kPinCos, INPUT_ANALOG);
    pinMode(kPinA, OUTPUT);
    pinMode(kPinB, OUTPUT);
    pinMode(kPinZ, OUTPUT);

    writePin(kPinA, false);
    writePin(kPinB, false);
    writePin(kPinZ, false);

    initTimer2(kSampleHz);
    initAdc1();
    initAdcDma();
    HAL_ADC_Start_DMA(&g_adc1, reinterpret_cast<uint32_t *>(g_adcBuffer), 2);
}

void loop() {
    __WFI();
}