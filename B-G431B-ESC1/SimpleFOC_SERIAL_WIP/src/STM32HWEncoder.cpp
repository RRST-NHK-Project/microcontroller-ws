#include "STM32HWEncoder.h"

#if defined(_STM32_DEF_) || defined(ARDUINO_ARCH_STM32)

#include "drivers/hardware_specific/stm32/stm32_mcu.h"

STM32HWEncoder::STM32HWEncoder(
    unsigned int _ppr,
    int pinA,
    int pinB,
    int pinI) {

    cpr = _ppr * 4;

    _pinA = digitalPinToPinName(pinA);
    _pinB = digitalPinToPinName(pinB);
    _pinI = digitalPinToPinName(pinI);

    index_found = false;
}

float STM32HWEncoder::getSensorAngle() {

    int16_t cnt =
        (int16_t)encoder_handle.Instance->CNT;

    return ((float)cnt / (float)cpr) * _2PI;
}

float STM32HWEncoder::getVelocity() {

    static int16_t last_cnt = 0;
    static uint32_t last_us = 0;

    uint32_t now = micros();

    int16_t cnt =
        (int16_t)encoder_handle.Instance->CNT;

    if (last_us == 0) {

        last_us = now;
        last_cnt = cnt;

        return 0.0f;
    }

    int16_t diff = cnt - last_cnt;

    float dt =
        (now - last_us) * 1e-6f;

    last_cnt = cnt;
    last_us = now;

    if (dt <= 0.0f)
        return 0.0f;

    return ((float)diff / (float)cpr) * _2PI / dt;
}

int32_t STM32HWEncoder::getCount() {

    return (int16_t)encoder_handle.Instance->CNT;
}

int STM32HWEncoder::needsSearch() {

    return 0;
}

int STM32HWEncoder::hasIndex() {

    return (_pinI != NC);
}

void STM32HWEncoder::init() {

    pinMode(
        pinNametoDigitalPin(_pinA),
        INPUT_PULLUP);

    pinMode(
        pinNametoDigitalPin(_pinB),
        INPUT_PULLUP);

    TIM_TypeDef *InstanceA =
        (TIM_TypeDef *)pinmap_peripheral(
            _pinA,
            PinMap_TIM);

    TIM_TypeDef *InstanceB =
        (TIM_TypeDef *)pinmap_peripheral(
            _pinB,
            PinMap_TIM);

    if (InstanceA == NP) {
        return;
    }

    if (InstanceA != InstanceB) {
        return;
    }

    pinmap_pinout(_pinA, PinMap_TIM);
    pinmap_pinout(_pinB, PinMap_TIM);

    encoder_handle.Instance = InstanceA;

    encoder_handle.Init.Prescaler = 0;

    encoder_handle.Init.CounterMode =
        TIM_COUNTERMODE_UP;

    encoder_handle.Init.Period = 0xFFFF;

    encoder_handle.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;

    encoder_handle.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;

    enableTimerClock(&encoder_handle);

    TIM_Encoder_InitTypeDef sConfig;

    sConfig.EncoderMode =
        TIM_ENCODERMODE_TI12;

    sConfig.IC1Polarity =
        TIM_INPUTCHANNELPOLARITY_RISING;

    sConfig.IC1Selection =
        TIM_ICSELECTION_DIRECTTI;

    sConfig.IC1Prescaler =
        TIM_ICPSC_DIV1;

    sConfig.IC1Filter = 10;

    sConfig.IC2Polarity =
        TIM_INPUTCHANNELPOLARITY_RISING;

    sConfig.IC2Selection =
        TIM_ICSELECTION_DIRECTTI;

    sConfig.IC2Prescaler =
        TIM_ICPSC_DIV1;

    sConfig.IC2Filter = 10;

    HAL_TIM_Encoder_Init(
        &encoder_handle,
        &sConfig);

    __HAL_TIM_SET_COUNTER(
        &encoder_handle,
        0);

    HAL_TIM_Encoder_Start(
        &encoder_handle,
        TIM_CHANNEL_ALL);
}

#endif