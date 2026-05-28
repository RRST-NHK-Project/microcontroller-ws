#include "driver/gpio.h"
#include "esp_timer.h"
#include <Arduino.h>
#include <math.h>

namespace {
    constexpr uint32_t kSampleHz = 40000;
    constexpr uint32_t kPpr = 1024;
    constexpr uint32_t kTotalCounts = kPpr * 4;

    // Update these pins to match your ESP32-S3 board.
    constexpr uint8_t kPinSin = 4; // ADC1 capable
    constexpr uint8_t kPinCos = 5; // ADC1 capable
    constexpr uint8_t kPinA = 15;
    constexpr uint8_t kPinB = 16;
    constexpr uint8_t kPinZ = 17;

    constexpr uint32_t kZCenterCounts = 0;
    constexpr uint32_t kZWidthCounts = 4;
    constexpr bool kInvertDirection = false;

    constexpr bool kUseFilter = false;
    constexpr float kFilterAlpha = 0.2f;
    constexpr uint32_t kMaxStepPerSample = 4;

    constexpr bool kEnableSerial = false;
    constexpr uint32_t kSerialBaud = 115200;
    constexpr uint32_t kSerialPeriodMs = 100;

    constexpr float kSinOffset = 2048.0f;
    constexpr float kCosOffset = 2048.0f;
    constexpr float kSinGain = 1.0f / 2048.0f;
    constexpr float kCosGain = 1.0f / 2048.0f;
    constexpr float kPhaseSkew = 0.0f;

    constexpr float kTwoPi = 6.283185307179586f;
    constexpr float kPhaseScale = static_cast<float>(kTotalCounts) / kTwoPi;

    volatile uint16_t g_lastRawSin = 0;
    volatile uint16_t g_lastRawCos = 0;
    volatile uint32_t g_lastPhase = 0;
    volatile uint32_t g_glitchCount = 0;
    volatile bool g_phaseValid = false;
    volatile bool g_samplePending = false;
    volatile uint32_t g_sampleMissed = 0;

    float g_sinFilt = 0.0f;
    float g_cosFilt = 0.0f;

    portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

    esp_timer_handle_t g_timer = nullptr;

    inline void writePin(uint8_t pin, bool level) {
        gpio_set_level(static_cast<gpio_num_t>(pin), level ? 1 : 0);
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

    void processSample(uint16_t raw_sin, uint16_t raw_cos) {
        uint32_t phase_state = 0;
        bool phase_valid = false;
        portENTER_CRITICAL(&g_mux);
        phase_state = g_lastPhase;
        phase_valid = g_phaseValid;
        portEXIT_CRITICAL(&g_mux);

        float sin_v = (static_cast<float>(raw_sin) - kSinOffset) * kSinGain;
        float cos_v = (static_cast<float>(raw_cos) - kCosOffset) * kCosGain;

        cos_v += sin_v * kPhaseSkew;

        if (kUseFilter) {
            g_sinFilt += (sin_v - g_sinFilt) * kFilterAlpha;
            g_cosFilt += (cos_v - g_cosFilt) * kFilterAlpha;
            sin_v = g_sinFilt;
            cos_v = g_cosFilt;
        }

        float angle = atan2f(sin_v, cos_v);
        if (angle < 0.0f) {
            angle += kTwoPi;
        }

        uint32_t phase_q = static_cast<uint32_t>(angle * kPhaseScale);
        if (phase_q >= kTotalCounts) {
            phase_q = 0;
        }

        if (!phase_valid) {
            updateOutputs(phase_q);
            portENTER_CRITICAL(&g_mux);
            g_lastPhase = phase_q;
            g_phaseValid = true;
            portEXIT_CRITICAL(&g_mux);
            return;
        }

        int32_t delta = static_cast<int32_t>(phase_q) - static_cast<int32_t>(phase_state);
        if (delta > static_cast<int32_t>(kTotalCounts / 2)) {
            delta -= static_cast<int32_t>(kTotalCounts);
        } else if (delta < -static_cast<int32_t>(kTotalCounts / 2)) {
            delta += static_cast<int32_t>(kTotalCounts);
        }

        if (delta == 0) {
            return;
        }

        const uint32_t delta_abs = (delta < 0) ? static_cast<uint32_t>(-delta)
                                               : static_cast<uint32_t>(delta);
        if (delta_abs > kMaxStepPerSample) {
            portENTER_CRITICAL(&g_mux);
            g_glitchCount++;
            portEXIT_CRITICAL(&g_mux);
            return;
        }

        const int32_t step = (delta > 0) ? 1 : -1;
        while (delta != 0) {
            uint32_t next = phase_state;
            if (step > 0) {
                next = (phase_state + 1U) % kTotalCounts;
            } else {
                next = (phase_state == 0) ? (kTotalCounts - 1U) : (phase_state - 1U);
            }
            updateOutputs(next);
            phase_state = next;
            delta -= step;
        }

        portENTER_CRITICAL(&g_mux);
        g_lastPhase = phase_state;
        portEXIT_CRITICAL(&g_mux);
    }

    void sampleTimerCb(void *arg) {
        (void)arg;
        portENTER_CRITICAL(&g_mux);
        if (g_samplePending) {
            g_sampleMissed++;
        } else {
            g_samplePending = true;
        }
        portEXIT_CRITICAL(&g_mux);
    }

    void startSamplingTimer(uint32_t sample_hz) {
        if (sample_hz == 0) {
            return;
        }

        esp_timer_create_args_t args = {};
        args.callback = &sampleTimerCb;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "sincos";
        if (esp_timer_create(&args, &g_timer) != ESP_OK) {
            return;
        }

        const uint64_t period_us = 1000000ULL / static_cast<uint64_t>(sample_hz);
        esp_timer_start_periodic(g_timer, period_us);
    }
} // namespace

void setup() {
    pinMode(kPinSin, INPUT);
    pinMode(kPinCos, INPUT);
    pinMode(kPinA, OUTPUT);
    pinMode(kPinB, OUTPUT);
    pinMode(kPinZ, OUTPUT);

    writePin(kPinA, false);
    writePin(kPinB, false);
    writePin(kPinZ, false);

    analogReadResolution(12);
    adcAttachPin(kPinSin);
    adcAttachPin(kPinCos);
    analogSetPinAttenuation(kPinSin, ADC_11db);
    analogSetPinAttenuation(kPinCos, ADC_11db);

    if (kEnableSerial) {
        Serial.begin(kSerialBaud);
    }

    startSamplingTimer(kSampleHz);
}

void loop() {
    bool do_sample = false;
    portENTER_CRITICAL(&g_mux);
    if (g_samplePending) {
        g_samplePending = false;
        do_sample = true;
    }
    portEXIT_CRITICAL(&g_mux);

    if (do_sample) {
        const uint16_t raw_sin = static_cast<uint16_t>(analogRead(kPinSin));
        const uint16_t raw_cos = static_cast<uint16_t>(analogRead(kPinCos));
        portENTER_CRITICAL(&g_mux);
        g_lastRawSin = raw_sin;
        g_lastRawCos = raw_cos;
        portEXIT_CRITICAL(&g_mux);
        processSample(raw_sin, raw_cos);
    }

    if (kEnableSerial) {
        static uint32_t last_print_ms = 0;
        const uint32_t now_ms = millis();
        if ((now_ms - last_print_ms) >= kSerialPeriodMs) {
            uint16_t raw_sin = 0;
            uint16_t raw_cos = 0;
            uint32_t phase_q = 0;
            uint32_t glitches = 0;
            uint32_t missed = 0;
            portENTER_CRITICAL(&g_mux);
            raw_sin = g_lastRawSin;
            raw_cos = g_lastRawCos;
            phase_q = g_lastPhase;
            glitches = g_glitchCount;
            missed = g_sampleMissed;
            portEXIT_CRITICAL(&g_mux);

            Serial.print("sin_raw=");
            Serial.print(raw_sin);
            Serial.print(" cos_raw=");
            Serial.print(raw_cos);
            Serial.print(" phase=");
            Serial.print(phase_q);
            Serial.print(" glitches=");
            Serial.print(glitches);
            Serial.print(" missed=");
            Serial.println(missed);
            last_print_ms = now_ms;
        }
    }

    delay(0);
}
