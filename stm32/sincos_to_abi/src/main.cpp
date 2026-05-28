#include <Arduino.h>
#include <math.h>

#define PIN_SIN PA0
#define PIN_COS PA1

float offset_s = 1.65f;
float offset_c = 1.65f;

float angle_prev = 0;
float angle_total = 0;

uint32_t last_us = 0;

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);

    delay(1000);

    // ---- 自動オフセット取得 ----
    float s_sum = 0;
    float c_sum = 0;

    for (int i = 0; i < 1000; i++) {
        s_sum += analogRead(PIN_SIN);
        c_sum += analogRead(PIN_COS);
        delayMicroseconds(200);
    }

    offset_s = (s_sum / 1000.0f) * (3.3f / 4095.0f);
    offset_c = (c_sum / 1000.0f) * (3.3f / 4095.0f);
}

void loop() {

    // 50kHz相当
    if (micros() - last_us < 20)
        return;
    last_us = micros();

    int s_raw = analogRead(PIN_SIN);
    int c_raw = analogRead(PIN_COS);

    // 電圧化 + オフセット除去
    float s = (s_raw * (3.3f / 4095.0f)) - offset_s;
    float c = (c_raw * (3.3f / 4095.0f)) - offset_c;

    // 正規化（超重要：円にする）
    float norm = sqrtf(s * s + c * c);
    if (norm < 0.001f)
        return;

    s /= norm;
    c /= norm;

    // 角度
    float angle = atan2f(s, c);

    // unwrap（累積角度）
    float diff = angle - angle_prev;

    if (diff > M_PI)
        diff -= 2 * M_PI;
    if (diff < -M_PI)
        diff += 2 * M_PI;

    angle_total += diff;
    angle_prev = angle;

    // 出力（遅く）
    static int cnt = 0;
    if (++cnt > 50) {
        cnt = 0;
        Serial.println(angle_total);
    }
}