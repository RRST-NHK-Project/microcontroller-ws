/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "defs.hpp"
#include "driver/pcnt.h"
#include "frame_data.hpp"
#include "pin_ctrl_init.hpp"
#include <Arduino.h>

constexpr uint32_t CTRL_PERIOD_MS = 5; // ピン更新周期（ミリ秒）

void pid_control();

// ================= TASK =================

// PID制御タスク馬渕385
void PID_Task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    Input_init();
    while (1) {
        // PID制御処理をここに実装予定
        // ENC_Input();
        pid_control();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

void pid_control() {

    int16_t enc_now;
    static int16_t enc_prev = 0;
    static int32_t enc_total;
    pcnt_get_counter_value(PCNT_UNIT_0, (int16_t *)&Tx_16Data[1]);
    int32_t diff = Tx_16Data[1] - enc_prev;

    // 16bit wrap 補正
    if (diff > 32767)
        diff -= 65536;
    if (diff < -32768)
        diff += 65536;

    enc_total += diff;
    enc_prev = enc_now;

    Tx_16Data[6] = (int16_t)(enc_total & 0xFFFF);         // low
    Tx_16Data[5] = (int16_t)((enc_total >> 16) & 0xFFFF); // high

    // int32_t enc_cnt = Tx_16Data[5]*1000 + Tx_16Data[6];//エンコーダカウント値
    float kp = 1.0; // Rx_16Data[21];
    float ki = 0.0; // Rx_16Data[22];
    float kd = 0.0; // Rx_16Data[23];
    float dt = CTRL_PERIOD_MS / 1000.0f;
    int target = 180; // deg;

    // static int32_t enc_total = 0;
    // static int16_t enc_prev = 0;
    // int16_t enc_now;

    // pcnt_get_counter_value(PCNT_UNIT_0, &enc_now);
    // // ここで使う

    // int32_t diff = enc_now - enc_prev;

    int32_t current = enc_total * (360.0f / 8192.0f); // deg
    float error = target - current;
    static float integral = 0;
    static float error_prev = 0;

    integral += error * dt;
    float derivative = (error - error_prev) / dt;
    float output = kp * error + ki * integral + kd * derivative;
    output = constrain(output, -MD_PWM_MAX, MD_PWM_MAX);
    // digitalWrite(MD1D, output > 0 ? HIGH : LOW);
    // ledcWrite(0, abs((int)output));
    error_prev = error;
}
