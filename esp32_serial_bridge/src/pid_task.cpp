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
void PID_Task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();
    IO_init();
    while (1)
    {
        // PID制御処理をここに実装予定
        pid_control();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

void pid_control()
{

    // int32_t enc_cnt = Tx_16Data[5]*1000 + Tx_16Data[6];//エンコーダカウント値
    float kp = 3.0; // Rx_16Data[21];
    float ki = 0.0; // Rx_16Data[22];
    float kd = 0.1; // Rx_16Data[23];
    float dt = CTRL_PERIOD_MS / 1000.0f;
    int target = Rx_16Data[0]; // deg;

    int16_t cnt0, cnt1;
    static int32_t total_cnt0 = 0;
    static int32_t total_cnt1 = 0;

    pcnt_get_counter_value(PCNT_UNIT_0, &cnt0);
    pcnt_get_counter_value(PCNT_UNIT_1, &cnt1);

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    total_cnt0 += cnt0;
    total_cnt1 += cnt1;

    float angle0 = total_cnt0 * DEG_PER_COUNT;
    float angle1 = total_cnt1 * DEG_PER_COUNT;

    // オーバーフロー対策が甘いがとりあえずそのまま送る
    Tx_16Data[1] = static_cast<int16_t>(angle0);
    Tx_16Data[2] = static_cast<int16_t>(angle1);

    float error = target - angle0;
    static float integral = 0;
    static float error_prev = 0;

    integral += error * dt;
    float derivative = (error - error_prev) / dt;
    float output = kp * error + ki * integral + kd * derivative;
    output = constrain(output, -MD_PWM_MAX, MD_PWM_MAX);

    static int Rx16Data_local[motor];

    for (int i = 1; i <= 2; i++)
    {
        Rx16Data_local[i] = constrain(output, -MD_PWM_MAX, MD_PWM_MAX);
    }

    digitalWrite(MD1D, Rx16Data_local[1] > 0 ? HIGH : LOW);
    digitalWrite(MD2D, Rx16Data_local[2] > 0 ? HIGH : LOW);

    ledcWrite(0, abs(Rx16Data_local[1]));
    ledcWrite(1, abs(Rx16Data_local[2]));

    error_prev = error;
}
