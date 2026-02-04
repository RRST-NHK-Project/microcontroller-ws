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
void pid_calculate();
// エンコーダのDIPスイッチをすべてoffにすること
//  ================= TASK =================

// PID制御タスク馬渕385
void PID_Task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();
    IO_init();
    while (1)
    {
        pid_control();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

float pid_calculate(float setpoint, float input, float &error_prev, float &integral,
                    float kp, float ki, float kd, float dt)
{
    float error = setpoint - input;
    integral += (error + error_prev) * dt;
    float derivative = (error - error_prev) / dt;
    error_prev = error;
    return kp * error + ki * integral + kd * derivative;
}

void pid_control()
{

    float kp = 3.0; // Rx_16Data[21];
    float ki = 0.0; // Rx_16Data[22];
    float kd = 0.1; // Rx_16Data[23];
    float dt = CTRL_PERIOD_MS / 1000.0f;

    target_angle[0] = Rx_16Data[0]; // deg;
    target_angle[1] = Rx_16Data[1]; // deg;

    int16_t cnt0, cnt1;
    static int32_t total_cnt0 = 0;
    static int32_t total_cnt1 = 0;

    pcnt_get_counter_value(PCNT_UNIT_0, &cnt0);
    pcnt_get_counter_value(PCNT_UNIT_1, &cnt1);

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    total_cnt0 += cnt0;
    total_cnt1 += cnt1;

    angle[0] = total_cnt0 * DEG_PER_COUNT;
    angle[1] = total_cnt1 * DEG_PER_COUNT;

    // オーバーフロー対策が甘いがとりあえずそのまま送る
    Tx_16Data[1] = static_cast<int16_t>(angle[0]);
    Tx_16Data[2] = static_cast<int16_t>(angle[1]);

    // float error = target - angle0;
    // static float integral = 0;
    // static float error_prev = 0;

    // integral += error * dt;
    // float derivative = (error - error_prev) / dt;
    // float output = kp * error + ki * integral + kd * derivative;

    output[0] = pid_calculate(target_angle[0], angle[0], pos_error_prev[0], pos_integral[0],
                              kp, ki, kd, dt);
    output[0] = constrain(output[0], -MD_PWM_MAX, MD_PWM_MAX);

    output[1] = pid_calculate(target_angle[1], angle[1], pos_error_prev[1], pos_integral[1],
                              kp, ki, kd, dt);
    output[1] = constrain(output[1], -MD_PWM_MAX, MD_PWM_MAX);

    digitalWrite(MD1D, output[0] > 0 ? HIGH : LOW);
    digitalWrite(MD2D, output[1] > 0 ? HIGH : LOW);

    ledcWrite(0, abs(output[0]));
    ledcWrite(1, abs(output[1]));

    //  error_prev = error;
}
