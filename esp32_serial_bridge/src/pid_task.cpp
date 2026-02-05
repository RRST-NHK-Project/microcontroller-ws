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
void md_enc_init();
// エンコーダのDIPスイッチをすべてoffにすること
//  ================= TASK =================

// PID制御タスク馬渕385
void PID_Task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();
    md_enc_init();
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

// 半田ミスったから使うの変更
void md_enc_init()
{
    // MDの方向ピンを出力に設定
    pinMode(MD3D, OUTPUT);
    pinMode(MD4D, OUTPUT);

    // PWMの初期化
    ledcSetup(2, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcSetup(3, MD_PWM_FREQ, MD_PWM_RESOLUTION);

    ledcAttachPin(MD3P, 2);
    ledcAttachPin(MD4P, 3);

    ENCx2_init();

    // SW ピン初期化
    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW7, INPUT_PULLUP);
    pinMode(SW8, INPUT_PULLUP);
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

    static uint8_t sw1_prev = HIGH;
    static uint8_t sw2_prev = HIGH;

    // swがncのときこれ
    uint8_t sw1_now = digitalRead(SW1);
    Tx_16Data[11] = !digitalRead(SW1);
    if (sw1_prev == LOW && sw1_now == HIGH)
    {
        // リセット処理
        total_cnt0 = 0;
    }
    uint8_t sw2_now = digitalRead(SW2);
    Tx_16Data[12] = !digitalRead(SW2);
    if (sw2_prev == LOW && sw2_now == HIGH)
    {
        // リセット処理
        total_cnt1 = 0;
    }

    sw1_prev = sw1_now;
    sw2_prev = sw2_now;
    // オーバーフロー対策が甘いがとりあえずそのまま送る
    Tx_16Data[1] = static_cast<int16_t>(angle[0]);
    Tx_16Data[2] = static_cast<int16_t>(angle[1]);

    output[0] = pid_calculate(target_angle[0], angle[0], pos_error_prev[0], pos_integral[0],
                              kp, ki, kd, dt);
    output[0] = constrain(output[0], -MD_PWM_MAX, MD_PWM_MAX);

    output[1] = pid_calculate(target_angle[1], angle[1], pos_error_prev[1], pos_integral[1],
                              kp, ki, kd, dt);
    output[1] = constrain(output[1], -MD_PWM_MAX, MD_PWM_MAX);

    digitalWrite(MD3D, output[0] > 0 ? HIGH : LOW);
    digitalWrite(MD4D, output[1] > 0 ? HIGH : LOW);

    ledcWrite(0, abs(output[0]));
    ledcWrite(1, abs(output[1]));
}
