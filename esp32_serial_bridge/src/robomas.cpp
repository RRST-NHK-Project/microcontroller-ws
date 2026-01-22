/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "frame_data.hpp"
#include <Arduino.h>
#include <defs.hpp>
#include "robomas.hpp"

// 状態量 / CAN受信関連
int encoder_count[NUM_MOTOR] = {0};
int rpm[NUM_MOTOR] = {0};
int current[NUM_MOTOR] = {0};

bool offset_ok[NUM_MOTOR] = {false};
int encoder_offset[NUM_MOTOR] = {0};
int last_encoder[NUM_MOTOR] = {-1, -1, -1, -1};
int rotation_count[NUM_MOTOR] = {0};
long total_encoder[NUM_MOTOR] = {0};

// // float angle[NUM_MOTOR] = {0.0f};
// float vel[NUM_MOTOR] = {0.0f};

// PID関連
float target_rpm[NUM_MOTOR] = {0.0f};

float pos_error_prev[NUM_MOTOR] = {0.0f};
float vel_error_prev[NUM_MOTOR] = {0.0f};
float cur_error_prev[NUM_MOTOR] = {0.0f};

float vel_prop_prev[NUM_MOTOR] = {0}; // 速度比例項

float pos_integral[NUM_MOTOR] = {0.0f};
float vel_integral[NUM_MOTOR] = {0.0f};
float cur_integral[NUM_MOTOR] = {0.0f};

float vel_out[NUM_MOTOR] = {0.0f};
float vel_output[NUM_MOTOR] = {0.0f};
float pos_output[NUM_MOTOR] = {0.0f};
float cur_output[NUM_MOTOR] = {0.0f};

float motor_output_current[NUM_MOTOR] = {0.0f};

// M3508物理量
float angle_m3508[NUM_MOTOR] = {0.0f};
float vel_m3508[NUM_MOTOR] = {0.0f};
float c[NUM_MOTOR] = {0.0f};

//-- -- -- --PIDゲイン-- -- -- -- //
float kp_pos = 0.8f;  // 角度比例ゲイン
float ki_pos = 0.01f; // 角度積分ゲイン
float kd_pos = 0.02f; // 角度微分ゲイン

// -------- 速度PIDゲイン -------- //
float kp_vel = 0.8;
float ki_vel = 0.0;
float kd_vel = 0.05; // 微分は控えめに

// -------- 電流PIDゲイン -------- //
float kp_cur = 0.01;
float ki_cur = 0.0;
float kd_cur = 0.0; // 微分は控えめに

// タイマー
unsigned long lastPidTime = 0;

float constrain_double(float val, float min_val, float max_val)
{
    if (val < min_val)
        return min_val;
    if (val > max_val)
        return max_val;
    return val;
}

void send_cur_all(float cur_array[NUM_MOTOR])
{
    twai_message_t tx;       // 送信用メッセージ
    tx.identifier = 0x200;   // CAN ID
    tx.extd = 0;             // 標準フレーム
    tx.rtr = 0;              // データフレーム
    tx.data_length_code = 8; // 8バイト

    // C620 の仕様: -16384 ～ +16384
    for (int i = 0; i < NUM_MOTOR; i++)
    {
        float amp = constrain_double(cur_array[i], -20, 20);
        int16_t val = amp * (16384.0f / 200.0f);

        tx.data[i * 2] = (val >> 8) & 0xFF;
        tx.data[i * 2 + 1] = val & 0xFF;
    }

    if (twai_transmit(&tx, pdMS_TO_TICKS(20)) != ESP_OK)
    {
        Serial.println("[ERR] twai_transmit failed");
    }
}

float pid_vel(float setpoint, float input, float &error_prev, float &prop_prev, float &output,
              float kp, float ki, float kd, float dt)
{
    float error = setpoint - input;
    float prop = error - error_prev;
    float deriv = prop - prop_prev;
    float du = kp * prop + ki * error * dt + kd * deriv;
    output += du;

    prop_prev = prop;
    error_prev = error;

    return output;
}

// M3508制御タスク
void M3508_Task(void *)
{
    static unsigned long lastPidTime = 0;

    // 初期化
    lastPidTime = millis();

    while (true)
    {
        // 時間差分 dt 計算
        unsigned long now = millis();
        float dt = (now - lastPidTime) / 1000.0f;

        if (dt <= 0.0f)
            dt = 0.000001f; // 0除算防止
        if (dt > 0.02f)
            dt = 0.02f; // 最大20ms制限

        lastPidTime = now;

        // 目標回転数生成
        for (int i = 0; i < NUM_MOTOR; i++)
        {

            target_rpm[i] = Rx_16Data[i + 1]; // Rx_16Data[1]～[4]に目標回転数が入っている想定
        }

        /* ================= PID制御 ================= */
        for (int i = 0; i < NUM_MOTOR; i++)
        {
            // 速度PID
            vel_out[i] = pid_vel(
                target_rpm[i],
                vel_m3508[i],
                vel_error_prev[i],
                vel_prop_prev[i],
                vel_output[i],
                kp_vel, ki_vel, kd_vel,
                dt);

            // 電流指令へ変換
            motor_output_current[i] = vel_out[i] * 10.0f;

            constrain_double(motor_output_current[i], -20.0f, 20.0f);
        }

        /* ================= CAN送信 ================= */
        send_cur_all(motor_output_current);

        /* ================= デバッグ ================= */
        Serial.print("vel:");
        Serial.print(vel_m3508[0]);
        Serial.print("\tcur:");
        Serial.println(current[0]);

        /* ================= タスク周期 ================= */
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms周期
    }
}

// M3508からのCAN受信タスク
void M3508_RX(void *)
{
    while (true)
    {
        twai_message_t rx_msg;

        while (twai_receive(&rx_msg, 0) == ESP_OK)
        {
            if (rx_msg.data_length_code != 8)
                continue;
            if (rx_msg.identifier < 0x201 || rx_msg.identifier > 0x204)
                continue;

            int m = rx_msg.identifier - 0x201;

            encoder_count[m] = (int16_t)(rx_msg.data[0] << 8 | rx_msg.data[1]);
            rpm[m] = (int16_t)(rx_msg.data[2] << 8 | rx_msg.data[3]);
            current[m] = (int16_t)(rx_msg.data[4] << 8 | rx_msg.data[5]);

            // エンコーダ回転数計算
            int diff = encoder_count[m] - last_encoder[m];
            if (diff > HALF_ENCODER)
                rotation_count[m]--;
            else if (diff < -HALF_ENCODER)
                rotation_count[m]++;

            last_encoder[m] = encoder_count[m];

            total_encoder[m] =
                rotation_count[m] * ENCODER_MAX + encoder_count[m];

            angle_m3508[m] = total_encoder[m] * (360.0f / (ENCODER_MAX * gear_m3508));
            vel_m3508[m] = rpm[m] / gear_m3508;
            c[m] = current[m] * 20.0f / 16384.0f;
        }

        /* CPUを占有しないように */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/*====================================================================

                robomas.cpp 処理全体フロー

        ┌─────────────────────────────┐
        │      CAN 受信  (M3508_RX)    　|
        │-----------------------------│
        │  ・CAN ID: 0x201～0x204      │
        │  ・encoder                  │
        │  ・rpm                      │
        │  ・current                  │
        └─────────────┬──────────────┘
                      │
                      ▼
        ┌─────────────────────────────┐
        │    状態量 更新   (M3508_RX)  │
        │-----------------------------│
        │ ・encoder_count[]           │
        │ ・rpm[]                     │
        │ ・current[]                 │
        │ ・rotation_count[]          │
        │ ・total_encoder[]           │
        │ ・angle[]                   │
        │ ・vel[]                     │
        └─────────────┬──────────────┘
                      │
                      ▼
        ┌─────────────────────────────┐
        │  制御計算 (PID) (M3508_Task) │
        │-----------------------------│
        │ ・目標角度 target_angle[]  │
        │ ・角度PID (外側)           │
        │ ・速度PID (中間)           │
        │ ・電流PID (内側 or 省略)    │
        └─────────────┬─────────────┘
                      │
                      ▼
        ┌─────────────────────────────┐
        │     CAN 送信   (send_cur) │
        │-----------------------------│
        │ ・CAN ID: 0x200             │
        │ ・motor_output_current[]   │
        └─────────────────────────────┘

====================================================================*/
