/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "frame_data.hpp"
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <SimpleFOC.h>
#include <defs.hpp>
#include <CAN.h> // STM32 Arduino CAN

// 状態量 / CAN受信関連
int encoder_count[NUM_MOTOR] = {0};
int rpm[NUM_MOTOR] = {0};
int current[NUM_MOTOR] = {0};

bool offset_ok[NUM_MOTOR] = {false};
int encoder_offset[NUM_MOTOR] = {0};
int last_encoder[NUM_MOTOR] = {-1, -1, -1, -1};
int rotation_count[NUM_MOTOR] = {0};
long total_encoder[NUM_MOTOR] = {0};

float angle[NUM_MOTOR] = {0.0f};
float vel[NUM_MOTOR] = {0.0f};

// PID関連
float target_rpm[NUM_MOTOR] = {0.0f};

float pos_error_prev[NUM_MOTOR] = {0.0f};
float vel_error_prev[NUM_MOTOR] = {0.0f};
float cur_error_prev[NUM_MOTOR] = {0.0f};

float pos_integral[NUM_MOTOR] = {0.0f};
float vel_integral[NUM_MOTOR] = {0.0f};
float cur_integral[NUM_MOTOR] = {0.0f};

float vel_out[NUM_MOTOR] = {0.0f};
float pos_output[NUM_MOTOR] = {0.0f};
float cur_output[NUM_MOTOR] = {0.0f};

float motor_output_current[NUM_MOTOR] = {0.0f};

// M3508物理量
float angle_m3508[NUM_MOTOR] = {0.0f};
float vel_m3508[NUM_MOTOR] = {0.0f};
float c[NUM_MOTOR] = {0.0f};

// タイマー
unsigned long lastPidTime = 0;

// M3508制御タスク
void M3508_Task()
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
void M3508_RX()
{
    while (true)
    {
        while (CAN.parsePacket())
        {
            int id = CAN.packetId();
            uint8_t rx[8] = {0};

            for (int i = 0; i < CAN.packetDlc(); i++)
            {
                rx[i] = CAN.read();
            }

            /* ================= モータID判定 ================= */
            if (id >= 0x201 && id <= 0x204)
            {
                int i = id - 0x201;

                /* ========= 生データ ========= */
                int16_t enc = (int16_t)((rx[0] << 8) | rx[1]);
                int16_t rpm_raw = (int16_t)((rx[2] << 8) | rx[3]);
                int16_t cur_raw = (int16_t)((rx[4] << 8) | rx[5]);

                encoder_count[i] = enc;
                rpm[i] = rpm_raw;
                current[i] = cur_raw;

                /* ========= エンコーダラップ補正 ========= */
                if (last_encoder[i] != -1)
                {
                    int diff = enc - last_encoder[i];

                    if (diff > HALF_ENCODER)
                        rotation_count[i]--;
                    else if (diff < -HALF_ENCODER)
                        rotation_count[i]++;
                }
                last_encoder[i] = enc;

                /* ========= 累積エンコーダ ========= */
                total_encoder[i] = rotation_count[i] * ENCODER_MAX + enc;

                /* ========= 物理量変換 ========= */
                angle_m3508[i] = total_encoder[i] * (360.0f / (ENCODER_MAX * gear_m3508));

                vel_m3508[i] = (float)rpm_raw / gear_m3508;

                c[i] = (float)cur_raw * 20.0f / 16384.0f;
            }
        }

        /* CPUを占有しないように */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// 複数モータ対応CAN送信関数
void send_cur_all(float cur_array[NUM_MOTOR])
{
    constexpr float MAX_CUR = 10;
    constexpr int MAX_CUR_VAL = 1000;
    uint8_t send_data[8] = {};

    for (int i = 0; i < NUM_MOTOR; i++)
    {
        float val = cur_array[i] * (MAX_CUR_VAL / MAX_CUR);
        if (val < -MAX_CUR_VAL)
            val = -MAX_CUR_VAL;
        if (val > MAX_CUR_VAL)
            val = MAX_CUR_VAL;

        int16_t transmit_val = val;
        send_data[i * 2] = (transmit_val >> 8) & 0xFF;
        send_data[i * 2 + 1] = transmit_val & 0xFF;
    }

    CAN.beginPacket(0x200);
    CAN.write(send_data, 8);
    CAN.endPacket();
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
