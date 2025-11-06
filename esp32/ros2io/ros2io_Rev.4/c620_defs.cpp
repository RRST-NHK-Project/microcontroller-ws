/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "c620_defs.h"
#include "can_defs.h"
#include "defs.h"
#include "input_task.h"
#include <Arduino.h>
#include <CAN.h>
//  パルスカウンタ関連
#include "driver/pcnt.h"

// ********* CAN関連 ********* //

// -------- 状態量 / CAN受信関連 -------- //
float angle_m3508[NUM_MOTOR] = {0}; // 角度
float vel_m3508[NUM_MOTOR] = {0};   // 速度

// -------- PID関連変数 -------- //
float target_rpm[NUM_MOTOR] = {0};     // 目標速度
float vel_error_prev[NUM_MOTOR] = {0}; // 前回速度誤差
float vel_prop_prev[NUM_MOTOR] = {0};  // 速度比例項
float vel_output[NUM_MOTOR] = {0};     // 速度PID出力
float vel_out[NUM_MOTOR] = {0};        // 最終速度出力

// -------- PIDゲイン -------- //
float kp_vel = 0.3;
float ki_vel = 0.0;
float kd_vel = 0.05; // 微分は控えめに

// 速度PID計算関数
float pid_vel(float setpoint, float input, float &error_prev, float &prop_prev, float &output,
              float kp, float ki, float kd, float dt) {
    float error = setpoint - input;
    float prop = error - error_prev;
    float deriv = prop - prop_prev;
    float du = kp * prop + ki * error * dt + kd * deriv;
    output += du;

    prop_prev = prop;
    error_prev = error;

    return output;
}

// ********* CAN関連ここまで ********* //

void C620_Task(void *pvParameters) {
    while (1) {
        unsigned long now = millis();
        float dt = (now - lastPidTime) / 1000.0f;
        if (dt <= 0)
            dt = 0.000001f; // dtが0にならないよう補正
        lastPidTime = now;

        // -------- 目標角度の更新 -------- //
        // received_data[1]～[4] にモータ1～4の目標角度が入っている前提
        for (int i = 0; i < NUM_MOTOR; i++) {
            target_rpm[i] = received_data[i + 1];
        }

        // -------- CAN受信処理 -------- //
        int packetSize = CAN.parsePacket();
        while (packetSize) {
            int id = CAN.packetId();
            if (id >= 0x201 && id < 0x201 + NUM_MOTOR) {
                int motor_index = id - 0x201;
                uint8_t rx[8];
                for (int i = 0; i < 8; i++)
                    rx[i] = CAN.read();

                // エンコーダ・速度・電流取得
                encoder_count[motor_index] = (rx[0] << 8) | rx[1];
                rpm[motor_index] = (rx[2] << 8) | rx[3];
                current[motor_index] = (rx[4] << 8) | rx[5];

                // 初回オフセット設定
                if (!offset_ok[motor_index]) {
                    encoder_offset[motor_index] = encoder_count[motor_index];
                    last_encoder[motor_index] = -1;
                    rotation_count[motor_index] = 0;
                    total_encoder[motor_index] = 0;
                    pos_integral[motor_index] = 0;
                    pos_error_prev[motor_index] = 0;
                    offset_ok[motor_index] = true;
                }

                // エンコーダ差分とラップ補正
                int enc_relative = encoder_count[motor_index] - encoder_offset[motor_index];
                if (enc_relative < 0)
                    enc_relative += ENCODER_MAX;

                if (last_encoder[motor_index] != -1) {
                    int diff = encoder_count[motor_index] - last_encoder[motor_index];
                    if (diff > HALF_ENCODER)
                        rotation_count[motor_index]--;
                    else if (diff < -HALF_ENCODER)
                        rotation_count[motor_index]++;
                }

                last_encoder[motor_index] = encoder_count[motor_index];
                total_encoder[motor_index] = rotation_count[motor_index] * ENCODER_MAX + encoder_count[motor_index];
                angle_m3508[motor_index] = total_encoder[motor_index] * (360.0f / (ENCODER_MAX * gear_m3508));
                vel_m3508[motor_index] = (rpm[motor_index] / gear_m3508);
            }

            packetSize = CAN.parsePacket(); // 次の受信も処理
        }

        // -------- PID制御（全モータ） -------- //
        for (int i = 0; i < NUM_MOTOR; i++) {
            // pos_output[i] = pid(target_angle, angle[i], pos_error_prev[i], pos_integral[i], kp_pos, ki_pos, kd_pos, dt);
            vel_out[i] = pid_vel(target_rpm[i], vel_m3508[i], vel_error_prev[i], vel_prop_prev[i], vel_output[i], kp_vel, ki_vel, kd_vel, dt);

            motor_output_current[i] = vel_out[i]; // constrain_double(pos_output, -current_limit_A, current_limit_A);
        }
        // -------- CAN送信（全モータ） -------- //
        send_cur_all(motor_output_current);

        vTaskDelay(1);
    }
}
