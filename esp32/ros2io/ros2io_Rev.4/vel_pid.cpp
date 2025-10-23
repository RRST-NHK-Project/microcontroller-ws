#include "vel_pid.h"
#include "defs.h"
#include "input_task.h"
#include <Arduino.h>
#include <CAN.h>
//  パルスカウンタ関連
#include "driver/pcnt.h"

// ********* CAN関連 ********* //

// -------- 状態量 / CAN受信関連 -------- //
int encoders[NUM_MOTORS] = {0};       // エンコーダ値
int rpms[NUM_MOTORS] = {0};           // 回転速度
int currents[NUM_MOTORS] = {0};       // 電流値
bool offset_ok[NUM_MOTORS] = {false}; // オフセット完了フラグ
int encoder_offset[NUM_MOTORS] = {0}; // エンコーダオフセット
int last_encoder[NUM_MOTORS];         // 前回エンコーダ値
int rotation_count[NUM_MOTORS] = {0}; // 回転数
long total_encoder[NUM_MOTORS] = {0}; // 累積エンコーダ値
float angles[NUM_MOTORS] = {0};       // 角度
float vels[NUM_MOTORS] = {0};         // 速度

// -------- PID関連変数 -------- //
float target_angle[NUM_MOTORS] = {0};         // 目標角度
float pos_error_prev[NUM_MOTORS] = {0};       // 前回角度誤差
float pos_integral[NUM_MOTORS] = {0};         // 角度積分項
float pos_output[NUM_MOTORS] = {0};           // PID出力
float vel_out[NUM_MOTORS] = {0};           // PID出力(速度)
float vel_prop_prev[NUM_MOTORS] = {0};     // 速度比例項の前回値
float motor_output_current[NUM_MOTORS] = {0}; // 出力電流
float vel_output[NUM_MOTORS] = {0};           // 速度PID出力

unsigned long lastPidTime = 0; // PID制御用タイマー

// -------- PIDゲイン -------- //
float kp_vel = 1.0;
float ki_vel = 0.0;
float kd_vel = 0.05;

// 複数モータ対応CAN送信関数
void send_cur_all(float cur_array[NUM_MOTORS]) {
    constexpr float MAX_CUR = 10;
    constexpr int MAX_CUR_VAL = 10000;
    uint8_t send_data[8] = {};

    for (int i = 0; i < NUM_MOTORS; i++) {
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

// PID計算関数（単独モータ用だが複数モータでループ使用可能）
// float pid(float setpoint, float input, float &error_prev, float &integral,
//           float kp, float ki, float kd, float dt) {
//     float error = setpoint - input;
//     integral += ((error + error_prev) * dt / 2.0f); // 台形積分
//     float derivative = (error - error_prev) / dt;
//     error_prev = error;
//     return kp * error + ki * integral + kd * derivative;
// }

//速度制御用PID(普通のPIDだったら振動しているのでこれ推奨)
float pid_vel(float setpoint, float input, float &error_prev,float &prop_prev, float &output,
          float kp, float ki, float kd, float dt){
            float error = setpoint - input;
            float prop = error - error_prev;
            float deriv = prop - prop_prev;
            float du = kp * prop + ki * error * dt + kd * deriv;
            output += du;

            prop_prev = prop;
            error_prev = error;

            return output;
        }

// 値制限関数(正直これはいらんかも)
float constrain_double(float val, float min_val, float max_val) {
    if (val < min_val)
        return min_val;
    if (val > max_val)
        return max_val;
    return val;
}

// ********* CAN関連ここまで ********* //

void ROBOMAS_ENC_SW_Read_Publish_Task(void *pvParameters) {
    // 初期化
    for (int i = 0; i < NUM_MOTORS; i++) {
        last_encoder[i] = -1;
    }
    while (1) {

        // パルスカウンタの値を取得
        pcnt_get_counter_value(PCNT_UNIT_0, &count[0]);
        pcnt_get_counter_value(PCNT_UNIT_1, &count[1]);
        pcnt_get_counter_value(PCNT_UNIT_2, &count[2]);
        pcnt_get_counter_value(PCNT_UNIT_3, &count[3]);

        // スイッチの状態を取得
        sw_state[0] = (digitalRead(SW1) == HIGH);
        sw_state[1] = (digitalRead(SW2) == HIGH);
        sw_state[2] = (digitalRead(SW3) == HIGH);
        sw_state[3] = (digitalRead(SW4) == HIGH);

        // ----一旦応急処置---- //
        // 2025/09/18 gptのコードを、gptを使って読みやすく書き換えました。そのうち書き直す、、、、はず

        // CAN受信
        int packetSize = CAN.parsePacket();
        while (packetSize) {
            int id = CAN.packetId();

            if (id >= 0x201 && id < 0x201 + NUM_MOTORS) {
                int idx = id - 0x201;
                uint8_t rx[8];
                for (int i = 0; i < 8; i++)
                    rx[i] = CAN.read();

                encoders[idx] = (rx[0] << 8) | rx[1];
                rpms[idx] = (rx[2] << 8) | rx[3];
                currents[idx] = (rx[4] << 8) | rx[5];

                // 初回オフセット設定
                if (!offset_ok[idx]) {
                    encoder_offset[idx] = encoders[idx];
                    last_encoder[idx] = -1;
                    rotation_count[idx] = 0;
                    total_encoder[idx] = 0;
                    offset_ok[idx] = true;
                }

                int enc_rel = encoders[idx] - encoder_offset[idx];
                if (enc_rel < 0)
                    enc_rel += ENCODER_MAX;

                if (last_encoder[idx] != -1) {
                    int diff = encoders[idx] - last_encoder[idx];
                    if (diff > HALF_ENCODER)
                        rotation_count[idx]--;
                    else if (diff < -HALF_ENCODER)
                        rotation_count[idx]++;
                }

                last_encoder[idx] = encoders[idx];
                total_encoder[idx] = rotation_count[idx] * ENCODER_MAX + encoders[idx];
                angles[idx] = total_encoder[idx] * (360.0 / (8192.0 * gear_ratio));
                vels[idx] = rpms[idx] / gear_ratio;
            }

            packetSize = CAN.parsePacket();
        }

        msg.data.data[0] = count[0];
        msg.data.data[1] = count[1];
        msg.data.data[2] = count[2];
        msg.data.data[3] = count[3];
        msg.data.data[4] = sw_state[0];
        msg.data.data[5] = sw_state[1];
        msg.data.data[6] = sw_state[2];
        msg.data.data[7] = sw_state[3];
        msg.data.data[8] = angles[0];
        msg.data.data[9] = angles[1];
        msg.data.data[10] = angles[2];
        msg.data.data[11] = angles[3];
        msg.data.data[12] = rpms[0];
        msg.data.data[13] = rpms[1];
        msg.data.data[14] = rpms[2];
        msg.data.data[15] = rpms[3];
        msg.data.data[16] = currents[0];
        msg.data.data[17] = currents[1];
        msg.data.data[18] = currents[2];
        msg.data.data[19] = currents[3];
        // ----応急処置ここまで---- //

        // msg.data.data[0] = count[0];
        // msg.data.data[1] = count[1];
        // msg.data.data[2] = count[2];
        // msg.data.data[3] = count[3];
        // msg.data.data[4] = sw_state[0];
        // msg.data.data[5] = sw_state[1];
        // msg.data.data[6] = sw_state[2];
        // msg.data.data[7] = sw_state[3];

        // Publish
        if (MODE != 0) {
            RCCHECK(rcl_publish(&publisher, &msg, NULL));
        }

        vTaskDelay(1); // ウォッチドッグタイマのリセット(必須)
    }
}

void C610_vel_Task(void *pvParameters) {
    while (1) {
        unsigned long now = millis();
        float dt = (now - lastPidTime) / 1000.0f;
        if (dt <= 0)
            dt = 0.000001f; // dtが0にならないよう補正
        lastPidTime = now;

        // -------- 目標角度の更新 -------- //
        // received_data[1]～[4] にモータ1～4の目標角度が入っている前提
        for (int i = 0; i < NUM_MOTORS; i++) {
            target_angle[i] = received_data[i + 1];
        }

        // -------- CAN受信処理 -------- //
        int packetSize = CAN.parsePacket();
        while (packetSize) {
            int id = CAN.packetId();
            if (id >= 0x201 && id < 0x201 + NUM_MOTORS) {
                int motor_index = id - 0x201;
                uint8_t rx[8];
                for (int i = 0; i < 8; i++)
                    rx[i] = CAN.read();

                // エンコーダ・速度・電流取得
                encoders[motor_index] = (rx[0] << 8) | rx[1];
                rpms[motor_index] = (rx[2] << 8) | rx[3];
                currents[motor_index] = (rx[4] << 8) | rx[5];

                // 初回オフセット設定
                if (!offset_ok[motor_index]) {
                    encoder_offset[motor_index] = encoders[motor_index];
                    last_encoder[motor_index] = -1;
                    rotation_count[motor_index] = 0;
                    total_encoder[motor_index] = 0;
                    pos_integral[motor_index] = 0;
                    pos_error_prev[motor_index] = 0;
                    offset_ok[motor_index] = true;
                }

                // エンコーダ差分とラップ補正
                int enc_relative = encoders[motor_index] - encoder_offset[motor_index];
                if (enc_relative < 0)
                    enc_relative += ENCODER_MAX;

                if (last_encoder[motor_index] != -1) {
                    int diff = encoders[motor_index] - last_encoder[motor_index];
                    if (diff > HALF_ENCODER)
                        rotation_count[motor_index]--;
                    else if (diff < -HALF_ENCODER)
                        rotation_count[motor_index]++;
                }

                last_encoder[motor_index] = encoders[motor_index];
                total_encoder[motor_index] = rotation_count[motor_index] * ENCODER_MAX + encoders[motor_index];
                angles[motor_index] = total_encoder[motor_index] * (360.0f / (ENCODER_MAX * gear_ratio));
                vels[motor_index] = (rpms[motor_index] / gear_ratio) * 360.0f / 60.0f;
            }

            packetSize = CAN.parsePacket(); // 次の受信も処理
        }

        // -------- PID制御（全モータ） -------- //
        for (int i = 0; i < NUM_MOTORS; i++) {
            pos_output[i] = pid(target_angle[i], angles[i], pos_error_prev[i], pos_integral[i],
                                kp_pos, ki_pos, kd_pos, dt);
            motor_output_current[i] = constrain_double(pos_output[i], -current_limit_A, current_limit_A);
        }

        // -------- CAN送信（全モータ） -------- //
        send_cur_all(motor_output_current);

        delay(1);
    }
}
