/*====================================================================
<can_defs.cpp>
・CAN関連の変数や関数の実装ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "can_defs.h"
#include "defs.h"
#include "input_task.h"
#include <Arduino.h>
#include <CAN.h>
//  パルスカウンタ関連
#include "driver/pcnt.h"

// ********* CAN関連 ********* //

// -------- 状態量 / CAN受信関連 -------- //
int encoder_count[NUM_MOTOR] = {0};  // エンコーダ値
int rpm[NUM_MOTOR] = {0};            // 回転速度
int current[NUM_MOTOR] = {0};        // 電流値
bool offset_ok[NUM_MOTOR] = {false}; // オフセット完了フラグ
int encoder_offset[NUM_MOTOR] = {0}; // エンコーダオフセット
int last_encoder[NUM_MOTOR];         // 前回エンコーダ値
int rotation_count[NUM_MOTOR] = {0}; // 回転数
long total_encoder[NUM_MOTOR] = {0}; // 累積エンコーダ値
float angle[NUM_MOTOR] = {0};        // 角度
float vel[NUM_MOTOR] = {0};          // 速度

// -------- PID関連変数 -------- //
float target_angle[NUM_MOTOR] = {0};         // 目標角度
float pos_error_prev[NUM_MOTOR] = {0};       // 前回角度誤差
float pos_integral[NUM_MOTOR] = {0};         // 角度積分項
float vel_integral[NUM_MOTOR] = {0};
float pos_output[NUM_MOTOR] = {0};           // PID出力
float motor_output_current[NUM_MOTOR] = {0}; // 出力電流
float output[NUM_MOTOR] = {0};

//速度PID
float target_rpm[NUM_MOTOR] = {0};         // 目標速度
float vel_error_prev[NUM_MOTOR] = {0};       // 前回速度誤差
float vel_prop_prev[NUM_MOTOR] = {0};         // 速度比例項
float vel_output[NUM_MOTOR] = {0};           // 速度PID出力
float vel_out[NUM_MOTOR] = {0};           // 最終速度出力


unsigned long lastPidTime = 0; // PID制御用タイマー

// -------- PIDゲイン -------- //
float kp_pos = 0.8f;  // 角度比例ゲイン
float ki_pos = 0.01f; // 角度積分ゲイン
float kd_pos = 0.02f; // 角度微分ゲイン

// -------- 速度PIDゲイン -------- //
float kp_vel = 0.8;
float ki_vel = 0.0;
float kd_vel = 0.05;  // 微分は控えめに

// 複数モータ対応CAN送信関数
void send_cur_all(float cur_array[NUM_MOTOR]) {
    constexpr float MAX_CUR = 10;
    constexpr int MAX_CUR_VAL = 1000;
    uint8_t send_data[8] = {};

    for (int i = 0; i < NUM_MOTOR; i++) {
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
float pid(float setpoint, float input, float &error_prev, float &integral,
          float kp, float ki, float kd, float dt) {
    float error = setpoint - input;
    integral += ((error + error_prev) * dt / 2.0f); // 台形積分
    float derivative = (error - error_prev) / dt;
    error_prev = error;
    return kp * error + ki * integral + kd * derivative;
}

//速度PID計算関数
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
    for (int i = 0; i < NUM_MOTOR; i++) {
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

            if (id >= 0x201 && id < 0x201 + NUM_MOTOR) {
                int idx = id - 0x201;
                uint8_t rx[8];
                for (int i = 0; i < 8; i++)
                    rx[i] = CAN.read();

                encoder_count[idx] = (rx[0] << 8) | rx[1];
                rpm[idx] = (rx[2] << 8) | rx[3];
                current[idx] = (rx[4] << 8) | rx[5];

                // 初回オフセット設定
                if (!offset_ok[idx]) {
                    encoder_offset[idx] = encoder_count[idx];
                    last_encoder[idx] = -1;
                    rotation_count[idx] = 0;
                    total_encoder[idx] = 0;
                    offset_ok[idx] = true;
                }
                // エンコーダ差分とラップ補正

                int enc_rel = encoder_count[idx] - encoder_offset[idx];
                if (enc_rel < 0)
                    enc_rel += ENCODER_MAX;

                if (last_encoder[idx] != -1) {
                    int diff = encoder_count[idx] - last_encoder[idx];
                    if (diff > HALF_ENCODER)
                        rotation_count[idx]--;
                    else if (diff < -HALF_ENCODER)
                        rotation_count[idx]++;
                }

                last_encoder[idx] = encoder_count[idx];
                total_encoder[idx] = rotation_count[idx] * ENCODER_MAX + encoder_count[idx];
                angle[idx] = total_encoder[idx] * (360.0 / (8192.0 * gear_ratio));
                vel[idx] = rpm[idx] / gear_ratio;
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
        msg.data.data[8] = angle[0];
        msg.data.data[9] = angle[1];
        msg.data.data[10] = angle[2];
        msg.data.data[11] = angle[3];
        msg.data.data[12] = rpm[0];
        msg.data.data[13] = rpm[1];
        msg.data.data[14] = rpm[2];
        msg.data.data[15] = rpm[3];
        msg.data.data[16] = current[0];
        msg.data.data[17] = current[1];
        msg.data.data[18] = current[2];
        msg.data.data[19] = current[3];
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



void C610_FB_Task(void *pvParameters) {
    for (int i = 0; i < NUM_MOTOR; i++) {
        last_encoder[i] = -1;
    }
    while (1) {
        unsigned long now = millis();
        float dt = (now - lastPidTime) / 1000.0f;
        if (dt <= 0)
            dt = 0.000001f; // dtが0にならないよう補正
        lastPidTime = now;

        // -------- 目標角度の更新 -------- //
        // received_data[1]～[4] にモータ1～4の目標角度が入っている前提
        for (int i = 0; i < NUM_MOTOR; i++) {
            target_angle[i] = received_data[i + 1];
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
                angle[motor_index] = total_encoder[motor_index] * (360.0f / (ENCODER_MAX * gear_ratio));
                vel[motor_index] = (rpm[motor_index] / gear_ratio) * 360.0f / 60.0f;
            }

            packetSize = CAN.parsePacket(); // 次の受信も処理
        }

        // -------- PID制御（全モータ） -------- //
        for (int i = 0; i < NUM_MOTOR; i++) {
            pos_output[i] = pid(target_angle[i], angle[i], pos_error_prev[i], pos_integral[i],
                                kp_pos, ki_pos, kd_pos, dt);
            motor_output_current[i] = constrain_double(pos_output[i], -current_limit_A, current_limit_A);
        }

        // -------- CAN送信（全モータ） -------- //
        send_cur_all(motor_output_current);

        msg.data.data[0] = count[0];
        msg.data.data[1] = count[1];
        msg.data.data[2] = count[2];
        msg.data.data[3] = count[3];
        msg.data.data[4] = sw_state[0];
        msg.data.data[5] = sw_state[1];
        msg.data.data[6] = sw_state[2];
        msg.data.data[7] = sw_state[3];
        msg.data.data[8] = angle[0];
        msg.data.data[9] = angle[1];
        msg.data.data[10] = angle[2];
        msg.data.data[11] = angle[3];
        msg.data.data[12] = rpm[0];
        msg.data.data[13] = rpm[1];
        msg.data.data[14] = rpm[2];
        msg.data.data[15] = rpm[3];
        msg.data.data[16] = current[0];
        msg.data.data[17] = current[1];
        msg.data.data[18] = current[2];
        msg.data.data[19] = current[3];

        delay(1);
    }
}


void CR25_Task(void *pvParameters) {
    while (1) {

        sw_state[0] = (digitalRead(26) == HIGH);
        sw_state[1] = (digitalRead(27) == HIGH);

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
                angle[motor_index] = total_encoder[motor_index] * (360.0f / (ENCODER_MAX * gear_ratio));
                vel[motor_index] = (rpm[motor_index] / gear_ratio);
            }

            packetSize = CAN.parsePacket(); // 次の受信も処理
        }
        unsigned long now = millis();
        float dt = (now - lastPidTime) / 1000.0f;
        if (dt <= 0)
            dt = 0.000001f; // dtが0にならないよう補正
        lastPidTime = now;


       // -------- PID制御（全モータ） -------- //
        for (int i = 0; i < NUM_MOTOR; i++) {
            output[i] = received_data[i + 1]*0.01;
            if(!sw_state[0]){
               output[0] = -output[0]*3;
               output[1] = -output[1]*3; 
            }
             if(!sw_state[1]){
               output[0] = -output[0]*3;
               output[1] = -output[1]*3; 
            }
            motor_output_current[i] = output[i];//constrain_double(output[i], -current_limit_A, current_limit_A);
            
        }
        // output[2] = received_data[3]*0.1;
        // if(!sw_state[0]){
        //        output[2] = -output[2];
        //     }
        // motor_output_current[2] = output[2];
        
       
        //motor_output_current[2]=constrain_double(output, -0.5, 0.5);
       static unsigned long lastSerial = 0;
        if (now - lastSerial > 50) {  // 20Hzくらい
            lastSerial = now;
            Serial1.printf("%.2f\t%.2f\t%.2f\n", current[0], current[1], current[2]);;
        }

   
        
        // -------- CAN送信（全モータ） -------- //
        send_cur_all(motor_output_current);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void CAN_Pb(void *pvParameters) {
   while(1) {
      if (MODE != 0) {
            RCCHECK(rcl_publish(&publisher, &msg, NULL));
        }

        vTaskDelay(1); 
   }
}


void SV_Task(void *pvParameters) {
    while (1) {

      // サーボ1
        int angle1 = 90+received_data[9];
        if (angle1 < SERVO1_MIN_DEG)
            angle1 = SERVO1_MIN_DEG;
        if (angle1 > SERVO1_MAX_DEG)
            angle1 = SERVO1_MAX_DEG;
        int us1 = map(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG, SERVO1_MIN_US, SERVO1_MAX_US);
        int duty1 = (int)(us1 * SERVO_PWM_SCALE);
        ledcWrite(16, duty1);

        digitalWrite(SV1, received_data[17] ? HIGH : LOW);
        
          if (MODE != 0) {
           rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        vTaskDelay(1);// ウォッチドッグタイマのリセット(必須)    
          }
   }    
}


