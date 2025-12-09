#include "robomas_twai.h"
#include "defs.h"
#include "input_task.h"
#include <Arduino.h>
//  パルスカウンタ関連
#include "driver/pcnt.h"


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
float target_angle[NUM_MOTOR] = {0};   // 目標角度
float pos_error_prev[NUM_MOTOR] = {0}; // 前回角度誤差
float cur_error_prev[NUM_MOTOR] = {0}; // 前回角度誤差
float pos_integral[NUM_MOTOR] = {0};   // 角度積分項
float vel_integral[NUM_MOTOR] = {0};
float cur_integral[NUM_MOTOR] = {0};
float pos_output[NUM_MOTOR] = {0};           // PID出力
float cur_output[NUM_MOTOR] = {0};           // PID出力
float motor_output_current[NUM_MOTOR] = {0}; // 出力電流
float output[NUM_MOTOR] = {0};

// 速度PID
float target_rpm[NUM_MOTOR] = {0};     // 目標速度
float vel_error_prev[NUM_MOTOR] = {0}; // 前回速度誤差
float vel_prop_prev[NUM_MOTOR] = {0};  // 速度比例項
float vel_output[NUM_MOTOR] = {0};     // 速度PID出力
float vel_out[NUM_MOTOR] = {0};        // 最終速度出力

// -------- 状態量 / CAN受信関連 -------- //
float angle_m3508[NUM_MOTOR] = {0}; // 角度
float vel_m3508[NUM_MOTOR] = {0};   // 速度
float c[NUM_MOTOR] = {0};           //

unsigned long lastPidTime = 0; // PID制御用タイマー

// -------- PIDゲイン -------- //
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

void send_cur_twai(float cur_array[NUM_MOTOR])
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

// PID計算関数（単独モータ用だが複数モータでループ使用可能）
float pid(float setpoint, float input, float &error_prev, float &integral,
          float kp, float ki, float kd, float dt)
{
    float error = setpoint - input;
    integral += ((error + error_prev) * dt / 2.0f); // 台形積分
    float derivative = (error - error_prev) / dt;
    error_prev = error;
    return kp * error + ki * integral + kd * derivative;
}

// 速度PID計算関数
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

// 値制限関数(正直これはいらんかも)
float constrain_double(float val, float min_val, float max_val)
{
    if (val < min_val)
        return min_val;
    if (val > max_val)
        return max_val;
    return val;
}

void twai_receive_feedback()
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
}

void C620_twai(void *pvParameters)
{
    while (1)
    {
        for (int i = 0; i < NUM_MOTOR; i++)
        {
            target_rpm[i] = received_data[i + 1];
        }
        unsigned long now = millis();
        float dt = (now - lastPidTime) / 1000.0f;
        if (dt <= 0)
            dt = 0.000001f;
        if (dt > 0.02f)
            dt = 0.02f;
        lastPidTime = now;

        // TWAI受信処理
        twai_receive_feedback();

        // // 5秒後にターゲットRPM増加
        for (int i = 0; i < NUM_MOTOR; i++)
        {
            //     static float rpm_step[NUM_MOTOR] = {100, 100, 100, 100};

            //     if (now < 5000)
            //     {
            //         target_rpm[i] = 0;
            //     }
            //     else
            //     {
            //         if (rpm_step[i] < 300)
            //             rpm_step[i] += 0.1;
            //         target_rpm[i] = rpm_step[i];
            //     }

            vel_out[i] = pid_vel(target_rpm[i], vel_m3508[i], vel_error_prev[i], vel_prop_prev[i], vel_output[i], kp_vel, ki_vel, kd_vel, dt);

            motor_output_current[i] = constrain_double(vel_out[i] * 10, -20, 20);
        }

        // 送信
        send_cur_twai(motor_output_current);

        // debug
        // Serial.print(vel_m3508[0]);
        // Serial.print("\t");
        // Serial.println(current[0]);

        vTaskDelay(1);
    }
}
