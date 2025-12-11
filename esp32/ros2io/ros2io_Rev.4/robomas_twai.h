#pragma once

#include "defs.h"
#include "input_task.h"
#include "ros_defs.h"
#include <Arduino.h>
//  パルスカウンタ関連
#include "driver/pcnt.h"

#include "driver/gpio.h"
#include "driver/twai.h"

// twai.h, twai.cppは名称が不適切なので変更してください。ex) c620_twai.h など

// ********* CAN関連 ********* //

// -------- 定数 -------- //
constexpr float gear_m3508 = 19.2f; // 減速比(m3508用)

// -------- PID関連変数 -------- //
extern float target_rpm[NUM_MOTOR];     // 目標速度
extern float vel_error_prev[NUM_MOTOR]; // 前回速度誤差
extern float vel_prop_prev[NUM_MOTOR];  // 速度比例項前回値

extern float vel_output[NUM_MOTOR]; // 速度PID出力
extern float vel_out[NUM_MOTOR];    // 最終速度出力

// -------- 定数 -------- //
constexpr int ENCODER_MAX = 8192;             // エンコーダの最大値
constexpr int HALF_ENCODER = ENCODER_MAX / 2; // エンコーダ半回転値
constexpr float gear_ratio = 36.0f;           // 減速比(m3508用)
extern float current_limit_A;

// -------- 状態量 / CAN受信関連 -------- //
extern int encoder_count[NUM_MOTOR];  // エンコーダ値
extern int rpm[NUM_MOTOR];            // 回転速度
extern int current[NUM_MOTOR];        // 電流値
extern bool offset_ok[NUM_MOTOR];     // オフセット完了フラグ
extern int encoder_offset[NUM_MOTOR]; // エンコーダオフセット
extern int last_encoder[NUM_MOTOR];   // 前回エンコーダ値
extern int rotation_count[NUM_MOTOR]; // 回転数
extern long total_encoder[NUM_MOTOR]; // 累積エンコーダ値
extern float angle[NUM_MOTOR];        // 角度
extern float vel[NUM_MOTOR];          // 速度

// -------- PID関連変数 -------- //
extern float target_angle[NUM_MOTOR];   // 目標角度
extern float pos_error_prev[NUM_MOTOR]; // 前回角度誤差
extern float cur_error_prev[NUM_MOTOR]; // 前回角度誤差
extern float pos_integral[NUM_MOTOR];   // 角度積分項
extern float vel_integral[NUM_MOTOR];
extern float cur_integral[NUM_MOTOR];
extern float pos_output[NUM_MOTOR];           // PID出力
extern float cur_output[NUM_MOTOR];           // PID出力
extern float motor_output_current[NUM_MOTOR]; // 出力電流

extern float angle_m3508[NUM_MOTOR];
extern float vel_m3508[NUM_MOTOR];
extern float c[NUM_MOTOR];
extern float output[NUM_MOTOR];

extern unsigned long lastPidTime; // PID制御用タイマー

// -------- PIDゲイン -------- //
extern float kp_pos; // 角度比例ゲイン
extern float ki_pos; // 角度積分ゲイン
extern float kd_pos; // 角度微分ゲイン

extern float kp_vel;
extern float ki_vel;
extern float kd_vel; // 速度微分ゲイン

extern float kp_cur;
extern float ki_cur;
extern float kd_cur; // 速度微分ゲイン

// 速度PID計算関数
float pid_vel(float setpoint, float input, float &error_prev, float &prop_prev, float &output,
              float kp, float ki, float kd, float dt);

// PID計算関数（単独モータ用だが複数モータでループ使用可能）
float pid(float setpoint, float input, float &error_prev, float &integral,
          float kp, float ki, float kd, float dt);

// 値制限関数
float constrain_double(float val, float min_val, float max_val);

void send_cur_twai(float cur_array[NUM_MOTOR]);

///
void twai_receive_feedback(void *pvParameters);

void C620_twai(void *pvParameters);

void C620_twai_FB(void *pvParameters);
