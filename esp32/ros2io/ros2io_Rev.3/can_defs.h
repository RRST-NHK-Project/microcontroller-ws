#pragma once

#include "can_defs.h"
#include "defs.h"
#include "input_task.h"
#include "ros_defs.h"
#include <Arduino.h>
#include <CAN.h>
//  パルスカウンタ関連
#include "driver/pcnt.h"

// ********* CAN関連 ********* //

// -------- 定数 -------- //
constexpr int ENCODER_MAX = 8192;             // エンコーダの最大値
constexpr int HALF_ENCODER = ENCODER_MAX / 2; // エンコーダ半回転値
constexpr float gear_ratio = 36.0f;           // 減速比
constexpr int NUM_MOTORS = 4;                 // モータ数
extern float current_limit_A;

// -------- 状態量 / CAN受信関連 -------- //
extern int encoders[NUM_MOTORS];       // エンコーダ値
extern int rpms[NUM_MOTORS];           // 回転速度
extern int currents[NUM_MOTORS];       // 電流値
extern bool offset_ok[NUM_MOTORS];     // オフセット完了フラグ
extern int encoder_offset[NUM_MOTORS]; // エンコーダオフセット
extern int last_encoder[NUM_MOTORS];   // 前回エンコーダ値
extern int rotation_count[NUM_MOTORS]; // 回転数
extern long total_encoder[NUM_MOTORS]; // 累積エンコーダ値
extern float angles[NUM_MOTORS];       // 角度
extern float vels[NUM_MOTORS];         // 速度

// -------- PID関連変数 -------- //
extern float target_angle[NUM_MOTORS];         // 目標角度
extern float pos_error_prev[NUM_MOTORS];       // 前回角度誤差
extern float pos_integral[NUM_MOTORS];         // 角度積分項
extern float pos_output[NUM_MOTORS];           // PID出力
extern float motor_output_current[NUM_MOTORS]; // 出力電流

extern unsigned long lastPidTime; // PID制御用タイマー

// -------- PIDゲイン -------- //
extern float kp_pos; // 角度比例ゲイン
extern float ki_pos; // 角度積分ゲイン
extern float kd_pos; // 角度微分ゲイン

// 複数モータ対応CAN送信関数
void send_cur_all(float cur_array[NUM_MOTORS]);

// PID計算関数（単独モータ用だが複数モータでループ使用可能）
float pid(float setpoint, float input, float &error_prev, float &integral,
          float kp, float ki, float kd, float dt);

// 値制限関数
float constrain_double(float val, float min_val, float max_val);

// ********* CAN関連ここまで ********* //

void ROBOMAS_ENC_SW_Read_Publish_Task(void *pvParameters);

void C610_Task(void *pvParameters);

void C610_FB_Task(void *pvParameters);
