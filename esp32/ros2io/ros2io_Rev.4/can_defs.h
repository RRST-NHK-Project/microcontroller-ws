/*====================================================================
<can_defs.h>
・CAN関連の変数や関数の定義ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

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
constexpr float gear_ratio = 36.0f;           // 減速比(m3508用)
constexpr int NUM_MOTOR = 4;                  // モータ数
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
extern float target_angle[NUM_MOTOR];         // 目標角度
extern float pos_error_prev[NUM_MOTOR];       // 前回角度誤差
extern float pos_integral[NUM_MOTOR];         // 角度積分項
extern float vel_integral[NUM_MOTOR];
extern float pos_output[NUM_MOTOR];           // PID出力
extern float motor_output_current[NUM_MOTOR]; // 出力電流

extern unsigned long lastPidTime; // PID制御用タイマー

// -------- PIDゲイン -------- //
extern float kp_pos; // 角度比例ゲイン
extern float ki_pos; // 角度積分ゲイン
extern float kd_pos; // 角度微分ゲイン

// 複数モータ対応CAN送信関数
void send_cur_all(float cur_array[NUM_MOTOR]);

// PID計算関数（単独モータ用だが複数モータでループ使用可能）
float pid(float setpoint, float input, float &error_prev, float &integral,
          float kp, float ki, float kd, float dt);

// 値制限関数
float constrain_double(float val, float min_val, float max_val);

// ********* CAN関連ここまで ********* //

void ROBOMAS_ENC_SW_Read_Publish_Task(void *pvParameters);

void C610_FB_Task(void *pvParameters);

void CR25_Task(void *pvParameters);


void CAN_Pb(void *pvParameters);

void SV_Task(void *pvParameters);
