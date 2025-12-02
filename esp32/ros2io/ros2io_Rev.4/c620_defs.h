/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

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
constexpr float gear_m3508 = 19.2f; // 減速比(m3508用)

// -------- PID関連変数 -------- //
extern float target_rpm[NUM_MOTOR];     // 目標速度
extern float vel_error_prev[NUM_MOTOR]; // 前回速度誤差
extern float vel_prop_prev[NUM_MOTOR];  // 速度比例項前回値

extern float vel_output[NUM_MOTOR]; // 速度PID出力
extern float vel_out[NUM_MOTOR];    // 最終速度出力



// 速度PID計算関数
float pid_vel(float setpoint, float input, float &error_prev, float &prop_prev, float &output,
              float kp, float ki, float kd, float dt);

// ********* CAN関連ここまで ********* //

void C620_Task(void *pvParameters);

void C620_Task_v2(void *pvParameters);

void C620_debug(void *pvParameters);
