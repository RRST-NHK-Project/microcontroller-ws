#pragma once

#include "defs.h"
#include "can_defs.h"
#include "input_task.h"
#include "ros_defs.h"
#include <Arduino.h>
#include <CAN.h>
//  パルスカウンタ関連
#include "driver/pcnt.h"

// ********* CAN関連 ********* //

// -------- 定数 -------- //
constexpr float gear_m3508 = 19.2f;           // 減速比(m3508用)

// -------- PID関連変数 -------- //
extern float target_rpm[NUM_MOTOR];         // 目標速度
extern float vel_error_prev[NUM_MOTOR];       // 前回速度誤差
extern float vel_prop_prev[NUM_MOTOR];         // 速度比例項前回値

extern float vel_output[NUM_MOTOR];           // 速度PID出力
extern float vel_out[NUM_MOTOR];           // 最終速度出力

// -------- PIDゲイン -------- //
extern float kp_vel;
extern float ki_vel;
extern float kd_vel; // 速度微分ゲイン

//速度PID計算関数
float pid_vel(float setpoint, float input, float &error_prev,float &prop_prev, float &output,
          float kp, float ki, float kd, float dt);

// ********* CAN関連ここまで ********* //

void m3508_ENC_SW_Read_Publish_Task(void *pvParameters);

void C620_Task(void *pvParameters);

void C620_FB_Task(void *pvParameters);
