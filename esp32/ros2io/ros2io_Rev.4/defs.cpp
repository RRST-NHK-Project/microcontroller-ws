/*====================================================================
<defs.cpp>
・defs.h で定義した変数の初期化を行う

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "defs.h"
#include <Arduino.h>
// microROS関連
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32_multi_array.h>

// 受信データ格納用のバッファ
int32_t buffer[MAX_ARRAY_SIZE];

// 受信データ格納用
int32_t received_data[MAX_ARRAY_SIZE]; // 受信データ //2025/09/29: volatileを削除
size_t received_size = 0;              // 受信データのサイズ //2025/09/29: volatileを削除

// エンコーダのカウント格納用
int16_t count[4] = {0};

// スイッチの状態格納用
bool sw_state[8] = {false};

// ===== タスクハンドルのグローバル変数 =====
TaskHandle_t led_blink100_handle = NULL;
TaskHandle_t led_pwm_handle = NULL;