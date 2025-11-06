/*====================================================================
<defs.h>
・定数の定義ファイル

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include "defs.h"
// microROS関連
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32_multi_array.h>

// 各アクチュエータの総数を定義
#define MD 4
#define SERVO 4
#define TR 7
#define ENC 4
#define SW 8

// 受信配列の要素数を定義
#define MAX_ARRAY_SIZE 18 // Rev.4から変更(25 -> 18)
/*
0:予備
1~4:MD
5~8:サーボ
9~10:予備
11~17:ソレノイドバルブ
*/

// パルスカウンタの上限・下限の定義
#define COUNTER_H_LIM 32767
#define COUNTER_L_LIM -32768
#define PCNT_FILTER_VALUE 1023 // 0~1023, 1 = 12.5ns

// MD出力の上限値
#define MD_PWM_MAX 255 // 8bit

// ピンの定義 //
// 状態表示LED
#define LED 0

// MD PWM
#define MD1P 5
#define MD2P 12
#define MD3P 13
#define MD4P 14

// MD DIR
#define MD1D 15
#define MD2D 16
#define MD3D 17
#define MD4D 18

// サーボ
#define SERVO1 19
#define SERVO2 21
#define SERVO3 22
#define SERVO4 23

// ソレノイドバルブ
#define TR1 25
#define TR2 26
#define TR3 27
#define TR4 32
#define TR5 33
#define TR6 22
#define TR7 23

// エンコーダ
#define ENC1_A 19
#define ENC1_B 21
#define ENC2_A 22
#define ENC2_B 23
#define ENC3_A 15
#define ENC3_B 16
#define ENC4_A 17
#define ENC4_B 18

// スイッチ
#define SW1 5
#define SW2 12
#define SW3 13
#define SW4 14
#define SW5 15
#define SW6 16
#define SW7 17
#define SW8 18

// ロボマス
#define CAN_RX 2
#define CAN_TX 4
// PWM関連の設定値を定義
// MD用
#define MD_PWM_FREQ 20000   // MDのPWM周波数
#define MD_PWM_RESOLUTION 8 // MDのPWM分解能（8ビット）

// サーボ用
#define SERVO_PWM_FREQ 50       // サーボPWM周波数
#define SERVO_PWM_RESOLUTION 16 // サーボPWM分解能（16ビット）

#define SERVO_PWM_PERIOD_US (1000000.0 / SERVO_PWM_FREQ) // 周波数から周期を計算
#define SERVO_PWM_MAX_DUTY ((1 << SERVO_PWM_RESOLUTION) - 1)
#define SERVO_PWM_SCALE (SERVO_PWM_MAX_DUTY / SERVO_PWM_PERIOD_US)

#define SERVO1_MIN_US 500
#define SERVO1_MAX_US 2500
#define SERVO1_MIN_DEG 0
#define SERVO1_MAX_DEG 180

#define SERVO2_MIN_US 500
#define SERVO2_MAX_US 2500
#define SERVO2_MIN_DEG 0
#define SERVO2_MAX_DEG 180

#define SERVO3_MIN_US 500
#define SERVO3_MAX_US 2500
#define SERVO3_MIN_DEG 0
#define SERVO3_MAX_DEG 180

#define SERVO4_MIN_US 500
#define SERVO4_MAX_US 2500
#define SERVO4_MIN_DEG 0
#define SERVO4_MAX_DEG 180

// 受信データ格納用のバッファ
extern int32_t buffer[MAX_ARRAY_SIZE];

// 受信データ格納用
extern int32_t received_data[MAX_ARRAY_SIZE]; // 受信データ //2025/09/29: volatileを削除
extern size_t received_size;                  // 受信データのサイズ //2025/09/29: volatileを削除

// エンコーダのカウント格納用
extern int16_t count[4];

// スイッチの状態格納用
extern bool sw_state[8];

// ===== タスクハンドルのグローバル変数 =====
extern TaskHandle_t led_blink100_handle;
extern TaskHandle_t led_pwm_handle;
