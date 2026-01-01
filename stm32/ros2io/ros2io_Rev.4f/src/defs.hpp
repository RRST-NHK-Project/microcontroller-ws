/*====================================================================
<defs.h>
・定数の定義ファイル

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once
#include <Arduino.h>

// ================= ピンの定義 =================

// ピンの定義 //
// 状態表示LED
#define F446RE_BUILTIN_LED LED_LD2 // 内蔵LED(NUCLEO-F446RE)

// 外部シリアル変換モジュール（実装予定）
// #define DEBUG_SERIAL_TxD 14
// #define DEBUG_SERIAL_RxD 18

// MD PWM
#define MD1P PB_6  // TIM4_CH1
#define MD2P PA_8  // TIM1_CH1
#define MD3P PB_1  // TIM3_CH4
#define MD4P PB_14 // TIM12_CH1

// MD DIR
#define MD1D PA_6
#define MD2D PA_9
#define MD3D PB_2
#define MD4D PB_15

// サーボ
#define SERVO1 PA_10 // TIM2_CH3
#define SERVO2 PA_11 // TIM2_CH4
#define SERVO3 PB_3  // TIM2_CH2
#define SERVO4 PB_10 // TIM2_CH3

// ソレノイドバルブ
#define TR1 PC_0
#define TR2 PC_1
#define TR3 PC_2
#define TR4 PC_3
#define TR5 PC_4
#define TR6 PC_5
#define TR7 PC_6

// エンコーダ
#define ENC1_A PA_0
#define ENC1_B PA_1
#define ENC2_A PA_2
#define ENC2_B PA_3
#define ENC3_A PA_4
#define ENC3_B PA_5
#define ENC4_A PA_7
#define ENC4_B PB_0

// スイッチ
#define SW1 PC_8
#define SW2 PC_9
#define SW3 PC_10
#define SW4 PC_11
#define SW5 PC_12
#define SW6 PB_4
#define SW7 PB_5
#define SW8 PB_7

// CAN通信（ロボマス）
// #define CAN_RX
// #define CAN_TX

// ================= 定数 =================
// MD出力の上限値
#define MD_PWM_MAX 255 // 8bit

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
