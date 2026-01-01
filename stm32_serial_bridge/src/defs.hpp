/*====================================================================
<defs.h>
・定数の定義ファイル

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once
#include <Arduino.h>

// ================= ピンの定義 =================

// ピンの定義 //
// 内蔵LED
#define F446RE_BUILTIN_LED LED_LD2 // 内蔵LED(NUCLEO-F446RE)

// 内蔵ボタン
#define F446RE_USER_BUTTON USER_BTN // ユーザーボタン(NUCLEO-F446RE)

// 外部シリアル変換モジュール（実装予定）
// #define DEBUG_SERIAL_TxD 14
// #define DEBUG_SERIAL_RxD 18

// ================================
// MD PWM
// ================================
#define MD1P PB_6 // MD PWM  TIM4 CH1
#define MD2P PB_7 // MD PWM  TIM4 CH2
#define MD3P PB_8 // MD PWM  TIM4 CH3
#define MD4P PB_9 // MD PWM  TIM4 CH4

#define MD5P PA_8  // MD PWM  TIM1 CH1
#define MD6P PA_9  // MD PWM  TIM1 CH2
#define MD7P PA_10 // MD PWM  TIM1 CH3
#define MD8P PA_11 // MD PWM  TIM1 CH4

// ================================
// MD DIR
// ================================
#define MD1D PA_6
#define MD2D PA_7
#define MD3D PB_2
#define MD4D PB_15
#define MD5D PC_0
#define MD6D PC_1
#define MD7D PC_2
#define MD8D PC_3

// ================================
// サーボ
// ================================
#define SERVO1 PA_0 // SERVO PWM  TIM2 CH1
#define SERVO2 PA_1 // SERVO PWM  TIM2 CH2
#define SERVO3 PA_2 // SERVO PWM  TIM2 CH3
#define SERVO4 PA_3 // SERVO PWM  TIM2 CH4

#define SERVO5 PB_4 // SERVO PWM  TIM3 CH1
#define SERVO6 PB_5 // SERVO PWM  TIM3 CH2

#define SERVO7 PA_6 // SERVO PWM  TIM5 CH1
#define SERVO8 PA_7 // SERVO PWM  TIM5 CH2

// ================================
// エンコーダ
// ================================
#define ENC1_A PC_4
#define ENC1_B PC_5

#define ENC2_A PC_6
#define ENC2_B PC_7

#define ENC3_A PC_8
#define ENC3_B PC_9

#define ENC4_A PC_10
#define ENC4_B PC_11

#define ENC5_A PC_12
#define ENC5_B PD_2

#define ENC6_A PB_12
#define ENC6_B PB_13

#define ENC7_A PB_14
#define ENC7_B PB_15

#define ENC8_A PA_4
#define ENC8_B PA_5

// ================================
// ソレノイドバルブ
// ================================
#define TR1 PC_13
#define TR2 PC_14
#define TR3 PC_15
#define TR4 PA_12
#define TR5 PA_15
#define TR6 PB_0
#define TR7 PB_1

// ================================
// スイッチ
// ================================
// #define SW1 PA_13
#define SW1 F446RE_USER_BUTTON
#define SW2 PA_14
#define SW3 PB_3
#define SW4 PB_10
#define SW5 PB_11
#define SW6 PC_0
#define SW7 PC_1
#define SW8 PC_2

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

#define SERVO5_MIN_US 500
#define SERVO5_MAX_US 2500
#define SERVO5_MIN_DEG 0
#define SERVO5_MAX_DEG 180

#define SERVO6_MIN_US 500
#define SERVO6_MAX_US 2500
#define SERVO6_MIN_DEG 0
#define SERVO6_MAX_DEG 180

#define SERVO7_MIN_US 500
#define SERVO7_MAX_US 2500
#define SERVO7_MIN_DEG 0
#define SERVO7_MAX_DEG 180

#define SERVO8_MIN_US 500
#define SERVO8_MAX_US 2500
#define SERVO8_MIN_DEG 0
#define SERVO8_MAX_DEG 180
