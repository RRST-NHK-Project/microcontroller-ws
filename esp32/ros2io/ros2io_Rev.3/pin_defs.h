#pragma once

// 各アクチュエータの総数を定義
#define MD 8
#define SERVO 8
#define SV 7

// 受信配列の要素数を定義
#define MAX_ARRAY_SIZE 25
/*
0:モード選択
1~8:MD
9~16:サーボ
17~23:ソレノイドバルブ
24:予備
*/

// パルスカウンタの上限・下限の定義
#define COUNTER_H_LIM 32767
#define COUNTER_L_LIM -32768
#define PCNT_FILTER_VALUE 1023 // 0~1023, 1 = 12.5ns

// MD出力の上限値
#define MD_PWM_MAX 255

// ピンの定義 //
// 状態表示LED
#define LED 0

// エンコーダ
#define ENC1_A 12
#define ENC1_B 13
#define ENC2_A 14
#define ENC2_B 15
#define ENC3_A 21
#define ENC3_B 22
#define ENC4_A 23
#define ENC4_B 25

// MD PWM
#define MD1P 21
#define MD2P 22
#define MD3P 23
#define MD4P 25
#define MD5P 26
#define MD6P 27
#define MD7P 2
#define MD8P 4

// MD DIR
#define MD1D 12
#define MD2D 13
#define MD3D 14
#define MD4D 15
#define MD5D 32
#define MD6D 33
#define MD7D 5
#define MD8D 19

// サーボ
#define SERVO1 16
#define SERVO2 17
#define SERVO3 18
#define SERVO4 19
#define SERVO5 21
#define SERVO6 22
#define SERVO7 23
#define SERVO8 25

// ソレノイドバルブ
#define SV1 2
#define SV2 4
#define SV3 5
#define SV4 12
#define SV5 13
#define SV6 14
#define SV7 15

// スイッチ
#define SW1 26
#define SW2 27
#define SW3 32
#define SW4 33
#define SW5 2
#define SW6 19
#define SW7 14
#define SW8 15

// ロボマス
#define CAN_RX 4
#define CAN_TX 5
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