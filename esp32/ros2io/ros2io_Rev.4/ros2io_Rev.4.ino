/*====================================================================
ros2io Rev.4 beta
Target board: ESP32 Dev Module (ESP32-DEVKITC-32E)

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

/*
For NHK-Robocon-2026
ESP32用microROSプログラム。ROSメッセージからマイコンのIOを操作する。
ライブラリはros2io/librariesに格納。修正済みのライブラリを使用しているため同梱のZipを使用すること。
ボードマネージャーはesp32 by Espressif Systemsを選択
config.hでIDとMODEを設定する。
*/

/*
TODO:ROSからのモード選択機能の実装（保留中）
TODO:定数定義の統一
*/

#include <Arduino.h>
#include <CAN.h>
#include <esp32-hal-ledc.h>

// microROS関連
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32_multi_array.h>

//  パルスカウンタ関連
#include "driver/pcnt.h"

// 自作ヘッダーファイル
#include "can_defs.h"    //CAN関連を管理
#include "config.h"      //モードやIDを管理
#include "defs.h"        //定数を管理
#include "input_task.h"  //入力系のタスクを管理
#include "output_task.h" //出力系のタスクを管理
#include "ros_defs.h"    //microROS関連を管理

void setup() {

    // 状態表示LEDの初期化
    pinMode(LED, OUTPUT);

    // MODEに応じた初期化
    switch (MODE) {
    case 0:
        while (1) {
            // テストモード実装までの応急所置
            ;
        }
        Serial.begin(115200);
        mode0_init();
        break;
    case 1:
        ros_init();
        mode1_init();
        break;
    case 2:
        ros_init();
        mode2_init();
        break;
    case 3:
        ros_init();
        mode3_init();
        break;
    case 4:
        ros_init();
        mode4_init();
        break;
    default:;
        ;
    }
}

void loop() {
    if (MODE != 0) {
        RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
        vTaskDelay(1);
    }
}

// 各モードの初期化関数

void mode1_init() {
    // モード1用の初期化

    // MDの方向ピンを出力に設定
    pinMode(MD1D, OUTPUT);
    pinMode(MD2D, OUTPUT);
    pinMode(MD3D, OUTPUT);
    pinMode(MD4D, OUTPUT);

    // PWMの初期化
    ledcAttach(MD1P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD2P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD3P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD4P, MD_PWM_FREQ, MD_PWM_RESOLUTION);

    // サーボのPWMの初期化
    ledcAttach(SERVO1, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO2, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO3, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO4, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);

    // トランジスタのピンを出力に設定
    pinMode(TR1, OUTPUT);
    pinMode(TR2, OUTPUT);
    pinMode(TR3, OUTPUT);
    pinMode(TR4, OUTPUT);
    pinMode(TR5, OUTPUT);
    pinMode(TR6, OUTPUT);
    pinMode(TR7, OUTPUT);

    // 受信＆ピン操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        Output_Task,
        "Output_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        PRO_CPU_NUM);
}

void mode2_init() {
    // モード2用の初期化
    while (1) {
        ;
    }
}

void mode3_init() {
    // モード3用の初期化

    // エンコーダの初期化
    enc_init_all();

    // スイッチのピンを入力に設定し内蔵プルアップ抵抗を有効化
    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW3, INPUT_PULLUP);
    pinMode(SW4, INPUT_PULLUP);

    // Rev.3からそのまま、そのうち変える
    msg.data.data = (int32_t *)malloc(sizeof(int32_t) * 8);
    msg.data.size = 8;
    msg.data.capacity = 8;

    xTaskCreateUniversal(
        ENC_PRI_Read_Publish_Task,
        "ENC_PRI_Read_Publish_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    xTaskCreateUniversal(
        LED_PWM_Task,
        "LED_PWM_Task",
        2048,
        NULL,
        1, // 優先度、最大25？
        &led_pwm_handle,
        APP_CPU_NUM);
}

void mode4_init() {
    // エンコーダの初期化
    enc_init_half();

    // スイッチのピンを入力に設定し内蔵プルアップ抵抗を有効化
    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW3, INPUT_PULLUP);
    pinMode(SW4, INPUT_PULLUP);
    pinMode(SW5, INPUT_PULLUP);
    pinMode(SW6, INPUT_PULLUP);
    pinMode(SW7, INPUT_PULLUP);
    pinMode(SW8, INPUT_PULLUP);

    // Rev.3からそのまま、そのうち変える
    msg.data.data = (int32_t *)malloc(sizeof(int32_t) * 11);
    msg.data.size = 11;
    msg.data.capacity = 11;

    xTaskCreateUniversal(
        SW_PRI_Read_Publish_Task,
        "SW_PRI_Read_Publish_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    xTaskCreateUniversal(
        LED_PWM_Task,
        "LED_PWM_Task",
        2048,
        NULL,
        1, // 優先度、最大25？
        &led_pwm_handle,
        APP_CPU_NUM);
}

// テストモード　※実機で「絶対」に実行するな！
// シリアルモニターからEnterが押されるまで待機する
void mode0_init() {
    while (1) {
        // テストモード実装予定
        ;
    }
}