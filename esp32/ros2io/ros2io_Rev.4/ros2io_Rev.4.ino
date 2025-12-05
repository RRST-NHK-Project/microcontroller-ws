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
config.hでIDとMODEを設定
defs.hでピンアサインや定数を管理
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
#include "bldc_can.h"    //BLDC用CAN関連を管理
#include "c620_defs.h"   //C620関連を管理
#include "can_defs.h"    //CAN関連を管理
#include "config.h"      //モードやIDを管理
#include "defs.h"        //定数を管理
#include "input_task.h"  //入力系のタスクを管理
#include "mode_init.h"   //各モードの初期化関数を管理
#include "output_task.h" //出力系のタスクを管理
#include "ros_defs.h"    //microROS関連を管理
#include "twai.h"        //twai関連を管理(ヘッダー名変えて)

// デバッグ出力用のマクロ
// #if DEBUG_SERIAL
// #define DEBUG_BEGIN(baud) Serial.begin(baud, SERIAL_8N1, DEBUG_SERIAL_TxD, DEBUG_SERIAL_RxD);
// #define DEBUG_PRINT(x) Serial.print(x);
// #define DEBUG_PRINTLN(x) Serial.println(x);
// #else
// #define DEBUG_BEGIN(baud)
// #define DEBUG_PRINT(x)
// #define DEBUG_PRINTLN(x)
// #endif

void setup() {

    DEBUG_BEGIN(115200);
    DEBUG_PRINTLN("Debug Serial Started");

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
        // 1:出力（モタドラ、サーボ、ソレノイド）
        ros_init();
        mode1_init();
        break;
    case 2:
        // 2:出力（ロボマス、モタドラ、サーボ、ソレノイド）
        ros_init();
        mode2_init();
        break;
    case 3:
        // 3:入力（エンコーダー優先）
        ros_init();
        mode3_init();
        break;
    case 4:
        // 4:入力（マイクロスイッチ優先）
        ros_init();
        mode4_init();
        break;
    case 5:
        // 5:入力（C620 + M3508）
        ros_init();
        mode5_init();
        break;
    case 6:
        // 6:出力（C610 + M2006）
        ros_init();
        mode6_init();
        break;
    case 7:
        ros_init();
        mode7_init();
        break;
    case 8:
        // ros_init();
        BLDC_CAN_init();
        mode8_init();
        break;
    case 9:
        // ros_init();
        BLDC_CAN_init();
        mode9_init();
        break;
    case 101: // テスト用（自由に変えていい）
        ros_wifi_init();
        mode3_init();
        break;
    default:;
        ;
    }
}

void loop() {
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        vTaskDelay(1); // 1msスリープ
    }
}
