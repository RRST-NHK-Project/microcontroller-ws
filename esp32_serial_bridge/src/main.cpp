/*====================================================================
Project: stm32_serial_bridge
Target board: ESP32 Dev Module

Description:
  ROS 2・マイコン間の通信を行うserial_bridgeパッケージのマイコン側プログラム。
  PCから送られてくるバイナリデータを受信、デコードしマイコンのGPIO出力に反映させる。
  config.hppで各種設定をするのみで使用可能です。このファイル(main.cpp)を直接編集しないこと。

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "config.hpp"
#include "defs.hpp"
#include "led_task.hpp"
#include "pin_ctrl_task.hpp"
#include "robomas.hpp"
#include "serial_task.hpp"
#include <Arduino.h>
// ================= SETUP =================

void setup() {

    // プログラムが書き込めなくなるバグの応急処置
    delay(2000); // 安定待ち

    // ボーレートは実機テストしながら調整する予定
    Serial.begin(115200);

    // // これどこかに移してくれませんか？ //
    // twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    // twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    // twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    // if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    //     // Serial.println("TWAI install failed");
    //     while (1)
    //         ;
    // }
    // if (twai_start() != ESP_OK) {
    //     // Serial.println("TWAI start failed");
    //     while (1)
    //         ;
    // }
    // // ここまで //

    pinMode(LED, OUTPUT);

    xTaskCreate(
        serialTask,   // タスク関数
        "serialTask", // タスク名
        2048,         // スタックサイズ（words）
        NULL,
        10, // 優先度
        NULL);

// モードに応じた初期化
#if defined(MODE_OUTPUT)
    // 出力モード初期化
    xTaskCreate(
        Output_Task,   // タスク関数
        "Output_Task", // タスク名
        2048,          // スタックサイズ（words）
        NULL,
        9, // 優先度
        NULL);
#elif defined(MODE_INPUT)
    // 入力モード初期化
    xTaskCreate(
        Input_Task,   // タスク関数
        "Input_Task", // タスク名
        1024,         // スタックサイズ（words）
        NULL,
        4, // 優先度
        NULL);
#elif defined(MODE_ROBOMAS)
    // ロボマスモード初期化
    xTaskCreate(
        M3508_Task,   // タスク関数
        "M3508_Task", // タスク名
        2048,         // スタックサイズ（words）
        NULL,
        9, // 優先度
        NULL);

    xTaskCreate(
        M3508_RX,   // タスク関数
        "M3508_RX", // タスク名
        2048,       // スタックサイズ（words）
        NULL,
        9, // 優先度
        NULL);
#elif defined(MODE_DEBUG)
    // デバッグモード初期化
#else
#error "No mode defined. Please define one mode in config.hpp."
#endif

#if (defined(MODE_OUTPUT) + defined(MODE_INPUT) + \
     defined(MODE_ROBOMAS) + defined(MODE_DEBUG)) != 1
#error "Invalid mode configuration. Please define exactly *one mode* in config.hpp."
#endif
}

// ================= LOOP =================

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    // メインループはなにもしない、処理はすべてFreeRTOSタスクで行う
}
