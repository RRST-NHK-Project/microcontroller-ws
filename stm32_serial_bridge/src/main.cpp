/*====================================================================
Project: stm32_serial_bridge
Target board: NUCLEO-F446RE

コメント整備中

Description:
  ROS 2・マイコン間の通信を行うserial_bridgeパッケージのマイコン側プログラム。
  PCから送られてくるバイナリデータを受信、デコードしマイコンのGPIO出力に反映させる。

  This is a simple serial communication example between a microcontroller
  and a PC using a custom binary protocol. The microcontroller sends and
  receives frames containing int16 data.

  Frame Structure:
  [START_BYTE][DEVICE_ID][LENGTH][DATA...][CHECKSUM]
    - START_BYTE: 0xAA
    - DEVICE_ID: 0x02
    - LENGTH: Number of data bytes (Tx16NUM * 2)
    - DATA: int16 data (big-endian)
    - CHECKSUM: XOR of all bytes except START_BYTE

  The microcontroller sends Tx16NUM int16 values to the PC and listens for
  incoming frames from the PC. Received frames are validated using the
  checksum and stored in Rx_16Data array.

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <defs.hpp>
#include <led_task.hpp>
#include <output_task.hpp>
#include <serial_task.hpp>

// ================= SETUP =================

void setup() {

    // ボーレートは実機テストしながら調整する予定
    Serial.begin(115200);

    pinMode(F446RE_BUILTIN_LED, OUTPUT);
    Output_init();

    // 以降FreeRTOSタスク関連

    // xTaskCreate(
    //     LED_Blink100_Task,   // タスク関数
    //     "LED_Blink100_Task", // タスク名
    //     256,                 // スタックサイズ（words）
    //     NULL,
    //     1, // 優先度
    //     NULL);

    // xTaskCreate(
    //     txTask,   // タスク関数
    //     "txTask", // タスク名
    //     256,      // スタックサイズ（words）
    //     NULL,
    //     10, // 優先度
    //     NULL);

    // xTaskCreate(
    //     rxTask,   // タスク関数
    //     "rxTask", // タスク名
    //     256,      // スタックサイズ（words）
    //     NULL,
    //     10, // 優先度
    //     NULL);

    xTaskCreate(
        serialTask,   // タスク関数
        "serialTask", // タスク名
        256,          // スタックサイズ（words）
        NULL,
        10, // 優先度
        NULL);

    xTaskCreate(
        Output_Task,   // タスク関数
        "Output_Task", // タスク名
        256,           // スタックサイズ（words）
        NULL,
        5, // 優先度
        NULL);

    vTaskStartScheduler();
}

// ================= LOOP =================

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    // メインループはなにもしない、処理はすべてFreeRTOSタスクで行う
}
