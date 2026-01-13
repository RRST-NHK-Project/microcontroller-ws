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
#include <serial_task.hpp>

// ================= SETUP =================

void setup() {

    // プログラムが書き込めなくなるバグの応急処置
    delay(2000); // 安定待ち

    // ボーレートは実機テストしながら調整する予定
    Serial.begin(115200);

    pinMode(BG431B_BUILTIN_LED, OUTPUT);

    // 以降FreeRTOSタスク関連

    xTaskCreate(
        serialTask,   // タスク関数
        "serialTask", // タスク名
        256,          // スタックサイズ（words）
        NULL,
        10, // 優先度
        NULL);

    vTaskStartScheduler();
}

// ================= LOOP =================

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    // メインループはなにもしない、処理はすべてFreeRTOSタスクで行う
}
