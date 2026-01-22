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
#include <defs.hpp>
#include <led_task.hpp>
#include <pin_ctrl_task.hpp>
#include <serial_task.hpp>
#include <robomas.hpp>

// ================= SETUP =================

void setup()
{

  // プログラムが書き込めなくなるバグの応急処置
  delay(2000); // 安定待ち

  // ボーレートは実機テストしながら調整する予定
  Serial.begin(115200);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
  {
    // Serial.println("TWAI install failed");
    while (1)
      ;
  }
  if (twai_start() != ESP_OK)
  {
    // Serial.println("TWAI start failed");
    while (1)
      ;
  }

  pinMode(LED, OUTPUT);

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
      2048,         // スタックサイズ（words）
      NULL,
      10, // 優先度
      NULL);

  xTaskCreate(
      Output_Task,   // タスク関数
      "Output_Task", // タスク名
      1024,          // スタックサイズ（words）
      NULL,
      4, // 優先度
      NULL);

  // xTaskCreate(
  //     Input_Task,   // タスク関数
  //     "Input_Task", // タスク名
  //     1024,         // スタックサイズ（words）
  //     NULL,
  //     4, // 優先度
  //     NULL);

  // xTaskCreate(
  //     Pin_Ctrl_Task,   // タスク関数
  //     "Pin_Ctrl_Task", // タスク名
  //     2048,            // スタックサイズ（words）
  //     NULL,
  //     8, // 優先度
  //     NULL);

  // xTaskCreate(
  //     M3508_Task,   // タスク関数
  //     "M3508_Task", // タスク名
  //     2048,         // スタックサイズ（words）
  //     NULL,
  //     9, // 優先度
  //     NULL);

  // xTaskCreate(
  //     M3508_RX,   // タスク関数
  //     "M3508_RX", // タスク名
  //     2048,       // スタックサイズ（words）
  //     NULL,
  //     9, // 優先度
  //     NULL);

  // vTaskStartScheduler();
}

// ================= LOOP =================

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
  // メインループはなにもしない、処理はすべてFreeRTOSタスクで行う
}
