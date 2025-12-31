/*
                              .oooo.    o8o                 ooooooooo.                                   .o    .o88o.
                            .dP""Y88b   `"'                 `888   `Y88.                               .d88    888 `"
oooo d8b  .ooooo.   .oooo.o       ]8P' oooo   .ooooo.        888   .d88'  .ooooo.  oooo    ooo       .d'888   o888oo
`888""8P d88' `88b d88(  "8     .d8P'  `888  d88' `88b       888ooo88P'  d88' `88b  `88.  .8'      .d'  888    888
 888     888   888 `"Y88b.    .dP'      888  888   888       888`88b.    888ooo888   `88..8'       88ooo888oo  888
 888     888   888 o.  )88b .oP     .o  888  888   888       888  `88b.  888    .o    `888'    .o.      888    888
d888b    `Y8bod8P' 8""888P' 8888888888 o888o `Y8bod8P'      o888o  o888o `Y8bod8P'     `8'     Y8P     o888o  o888o
*/

/*====================================================================
ros2io Rev.4f
Target board: NUCLEO-F446RE

Description:
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
    Serial.begin(115200);

    pinMode(F446RE_BUILTIN_LED, OUTPUT);

    // 以降FreeRTOSタスク関連

    // xTaskCreate(
    //     LED_Blink100_Task,   // タスク関数
    //     "LED_Blink100_Task", // タスク名
    //     256,                 // スタックサイズ（words）
    //     NULL,
    //     1, // 優先度
    //     NULL);

    xTaskCreate(
        txTask,   // タスク関数
        "txTask", // タスク名
        256,      // スタックサイズ（words）
        NULL,
        10, // 優先度
        NULL);

    xTaskCreate(
        rxTask,   // タスク関数
        "rxTask", // タスク名
        256,      // スタックサイズ（words）
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
}
