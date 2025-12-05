/*====================================================================
<bldc_can.cpp>
・bldc_can.h の実装ファイル

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "defs.h"
#include "driver/twai.h"
#include <Arduino.h>

static uint32_t gSendDate = 0;
static uint32_t gSentCount = 0;
static uint8_t gFrameLength = 0;

void BLDC_CAN_init() {

    DEBUG_PRINTLN("ESP32 TWAI Sender");

    // TWAI 設定
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)BLDC_CAN_TX, (gpio_num_t)BLDC_CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();   // 500 kbps
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // フィルターなし

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        DEBUG_PRINTLN("TWAI driver install failed!");
        while (1)
            ;
    }

    if (twai_start() != ESP_OK) {
        DEBUG_PRINTLN("TWAI start failed!");
        while (1)
            ;
    }

    DEBUG_PRINTLN("TWAI initialized");
}

void BLDC_CAN_Send_Task(void *pvParameters) {
    while (1) {
        if (gSendDate < millis()) {
            twai_message_t msg;
            msg.data_length_code = gFrameLength; // DLC 0～8

            msg.data[0] = 100;
            msg.data[1] = 100;
            msg.data[2] = 100;
            msg.data[3] = 100;
            msg.data[4] = 100;
            msg.data[5] = 100;
            msg.data[6] = 100;
            msg.data[7] = 100;
            

            msg.extd = 0;           // 標準ID,1に変更すると拡張IDが使用可能
            msg.identifier = 0x431; // ID
            gFrameLength = 8;
            msg.rtr = 0;            // データフレーム

            if (twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
                // Serial.print("Sent ");
                // Serial.print(msg.extd ? "Extended" : "Standard");
                // Serial.print(" ID 0x");
                // Serial.print(msg.identifier, HEX);
                // Serial.print(" DLC: ");
                // Serial.println(msg.data_length_code);

                 gSendDate += 10; // 一定周期ごと送信
            //     gFrameLength++;
            //     if (gFrameLength > 8)
            //         gFrameLength = 0; // DLCは最大8
            } else {
            //     // Serial.println("Send failed");
            }
        }
    }
}

void BLDC_CAN_Receive_Task(void *pvParameters) {
    while (1) {
        ;
    }
}
