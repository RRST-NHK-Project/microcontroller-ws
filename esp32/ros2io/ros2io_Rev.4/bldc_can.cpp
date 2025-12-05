/*====================================================================
<bldc_can.cpp>
・bldc_can.h の実装ファイル

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "defs.h"
#include "driver/twai.h"
#include <Arduino.h>

extern int16_t received_data[MAX_ARRAY_SIZE]; // 受信データ

int16_t received_data_dammy[MAX_ARRAY_SIZE] = {0}; // ダミー受信データ

static int16_t gSendDate = 0;
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

            // 上位バイト・下位バイトに分割してペイロードに詰める
            msg.data[0] = (uint8_t)(received_data_dammy[1] >> 8);   // 上位バイト
            msg.data[1] = (uint8_t)(received_data_dammy[1] & 0xFF); // 下位バイト
            msg.data[2] = (uint8_t)(received_data_dammy[2] >> 8);   // 上位バイト
            msg.data[3] = (uint8_t)(received_data_dammy[2] & 0xFF); // 下位バイト
            msg.data[4] = (uint8_t)(received_data_dammy[3] >> 8);   // 上位バイト
            msg.data[5] = (uint8_t)(received_data_dammy[3] & 0xFF); // 下位バイト
            msg.data[6] = (uint8_t)(received_data_dammy[4] >> 8);   // 上位バイト
            msg.data[7] = (uint8_t)(received_data_dammy[4] & 0xFF); // 下位バイト

            msg.extd = 0;           // 標準ID,1に変更すると拡張IDが使用可能
            msg.identifier = 0x431; // ID
            gFrameLength = 8;
            msg.rtr = 0; // データフレーム

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
        vTaskDelay(1); // WDTのリセット(必須)
    }
}

// void BLDC_CAN_8bit_Receive_Task(void *pvParameters) {
//     Serial.begin(115200);
//     while (1) {
//         twai_message_t rx_msg;

//         // 20ms タイムアウトで受信
//         if (twai_receive(&rx_msg, pdMS_TO_TICKS(20)) == ESP_OK) {
//             Serial.printf("ID:0x%X Data:", rx_msg.identifier);
//             for (int i = 0; i < 8; i++) {
//                 Serial.printf(" %d", rx_msg.data[i]);
//             }
//             Serial.println();
//         }
//         vTaskDelay(1); // WDTのリセット(必須)
//     }
// }

// void BLDC_CAN_16bit_Receive_Task(void *pvParameters) {
//     Serial.begin(115200);
//     while (1) {
//         twai_message_t rx_msg;
//         int16_t msg_data[4];

//         // 20ms タイムアウトで受信(要調整？)
//         if (twai_receive(&rx_msg, pdMS_TO_TICKS(20)) == ESP_OK) {
//             msg_data[0] = (int16_t)(rx_msg.data[0] << 8 | rx_msg.data[1]);
//             msg_data[1] = (int16_t)(rx_msg.data[2] << 8 | rx_msg.data[3]);
//             msg_data[2] = (int16_t)(rx_msg.data[4] << 8 | rx_msg.data[5]);
//             msg_data[3] = (int16_t)(rx_msg.data[6] << 8 | rx_msg.data[7]);
//             Serial.printf("ID:0x%X Data:", rx_msg.identifier);
//             for (int i = 0; i < 4; i++) {
//                 Serial.printf(" %d", msg_data[i]);
//             }
//             Serial.println();
//         }
//         vTaskDelay(1); // WDTのリセット(必須)
//     }
// }
