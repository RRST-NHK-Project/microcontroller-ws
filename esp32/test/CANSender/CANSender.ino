#include "driver/twai.h"
#include <Arduino.h>

#define CAN_TX_PIN ((gpio_num_t)4)
#define CAN_RX_PIN ((gpio_num_t)2)

static uint32_t gSendDate = 0;
static uint32_t gSentCount = 0;
static uint8_t gFrameLength = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;

    Serial.println("ESP32 TWAI Sender");

    // TWAI 設定
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();   // 500 kbps
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // フィルターなし

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("TWAI driver install failed!");
        while (1)
            ;
    }

    if (twai_start() != ESP_OK) {
        Serial.println("TWAI start failed!");
        while (1)
            ;
    }

    Serial.println("TWAI initialized");
}

void loop() {
    if (gSendDate < millis()) {
        twai_message_t msg;
        msg.data_length_code = gFrameLength; // DLC 0～8
        for (uint8_t i = 0; i < gFrameLength; i++) {
            msg.data[i] = i;
        }

        if ((gSentCount % 2) == 0) {
            // 拡張ID
            msg.extd = 1;
            msg.identifier = 0x320431;
        } else {
            // 標準ID
            msg.extd = 0;
            msg.identifier = 0x431;
        }

        msg.rtr = 0; // データフレーム

        if (twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
            Serial.print("Sent ");
            Serial.print(msg.extd ? "Extended" : "Standard");
            Serial.print(" ID 0x");
            Serial.print(msg.identifier, HEX);
            Serial.print(" DLC: ");
            Serial.println(msg.data_length_code);

            gSendDate += 10; // 1秒ごと送信
            gSentCount++;
            gFrameLength++;
            if (gFrameLength > 8)
                gFrameLength = 0; // DLCは最大8
        } else {
            Serial.println("Send failed");
        }
    }
}
