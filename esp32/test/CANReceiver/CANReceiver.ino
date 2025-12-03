#include "driver/twai.h"
#include <Arduino.h>

#define CAN_TX_PIN ((gpio_num_t)4) // このESPは送信しない場合未接続でもOK
#define CAN_RX_PIN ((gpio_num_t)2) // バスに接続されているCAN RX

void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;

    Serial.println("ESP32 TWAI Receiver");

    // TWAI 設定
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();   // 500 kbps
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // すべて受信

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

    Serial.println("TWAI initialized for receiving");
}

void loop() {
    twai_message_t rx_msg;

    // 10ms タイムアウトで受信
    if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
        Serial.print("Received ");
        Serial.print(rx_msg.extd ? "Extended" : "Standard");
        Serial.print(" ID: 0x");
        Serial.print(rx_msg.identifier, HEX);
        Serial.print(" DLC: ");
        Serial.print(rx_msg.data_length_code);
        Serial.print(" Data: ");
        for (int i = 0; i < rx_msg.data_length_code; i++) {
            Serial.print(rx_msg.data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}
