#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_PIN ((gpio_num_t)4)
#define CAN_RX_PIN ((gpio_num_t)2)

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CAN Receiver (TWAI)");

  // TWAI 設定
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500 kbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("TWAI driver install failed!");
    while (1);
  }

  if (twai_start() != ESP_OK) {
    Serial.println("TWAI start failed!");
    while (1);
  }

  Serial.println("TWAI initialized");
}

void loop() {
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {

    Serial.print("Received ");

    if (message.extd) {
      Serial.print("extended ");
    } else {
      Serial.print("standard ");
    }

    if (message.rtr) {
      Serial.print("RTR ");
    }

    Serial.print("packet with id 0x");
    Serial.print(message.identifier, HEX);

    if (message.rtr) {
      Serial.print(" and requested length ");
      Serial.println(message.data_length_code);
    } else {
      Serial.print(" and length ");
      Serial.println(message.data_length_code);

      // データ表示
      for (int i = 0; i < message.data_length_code; i++) {
        Serial.print((char)message.data[i]);
      }
      Serial.println();
    }

    Serial.println();
  }
}
