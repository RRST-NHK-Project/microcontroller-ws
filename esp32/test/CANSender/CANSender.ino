#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_PIN ((gpio_num_t)4)
#define CAN_RX_PIN ((gpio_num_t)2)

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CAN Sender (TWAI)");

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
  // 標準IDパケット送信
  Serial.print("Sending standard packet ... ");
  twai_message_t msg_std;
  msg_std.identifier = 0x12;       // 11bit標準ID
  msg_std.extd = 0;                 // 標準ID
  msg_std.rtr = 0;                  // データフレーム
  msg_std.data_length_code = 5;     // データ長
  msg_std.data[0] = 'h';
  msg_std.data[1] = 'e';
  msg_std.data[2] = 'l';
  msg_std.data[3] = 'l';
  msg_std.data[4] = 'o';

  if (twai_transmit(&msg_std, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("done");
  } else {
    Serial.println("failed");
  }

  delay(1000);

  // 拡張IDパケット送信
  Serial.print("Sending extended packet ... ");
  twai_message_t msg_ext;
  msg_ext.identifier = 0xABCDEF;   // 29bit拡張ID
  msg_ext.extd = 1;                 // 拡張IDフラグ
  msg_ext.rtr = 0;
  msg_ext.data_length_code = 5;
  msg_ext.data[0] = 'w';
  msg_ext.data[1] = 'o';
  msg_ext.data[2] = 'r';
  msg_ext.data[3] = 'l';
  msg_ext.data[4] = 'd';

  if (twai_transmit(&msg_ext, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("done");
  } else {
    Serial.println("failed");
  }

  delay(1000);
}
