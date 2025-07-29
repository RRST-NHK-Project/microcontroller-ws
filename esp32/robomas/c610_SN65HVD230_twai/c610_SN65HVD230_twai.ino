#include <Arduino.h>
#include "driver/twai.h"
#include "driver/gpio.h"

// GPIO番号はgpio_num_t型で指定
const gpio_num_t TX_GPIO_NUM = GPIO_NUM_25;  // ESP32のCAN TXピン
const gpio_num_t RX_GPIO_NUM = GPIO_NUM_26;  // ESP32のCAN RXピン

void setup() {
  Serial.begin(115200);
  delay(1000);

  // TWAIドライバの設定
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // ドライバインストール
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("TWAI driver install failed");
    while (1);
  }

  // ドライバ開始
  if (twai_start() != ESP_OK) {
    Serial.println("TWAI start failed");
    while (1);
  }

  Serial.println("TWAI driver started");
}

void loop() {
  // 4つのM2006モーター用トルク値を送信
  send_motors(3000, 0, 0, 0);
  delay(10);
}

void send_motors(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
  twai_message_t message;
  message.identifier = 0x200;   // CAN ID
  message.extd = 0;             // 標準フレーム
  message.rtr = 0;              // データフレーム
  message.data_length_code = 8; // 8バイト

  // 範囲制限
  m1 = constrain(m1, -16384, 16384);
  m2 = constrain(m2, -16384, 16384);
  m3 = constrain(m3, -16384, 16384);
  m4 = constrain(m4, -16384, 16384);

  // データ格納（高バイト→低バイト）
  message.data[0] = (m1 >> 8) & 0xFF;
  message.data[1] = m1 & 0xFF;
  message.data[2] = (m2 >> 8) & 0xFF;
  message.data[3] = m2 & 0xFF;
  message.data[4] = (m3 >> 8) & 0xFF;
  message.data[5] = m3 & 0xFF;
  message.data[6] = (m4 >> 8) & 0xFF;
  message.data[7] = m4 & 0xFF;

  // 送信（タイムアウト100ms）
  if (twai_transmit(&message, pdMS_TO_TICKS(100)) != ESP_OK) {
    Serial.println("Transmit failed");
  }
}
