#include <Arduino.h>
#include "driver/twai.h"

// ESP32 CAN GPIO設定
const gpio_num_t CAN_TX = GPIO_NUM_26;
const gpio_num_t CAN_RX = GPIO_NUM_25;

void setup() {
  Serial.begin(115200);

  // TWAI設定: ノーマルモードで初期化
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // ドライバインストール
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("CANドライバインストール失敗");
    while (1);
  }

  if (twai_start() != ESP_OK) {
    Serial.println("CANスタート失敗");
    while (1);
  }

  Serial.println("CAN初期化成功 - C610制御開始");
}

void loop() {
  // 電流指令（-16384〜+16384）目安: 3000〜8000 で回転が始まる
  int16_t current1 = 2000;  // モーター1
  int16_t current2 = 0;     // モーター2
  int16_t current3 = 0;     // モーター3
  int16_t current4 = 0;     // モーター4

  twai_message_t msg;
  msg.identifier = 0x200;  // 電流制御用ID（固定）
  msg.flags = TWAI_MSG_FLAG_NONE;
  msg.data_length_code = 8;

  // 各モーターの電流を上位→下位バイトで格納
  msg.data[0] = (current1 >> 8) & 0xFF;
  msg.data[1] = current1 & 0xFF;
  msg.data[2] = (current2 >> 8) & 0xFF;
  msg.data[3] = current2 & 0xFF;
  msg.data[4] = (current3 >> 8) & 0xFF;
  msg.data[5] = current3 & 0xFF;
  msg.data[6] = (current4 >> 8) & 0xFF;
  msg.data[7] = current4 & 0xFF;

  // 送信
  if (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.println("モータに電流指令送信");
  } else {
    Serial.println("CAN送信失敗");
  }

  // ステータス表示
  twai_status_info_t status;
  twai_get_status_info(&status);

  Serial.print("Bus状態: ");
  switch (status.state) {
    case TWAI_STATE_BUS_OFF: Serial.println("BUS OFF"); break;
    case TWAI_STATE_STOPPED: Serial.println("STOPPED"); break;
    case TWAI_STATE_RUNNING: Serial.println("RUNNING"); break;
    default: Serial.println("UNKNOWN"); break;
  }

  Serial.print("TXエラー数: ");
  Serial.println(status.tx_error_counter);
  Serial.print("RXエラー数: ");
  Serial.println(status.rx_error_counter);
  Serial.print("Busエラー数: ");
  Serial.println(status.bus_error_count);

  delay(10);  // 100Hz送信（10ms間隔）
}
