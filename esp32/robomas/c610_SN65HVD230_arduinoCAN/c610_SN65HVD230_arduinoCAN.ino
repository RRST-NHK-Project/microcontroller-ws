#include <CAN.h>

const int motor_id = 0x201;  // C610は 0x201〜0x204（モーターID）

void setup() {
  Serial.begin(115200);

  // CAN初期化（baud: 1Mbps）
  if (!CAN.begin(1000000)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  Serial.println("CAN initialized");
}

void loop() {
  send_motor_command(motor_id, 1000);  // 例: モーターIDにトルク指令
  delay(10);
}

void send_motor_command(uint16_t id, int16_t current) {
  CAN.beginPacket(id);
  CAN.write(highByte(current));  // モーター1のcurrent（2バイト）
  CAN.write(lowByte(current));
  // 他のモーターが不要な場合は残り6バイトは0
  for (int i = 0; i < 6; i++) {
    CAN.write(0);
  }
  CAN.endPacket();
}
