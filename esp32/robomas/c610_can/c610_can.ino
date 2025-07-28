#include <CAN.h>
//#include "c610.hpp"

float vel = 0;
float pos = 0;

int counter = 0;
bool data_size_error_flag_ = false;


constexpr int motor_id = 2;

void send_cur(float cur) {
  constexpr float MAX_CUR = 10;
  constexpr int MAX_CUR_VAL = 10000;

  float val = cur * (MAX_CUR_VAL / MAX_CUR);
  if (val < -MAX_CUR_VAL) val = -MAX_CUR_VAL;
  else if (val > MAX_CUR_VAL) val = MAX_CUR_VAL;
  int16_t transmit_val = val;

  uint8_t send_data[8] = {};

  send_data[(motor_id - 1) * 2] = (transmit_val >> 8) & 0xFF;
  send_data[(motor_id - 1) * 2 + 1] = transmit_val & 0xFF;
  CAN.beginPacket(0x200);
  CAN.write(send_data, 8);
  CAN.endPacket();
}

void can_callback(int packetSize) {
  static std::array<uint8_t, 8> received_data;
  if (CAN.packetId() == 0x200 + motor_id) {
    counter++;
    int data_size_counter = 0;
    while (CAN.available()) {
      received_data[data_size_counter % 8] = CAN.read();
      data_size_counter++;
    }
    data_size_error_flag_ = (data_size_counter != 8);  // 8byte以上のデータが溜まってないか一応チェック

    pos = static_cast<int16_t>((received_data[0] << 8) + received_data[1]) / 8192.; //0~1に
vel = static_cast<int16_t>((received_data[2] << 8) + received_data[3]) / 60.;  //rpmをrpsに
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
    ;

  CAN.setPins(25, 26);
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }

  CAN.onReceive(can_callback);
}

void loop() {
  int i = 0;
  for (float cur = -0.5; cur < 0.5; cur += 0.01) {
    send_cur(cur);
    delay(10);
    i++;
    if (!(i % 10)) { // シリアルプロッタを伸ばしすぎないために1/10に制限
      Serial.print("pos:");
      Serial.println(pos);
      Serial.print("vel:");
      Serial.println(vel);
    }
  }
  for (float cur = 0.5; cur > -0.5; cur -= 0.01) {
    send_cur(cur);
    delay(10);
    i++;
    if (!(i % 10)) { // シリアルプロッタを伸ばしすぎないために1/10に制限
      Serial.print("pos:");
      Serial.println(pos);
      Serial.print("vel:");
      Serial.println(vel);
    }
  }

  /*
  //デバッグ
  Serial.print("counter:");
  Serial.print(counter);
  Serial.print(" error:");
  Serial.println(data_size_error_flag_);
  */
}