#include <CAN.h>

//---------定義--------//
const int ENCODER_MAX = 8192;
const int HALF_ENCODER = ENCODER_MAX / 2;
constexpr float gear_ratio = 36.0f;

const int NUM_MOTORS = 2;  // 使用するモータの台数（最大4台）

// モータ状態
struct Motor {
  int16_t encoder = 0;
  int16_t rpm = 0;
  int16_t last_encoder = -1;
  int32_t rotation_count = 0;
  int32_t total_encoder = 0;
  float angle = 0.0f;
  float vel = 0.0f;
  bool offset_ok = false;
  int encoder_offset = 0;

  // PID用
  float pos_error_prev = 0.0f;
  float pos_integral = 0.0f;
  float target_angle = 0.0f;
  float output_current = 0.0f;
};

Motor motors[NUM_MOTORS];

// PIDゲイン
float kp_pos = 0.8;
float ki_pos = 0.01;
float kd_pos = 0.02;
float current_limit_A = 10.0f;

// 時間管理
unsigned long lastPidTime = 0;

//--------------関数-------------//
float pid(float setpoint, float input, float &error_prev, float &integral, float kp, float ki, float kd, float dt);
float constrain_double(float val, float min_val, float max_val);
void send_cur(float cur1, float cur2);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  CAN.setPins(4, 5); // rx, tx
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // 目標角度設定
  motors[0].target_angle = 720;   // モータ1 → 2回転
  motors[1].target_angle = -360;  // モータ2 → -1回転
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastPidTime) / 1000.0f;
  if (dt <= 0) dt = 0.000001f;
  lastPidTime = now;

  // --- CAN受信 ---
  int packetSize = CAN.parsePacket();
  while (packetSize) {
    int id = CAN.packetId();
    if (id >= 0x201 && id < 0x201 + NUM_MOTORS) {
      int idx = id - 0x201; // モータ番号(0から)
      uint8_t rx[8];
      for (int i=0;i<8;i++) rx[i] = CAN.read();

      motors[idx].encoder = (rx[0] << 8) | rx[1];
      motors[idx].rpm = (rx[2] << 8) | rx[3];

      // 初回オフセット設定
      if (!motors[idx].offset_ok) {
        motors[idx].encoder_offset = motors[idx].encoder;
        motors[idx].last_encoder = -1;
        motors[idx].rotation_count = 0;
        motors[idx].total_encoder = 0;
        motors[idx].pos_integral = 0;
        motors[idx].pos_error_prev = 0;
        motors[idx].offset_ok = true;
        Serial.printf("Motor %d Offset set!\n", idx+1);
      }

      int enc_relative = motors[idx].encoder - motors[idx].encoder_offset;
      if (enc_relative < 0) enc_relative += ENCODER_MAX;

      if (motors[idx].last_encoder != -1) {
        int diff = motors[idx].encoder - motors[idx].last_encoder;
        if (diff > HALF_ENCODER) motors[idx].rotation_count--;
        else if (diff < -HALF_ENCODER) motors[idx].rotation_count++;
      }
      motors[idx].last_encoder = motors[idx].encoder;
      motors[idx].total_encoder = motors[idx].rotation_count * ENCODER_MAX + motors[idx].encoder;
      motors[idx].angle = motors[idx].total_encoder * (360.0 / (8192.0 * gear_ratio));
      motors[idx].vel = (motors[idx].rpm / gear_ratio) * 360.0 / 60.0;
    }
    packetSize = CAN.parsePacket();
  }

  // --- PID計算 & 電流決定 ---
  for (int i=0; i<NUM_MOTORS; i++) {
    float pos_out = pid(motors[i].target_angle, motors[i].angle,
                        motors[i].pos_error_prev, motors[i].pos_integral,
                        kp_pos, ki_pos, kd_pos, dt);
    motors[i].output_current = constrain_double(pos_out, -current_limit_A, current_limit_A);
  }

  // --- 電流指令送信（2台分まとめて） ---
  send_cur(motors[0].output_current, motors[1].output_current);

  // --- デバッグ出力 ---
  Serial.printf("M1: %.2f/%.2f, M2: %.2f/%.2f\n",
                motors[0].angle, motors[0].target_angle,
                motors[1].angle, motors[1].target_angle);

  delay(1);
}

// --- 電流送信（2台分） ---
void send_cur(float cur1, float cur2) {
  constexpr float MAX_CUR = 10;
  constexpr int MAX_CUR_VAL = 10000;

  auto conv = [&](float c) {
    float val = c * (MAX_CUR_VAL / MAX_CUR);
    if (val < -MAX_CUR_VAL) val = -MAX_CUR_VAL;
    else if (val > MAX_CUR_VAL) val = MAX_CUR_VAL;
    return (int16_t)val;
  };

  int16_t t1 = conv(cur1);
  int16_t t2 = conv(cur2);

  uint8_t send_data[8] = {};
  send_data[0] = (t1 >> 8) & 0xFF;
  send_data[1] = t1 & 0xFF;
  send_data[2] = (t2 >> 8) & 0xFF;
  send_data[3] = t2 & 0xFF;

  CAN.beginPacket(0x200);
  CAN.write(send_data, 8);
  CAN.endPacket();
}

// --- PID ---
float pid(float setpoint, float input, float &error_prev, float &integral,
          float kp, float ki, float kd, float dt)
{
  float error = setpoint - input;
  integral += ((error + error_prev) * dt / 2.0f);
  float derivative = (error - error_prev) / dt;
  error_prev = error;

  return kp * error + ki * integral + kd * derivative;
}

float constrain_double(float val, float min_val, float max_val) {
  if (val < min_val) return min_val;
  if (val > max_val) return max_val;
  return val;
}
