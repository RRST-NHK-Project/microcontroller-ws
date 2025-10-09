#include <CAN.h>
//#include "c610.hpp"
//---------定義--------//
const int ENCODER_MAX = 8192; //エンコーダの最大
const int HALF_ENCODER = ENCODER_MAX / 2;
constexpr float gear_ratio = 19.0f; // 減速比
constexpr int motor_id = 1; //モータID
// constexpr int   CAN_ID_TX = 0x200 + 2;             // 1〜4にまとめて送るグループID
// constexpr int   CAN_ID_RX = 0x200 + MOTOR_ID;      // C610フィードバック


//-----状態量-----//

int16_t encoder_count = 0;
int16_t rpm = 0;
int16_t last_encoder_count = -1;     // 前回の角度（0〜8191）
int32_t rotation_count = 0;     // 回転数（±）
int32_t total_encoder_count = 0; // 累積カウント（8192カウント/回転）
float angle = 0.0f; //出力軸角度
float vel = 0.0f; //出力速度
bool offset_ok = false;constexpr float MAX_CUR = 10;          // 最大 10A
int encoder_offset = 0;

int32_t current_position = 0; // 累積角度カウント
float motor_output_current_A = 0.0;
float limit = 19520;
float current_limit_A = 10.0f; // 最大出力電流（例：5A）

//------設定値-----//
float target_angle = 720.0; //目標角度
//float encoder_count = 0.0; //現在のエンコーダの値
float pos_error_prev = 0.0;        // 前回の角度誤差
float pos_integral = 0.0;          // 角度積分項
float pos_output = 0;            // 角度PID出力（目標速度）

float vel_input = 0.0; //現在の速度
float vel_error_prev = 0.0;        // 前回の速度誤差
float vel_integral = 0.0;          // 速度積分項
float vel_output = 0;            // 速度PID出力

unsigned long lastPidTime = 0; // PID制御の時間計測用

//------------PIDゲイン-----------//
float kp_pos = 0.8;//0.4;
float ki_pos = 0.01;
float kd_pos = 0.02;//0.02;

// float kp_vel = 1.0;
// float ki_vel = 0.01;
// float kd_vel = 0.05;



//--------------関数作成-------------//
float pid(float setpoint, float input, float &error_prev, float &integral, float kp, float ki, float kd, float dt);
//setpoint:目標値,input:現在の値,&error_prev:前回の誤差,&integral:積分値
float constrain_double(float val, float min_val, float max_val);// 範囲制限

void send_cur(float cur) {
  constexpr float MAX_CUR = 10;
  constexpr int MAX_CUR_VAL = 10000;

  float val = cur * (MAX_CUR_VAL / MAX_CUR);
  if (val < -MAX_CUR_VAL) val = -MAX_CUR_VAL;
  else if (val > MAX_CUR_VAL) val = MAX_CUR_VAL;
  int16_t transmit_val = val;

  uint8_t send_data[8] = {};


  //電流のデータ送信
  send_data[(motor_id - 1) * 2] = (transmit_val >> 8) & 0xFF;
  send_data[(motor_id - 1) * 2 + 1] = transmit_val & 0xFF;
  CAN.beginPacket(0x200);
  CAN.write(send_data, 8);
  CAN.endPacket();
}

// void can_callback(int packetSize) {
//   // Serial.print("RX ID: 0x");
//   // Serial.print(CAN.packetId(), HEX);
//   // Serial.print(" DLC:");
//   // Serial.println(packetSize);
//    static uint8_t received_data[8];
//   if (CAN.packetId() == 0x200 + motor_id) {
//     counter++;
//     int data_size_counter = 0;
//     while (CAN.available()) {
//       received_data[data_size_counter % 8] = CAN.read();
//       data_size_counter++;
//     }
//     //Serial.println();
//     data_size_error_flag_ = (data_size_counter != 8);  // 8byte以上のデータが溜まってないか一応チェック

//     pos = static_cast<int16_t>((received_data[0] << 8) + received_data[1]) ; //0~8192に
//     vel = static_cast<int16_t>((received_data[2] << 8) + received_data[3]) ;//rpm
//   }
// }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
    ;

  CAN.setPins(5, 4);//rx.tx
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }

//   CAN.onReceive(can_callback);
}


void loop() {
  unsigned long now = millis();
    float dt = (now - lastPidTime) / 1000.0;
    if(dt <= 0) dt = 0.000001f; // dtが0にならないようにする
    lastPidTime = now;

  // 1. CAN受信
  int packetSize = CAN.parsePacket();
    while (packetSize) {  // 複数パケットも処理
        if (CAN.packetId() == 0x201) { // モータID=1
            uint8_t rx[8];
            for(int i=0;i<8;i++) rx[i] = CAN.read();
            encoder_count = (rx[0]<<8)|rx[1];
            rpm = (rx[2]<<8)|rx[3];

             // --- 初回オフセット設定 --- //
      if (!offset_ok) {
        encoder_offset = encoder_count;
        last_encoder_count = -1;
        rotation_count = 0;
        total_encoder_count = 0;
        pos_integral = 0;
        pos_error_prev = 0;
        offset_ok = true;
        Serial.println("Offset set!");
      }

      int enc_relative = encoder_count - encoder_offset;
      if (enc_relative < 0) enc_relative += ENCODER_MAX; // wrap-around補正

              if(last_encoder_count != -1) {
                int diff = encoder_count - last_encoder_count;
                if(diff > HALF_ENCODER) rotation_count--;
                else if(diff < -HALF_ENCODER) rotation_count++;
            }

            last_encoder_count = encoder_count;
            total_encoder_count = rotation_count * ENCODER_MAX + encoder_count;
            angle = total_encoder_count * (360.0 / (8192.0 * gear_ratio));
            vel_input = (rpm / gear_ratio) * 360.0 / 60.0;
        }
        packetSize = CAN.parsePacket(); // 次の受信も処理
    }
float pos_output = pid(target_angle, angle, pos_error_prev, pos_integral, kp_pos, ki_pos, kd_pos, dt);
    //float vel_output = pid(pos_output, vel_input, vel_error_prev, vel_integral, kp_vel, ki_vel, kd_vel, dt);
    motor_output_current_A = 1.2; //constrain_double(pos_output, -current_limit_A, current_limit_A);
    //motor_output_current_A = 0.3;
  // 2. コマンド送信
  send_cur(motor_output_current_A);

  // 3. デバッグ出力
  //Serial.print("pos:\t"); Serial.println(angle);
  Serial.print(encoder_count);
  Serial.print("vel:\t");
  Serial.println(rpm);

  delay(1);
}


float pid(float setpoint, float input, float &error_prev, float &integral,
          float kp, float ki, float kd, float dt)
{
    float error = setpoint - input;
    integral += ((error + error_prev) * dt / 2.0f); // 台形積分
    float derivative = (error - error_prev) / dt;
    error_prev = error;

    return kp * error + ki * integral + kd * derivative;
}

float constrain_double(float val, float min_val, float max_val)
{
    if(val < min_val) return min_val;
    if(val > max_val) return max_val;
    return val;
}
