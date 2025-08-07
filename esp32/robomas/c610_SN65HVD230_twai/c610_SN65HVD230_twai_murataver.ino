#include <Arduino.h>
#include "driver/twai.h"


int16_t last_raw_angle = -1;     // 前回の角度（0〜8191）
int32_t rotation_count = 0;     // 回転数（±）
int32_t total_encoder_count = 0; // 累積カウント（8192カウント/回転）
const int ENCODER_MAX = 8192;
const int HALF_ENCODER = ENCODER_MAX / 2;
const double gear_ratio = 36.0; // エンコーダ:モーター軸

const int MOTOR_ID = 1;
const int CAN_ID_RX = 0x200 + MOTOR_ID;
int32_t current_position = 0; // 累積角度カウント
double motor_output_current_A = 0.0;
double current_limit_A = 10.0; // 最大出力電流（例：5A）


//------------PIDゲイン-----------//
double kp_pos = 1.5;
double ki_pos = 0.0;
double kd_pos = 0.8;

//----------設定値-------------//
double pos_setpoint = 0.0;  //目標角度(カウント)
double pos_setpoint_deg = 180.0; //角度指定
double pos_input = 0.0; //現在のエンコーダの値
double pos_error_prev = 0.0;        // 前回の角度誤差
double pos_integral = 0.0;          // 角度積分項
double pos_output = 0;            // 角度PID出力（目標速度）
unsigned long lastPidTime = 0; // PID制御の時間計測用

//--------------関数作成-------------//
double pid(double setpoint, double input, double &error_prev, double &integral, double kp, double ki, double kd, double dt);
//setpoint:目標値,input:現在の値,&error_prev:前回の誤差,&integral:積分値
double constrain_double(double val, double min_val, double max_val);// 範囲制限

void setup() {
  Serial.begin(115200);

  // CAN設定 (TX: GPIO26, RX: GPIO25)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)26, (gpio_num_t)25, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // ←1Mbpsで設定
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

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
// PID出力を電流に変換
int16_t current_cmd = (int16_t)(motor_output_current_A * 16384.0 / 20.0);
current_cmd = constrain(current_cmd, -16384, 16384);  // 安全制限

// === 電流指令を送信 ===
twai_message_t msg;
msg.identifier = 0x200 + 1;  // モータID1への指令
msg.flags = TWAI_MSG_FLAG_NONE;
msg.data_length_code = 8;
msg.data[0] = (current_cmd >> 8) & 0xFF;
msg.data[1] = current_cmd & 0xFF;

// 残りのモータには指令0
for (int i = 2; i < 8; i++) msg.data[i] = 0;

if (twai_transmit(&msg, pdMS_TO_TICKS(10)) != ESP_OK) {
  Serial.println("CAN送信失敗");
}


  // === C610からのフィードバック受信 ===
  twai_message_t rx_msg;
 // C610からの受信
if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
  if (rx_msg.identifier == CAN_ID_RX) {
    int16_t raw_angle = (rx_msg.data[0] << 8) | rx_msg.data[1];

    if (last_raw_angle != -1) {
      int diff = raw_angle - last_raw_angle;

      // オーバーフロー・アンダーフロー補正
      if (diff > HALF_ENCODER) {
        rotation_count--;  // 負方向に1回転
      } else if (diff < -HALF_ENCODER) {
        rotation_count++;  // 正方向に1回転
      }
    }

    last_raw_angle = raw_angle;
    total_encoder_count = rotation_count * ENCODER_MAX + raw_angle;
    current_position = total_encoder_count;
    double motor_angle_deg = total_encoder_count * (360.0 / (8191.0 * gear_ratio));
    pos_input = motor_angle_deg;

    unsigned long now = millis();
      double dt = (now - lastPidTime) / 1000.0;
      if (dt <= 0) dt = 0.05/1000000000; // dtが0にならないようにする
      lastPidTime = now;
      pos_output = pid(pos_setpoint,pos_input,pos_error_prev,pos_integral,kp_pos,ki_pos,kd_pos, dt);
      motor_output_current_A = constrain_double(pos_output, -current_limit_A, current_limit_A);
    Serial.print("現在角度: "); Serial.print(motor_angle_deg);
   // Serial.print("  回転数: "); Serial.print(rotation_count);
    Serial.print("  累積カウント: "); Serial.print(total_encoder_count);
    Serial.print("  出力: "); Serial.println(pos_output);
  }
}
  delay(10);  // 100Hz
}

double pid(double setpoint, double input, double &error_prev, double &integral,
                   double kp, double ki, double kd, double dt)
{
  double error = setpoint - input;
  if(error<5&&error>-5)
  error = 0;
  integral += ((error +error_prev)* dt/2);
  double derivative = (error - error_prev) / dt;
  error_prev = error;
  
  return kp * error + ki * integral + kd * derivative;
}

double constrain_double(double val, double min_val, double max_val)
{
  if (val < min_val) return min_val;
  if (val > max_val) return max_val;
  return val;
}
