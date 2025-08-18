#include <Arduino.h>
#include "driver/twai.h"

//---------定義--------//
const int ENCODER_MAX = 8192; //エンコーダの最大
const int HALF_ENCODER = ENCODER_MAX / 2;
constexpr float gear_ratio = 36.0f; // 減速比
constexpr int MOTOR_ID = 1; //モータID
constexpr int   CAN_ID_TX = 0x200 + 2;             // 1〜4にまとめて送るグループID
constexpr int   CAN_ID_RX = 0x200 + MOTOR_ID;      // C610フィードバック

//-----状態量-----//
int16_t last_encoder_count = -1;     // 前回の角度（0〜8191）
int32_t rotation_count = 0;     // 回転数（±）
int32_t total_encoder_count = 0; // 累積カウント（8192カウント/回転）
float angle = 0.0f; //出力軸角度
float vel = 0.0f; //出力速度

int32_t current_position = 0; // 累積角度カウント
float motor_output_current_A = 0.0;
float limit = 19520;
float current_limit_A = 10.0f; // 最大出力電流（例：5A）

//------設定値-----//
float pos_setpoint = 720.0; //目標角度
float pos_input = 0.0; //現在のエンコーダの値
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
float kd_pos = 0.015;//0.02;

// float kp_vel = 1.0;
// float ki_vel = 0.01;
// float kd_vel = 0.05;



//--------------関数作成-------------//
float pid(float setpoint, float input, float &error_prev, float &integral, float kp, float ki, float kd, float dt);
//setpoint:目標値,input:現在の値,&error_prev:前回の誤差,&integral:積分値
float constrain_double(float val, float min_val, float max_val);// 範囲制限

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
    unsigned long now = millis();
    float dt = (now - lastPidTime) / 1000.0;
    if(dt <= 0) dt = 0.000001f; // dtが0にならないようにする
    lastPidTime = now;

    // ===== CAN受信（バッファ内すべて処理） =====
    twai_message_t rx_msg;
    while(twai_receive(&rx_msg, 0) == ESP_OK) {
        if(rx_msg.identifier == CAN_ID_RX) {
            int16_t encoder_count = (rx_msg.data[0] << 8) | rx_msg.data[1];
            int16_t rpm = (rx_msg.data[2] << 8) | rx_msg.data[3];

            if(last_encoder_count != -1) {
                int diff = encoder_count - last_encoder_count;
                if(diff > HALF_ENCODER) rotation_count--;
                else if(diff < -HALF_ENCODER) rotation_count++;
            }

            last_encoder_count = encoder_count;
            total_encoder_count = rotation_count * ENCODER_MAX + encoder_count;
            pos_input = total_encoder_count * (360.0 / (8192.0 * gear_ratio));
            vel_input = (rpm / gear_ratio) * 360.0 / 60.0;
        }
    }

    // ===== PID計算 =====
    float pos_output = pid(pos_setpoint, pos_input, pos_error_prev, pos_integral, kp_pos, ki_pos, kd_pos, dt);
    //float vel_output = pid(pos_output, vel_input, vel_error_prev, vel_integral, kp_vel, ki_vel, kd_vel, dt);
    motor_output_current_A = constrain_double(pos_output, -current_limit_A, current_limit_A);
    //motor_output_current_A = 0.3;

    // ===== CAN送信 =====
    int16_t current_cmd = (int16_t)(motor_output_current_A * 16384.0 / 10.0);
    current_cmd = constrain(current_cmd, -16384, 16384);

    twai_message_t msg;
    msg.identifier = 0x200;
    msg.flags = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 8;
    msg.data[0] = (current_cmd >> 8) & 0xFF;
    msg.data[1] = current_cmd & 0xFF;
    for(int i=2;i<8;i++) msg.data[i] = 0;
    twai_transmit(&msg, 0); // 非ブロッキング

   // Serial.print("raw:"); Serial.print(last_encoder_count);
    //Serial.print(" rot:"); Serial.print(rotation_count);
    //Serial.print(" total:"); Serial.print(total_encoder_count);
   // Serial.print(" angle:"); 
  //  Serial.println(pos_input);
    Serial.println(pos_error_prev);
    //Serial.print(" pos_set:"); Serial.print(pos_setpoint);
    //Serial.print(" cur[A]:"); Serial.println(motor_output_current_A,2);




    delay(1);
}


//-----関数-----//

float pid(float setpoint, float input, float &error_prev, float &integral,
                   float kp, float ki, float kd, float dt)
{
  float error = setpoint - input;
  // if(error<2&&error>-2)
  // error = 0;
  integral += ((error +error_prev)* dt/2);
  float derivative = (error - error_prev) / dt;
  error_prev = error;
  
  return kp * error + ki * integral + kd * derivative;
}

float constrain_double(float val, float min_val, float max_val)
{
  if (val < min_val) return min_val;
  if (val > max_val) return max_val;
  return val;
}
