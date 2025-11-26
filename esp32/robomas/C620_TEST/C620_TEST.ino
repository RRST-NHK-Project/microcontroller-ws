#include <CAN.h>

// ロボマス
#define CAN_RX 2
#define CAN_TX 4
#define NUM_MOTOR 4

//#include "c610.hpp"
//---------定義--------//
const int ENCODER_MAX = 8192; //エンコーダの最大
const int HALF_ENCODER = ENCODER_MAX / 2;
//constexpr float gear_ratio = 19.2 // 減速比
constexpr float gear_m3508 = 19.2f; // 減速比(m3508用)
// constexpr int   CAN_ID_TX = 0x200 + 2;             // 1〜4にまとめて送るグループID
// constexpr int   CAN_ID_RX = 0x200 + MOTOR_ID;      // C610フィードバック


// -------- 状態量 / CAN受信関連 -------- //
int encoder_count[NUM_MOTOR] = {0};  // エンコーダ値
int rpm[NUM_MOTOR] = {0};            // 回転速度
int current[NUM_MOTOR] = {0};        // 電流値
bool offset_ok[NUM_MOTOR] = {false}; // オフセット完了フラグ
int encoder_offset[NUM_MOTOR] = {0}; // エンコーダオフセット
int last_encoder[NUM_MOTOR];         // 前回エンコーダ値
int rotation_count[NUM_MOTOR] = {0}; // 回転数
long total_encoder[NUM_MOTOR] = {0}; // 累積エンコーダ値
float angle[NUM_MOTOR] = {0};        // 角度
float vel[NUM_MOTOR] = {0};          // 速度

// -------- PID関連変数 -------- //
float target_angle[NUM_MOTOR] = {0};         // 目標角度
float pos_error_prev[NUM_MOTOR] = {0};       // 前回角度誤差
float cur_error_prev[NUM_MOTOR] = {0};       // 前回角度誤差
float pos_integral[NUM_MOTOR] = {0};         // 角度積分項
float vel_integral[NUM_MOTOR] = {0};
float cur_integral[NUM_MOTOR] = {0};
float pos_output[NUM_MOTOR] = {0};           // PID出力
float cur_output[NUM_MOTOR] = {0};           // PID出力
float motor_output_current[NUM_MOTOR] = {0}; // 出力電流
float output[NUM_MOTOR] = {0};

//速度PID
float target_rpm[NUM_MOTOR] = {0};         // 目標速度
float vel_error_prev[NUM_MOTOR] = {0};       // 前回速度誤差
float vel_prop_prev[NUM_MOTOR] = {0};         // 速度比例項
float vel_output[NUM_MOTOR] = {0};           // 速度PID出力
float vel_out[NUM_MOTOR] = {0};           // 最終速度出力

// -------- 状態量 / CAN受信関連 -------- //
float angle_m3508[NUM_MOTOR] = {0};       // 角度
float vel_m3508[NUM_MOTOR] = {0};         // 速度
//float cur_m3508[NUM_MOTOR] = {0};         // 速度



unsigned long lastPidTime = 0; // PID制御の時間計測用

//------------PIDゲイン-----------//
float kp_pos = 0.8;//0.4;
float ki_pos = 0.01;
float kd_pos = 0.02;//0.02;

float kp_vel = 0.8;
float ki_vel = 0.0;
float kd_vel = 0.05;

float kp_cur = 0.01;
float ki_cur = 0.0;
float kd_cur = 0.0;


float pid(float setpoint, float input, float &error_prev, float &integral,
          float kp, float ki, float kd, float dt)
{
    float error = setpoint - input;
    integral += ((error + error_prev) * dt / 2.0f); // 台形積分
    float derivative = (error - error_prev) / dt;
    error_prev = error;

    return kp * error + ki * integral + kd * derivative;
}

//速度PID計算関数
float pid_vel(float setpoint, float input, float &error_prev,float &prop_prev, float &output,
          float kp, float ki, float kd, float dt){
            float error = setpoint - input;
            float prop = error - error_prev;
            float deriv = prop - prop_prev;
            float du = kp * prop + ki * error * dt + kd * deriv;
            output += du;

            prop_prev = prop;
            error_prev = error;

            return output;
        }


float constrain_double(float val, float min_val, float max_val)
{
    if(val < min_val) return min_val;
    if(val > max_val) return max_val;
    return val;
}


void send_cur_all(float cur_array[NUM_MOTOR]) {
    constexpr float MAX_CUR = 10;
    constexpr int MAX_CUR_VAL = 10000;
    uint8_t send_data[8] = {};

    for (int i = 0; i < NUM_MOTOR; i++) {
        float val = cur_array[i] * (MAX_CUR_VAL / MAX_CUR);
        if (val < -MAX_CUR_VAL)
            val = -MAX_CUR_VAL;
        if (val > MAX_CUR_VAL)
            val = MAX_CUR_VAL;

        int16_t transmit_val = val;
        send_data[i * 2] = (transmit_val >> 8) & 0xFF;
        send_data[i * 2 + 1] = transmit_val & 0xFF;
    }

    CAN.beginPacket(0x200);
    CAN.write(send_data, 8);
    CAN.endPacket();
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
    ;

  CAN.setPins(CAN_RX, CAN_TX);//rx.tx
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }

//   CAN.onReceive(can_callback);
}


void loop() {
unsigned long now = millis();
        float dt = (now - lastPidTime) / 1000.0f;
        if (dt <= 0)
            dt = 0.000001f; // dtが0にならないよう補正
        lastPidTime = now;

        // -------- 目標角度の更新 -------- //
        // received_data[1]～[4] にモータ1～4の目標角度が入っている前提
        for (int i = 0; i < NUM_MOTOR; i++) {
           if (now < 5000) {
        target_rpm[i] = 0;
    }
    else {
        // 5秒経過後、1ずつ増加
       static float rpm_step[NUM_MOTOR] = {100, 100, 100, 100}; 

        // 1ずつ増加
        if (rpm_step[i] < 300) {
            rpm_step[i] += 0.1;
        }

        target_rpm[i] = rpm_step[i];
    }
        }
        

        // -------- CAN受信処理 -------- //
        int packetSize = CAN.parsePacket();
        while (packetSize) {
            int id = CAN.packetId();
            if (id >= 0x201 && id < 0x201 + NUM_MOTOR) {
                int motor_index = id - 0x201;
                uint8_t rx[8];
                for (int i = 0; i < 8; i++)
                    rx[i] = CAN.read();

                // エンコーダ・速度・電流取得
                encoder_count[motor_index] = (rx[0] << 8) | rx[1];
                rpm[motor_index] = (rx[2] << 8) | rx[3];
                current[motor_index] = (rx[4] << 8) | rx[5];

                // 初回オフセット設定
                if (!offset_ok[motor_index]) {
                    encoder_offset[motor_index] = encoder_count[motor_index];
                    last_encoder[motor_index] = -1;
                    rotation_count[motor_index] = 0;
                    total_encoder[motor_index] = 0;
                    pos_integral[motor_index] = 0;
                    pos_error_prev[motor_index] = 0;
                    offset_ok[motor_index] = true;
                }

                // エンコーダ差分とラップ補正
                int enc_relative = encoder_count[motor_index] - encoder_offset[motor_index];
                if (enc_relative < 0)
                    enc_relative += ENCODER_MAX;

                if (last_encoder[motor_index] != -1) {
                    int diff = encoder_count[motor_index] - last_encoder[motor_index];
                    if (diff > HALF_ENCODER)
                        rotation_count[motor_index]--;
                    else if (diff < -HALF_ENCODER)
                        rotation_count[motor_index]++;
                }

                last_encoder[motor_index] = encoder_count[motor_index];
                total_encoder[motor_index] = rotation_count[motor_index] * ENCODER_MAX + encoder_count[motor_index];
                angle_m3508[motor_index] = total_encoder[motor_index] * (360.0f / (ENCODER_MAX * gear_m3508));
                vel_m3508[motor_index] = (rpm[motor_index] / gear_m3508);
            }

            packetSize = CAN.parsePacket(); // 次の受信も処理
        }

        // -------- PID制御（全モータ） -------- //
        for (int i = 0; i < NUM_MOTOR; i++) {
        vel_out[i] = pid_vel(target_rpm[i], vel_m3508[i], vel_error_prev[i], vel_prop_prev[i],vel_output[i], kp_vel, ki_vel, kd_vel, dt);
        cur_output[i] = pid(vel_out[i], current[i], cur_error_prev[i], cur_integral[i], kp_cur, ki_cur, kd_cur, dt);
        //vel_out[i] = pid_vel(100, vel_m3508[i], vel_error_prev[i], vel_prop_prev[i],vel_output[i], kp_vel, ki_vel, kd_vel, dt);
   
         //motor_output_current[i] = cur_output[i];
        motor_output_current[i] =vel_out[i];////pos_output[i]; 
        //constrain_double(motor_output_current[i], -15, 0.5);
}
        // -------- CAN送信（全モータ） -------- //
        send_cur_all(motor_output_current);

  // 3. デバッグ出力
  // Serial.print(current[0]);
  // Serial.print("\t");
  // Serial.println(cur_output[0]);
  
  Serial.print(vel_m3508[0]);
  Serial.print("\t");
  Serial.println(target_rpm[0]);
  
  // //Serial.print("\t");  
  //Serial.println(angle);
  //Serial.println(angle);

  delay(1);
}


