#include <CAN.h>
//#include "c610.hpp"
//---------定義--------//
const int ENCODER_MAX = 8192; //エンコーダの最大
const int HALF_ENCODER = ENCODER_MAX / 2;
constexpr float gear_ratio = 19.2f; // 減速比
// constexpr int   CAN_ID_TX = 0x200 + 2;             // 1〜4にまとめて送るグループID
// constexpr int   CAN_ID_RX = 0x200 + MOTOR_ID;      // C610フィードバック
constexpr int NUMMOTOR = 4;

//-----状態量-----//

int encoder_count[NUMMOTOR] = {0};
int16_t rpm[NUMMOTOR] = {0};
int16_t current[NUMMOTOR] = {0};


int16_t last_encoder_count[NUMMOTOR] = {0};     // 前回の角度（0〜8191）
int32_t rotation_count[NUMMOTOR] = {0};     // 回転数（±）
int32_t total_encoder[NUMMOTOR] = {0}; // 累積カウント（8192カウント/回転）
int last_encoder[NUMMOTOR];         // 前回エンコーダ値
float angle [NUMMOTOR] = {0}; //出力軸角度
float vel[NUMMOTOR] = {0}; //出力速度
bool offset_ok[NUMMOTOR] = {false}; // オフセット完了フラグ
int encoder_offset[NUMMOTOR] = {0};

int32_t current_position[NUMMOTOR] = {0}; // 累積角度カウント
float motor_output_current_A[NUMMOTOR] = {0};
float limit = 19520;
float current_limit_A = 10.0f; // 最大出力電流（例：5A）

//------設定値-----//
float target_angle = 360.0; //目標角度
float target_rpm = 100.0; //目標rpm
float pos_error_prev[NUMMOTOR] = {0};        // 前回の角度誤差
float pos_integral[NUMMOTOR] = {0};          // 角度積分項
float pos_output[NUMMOTOR] = {0};           // 角度PID出力（目標速度）

float vel_input[NUMMOTOR] = {0}; //現在の速度
float vel_error_prev[NUMMOTOR] = {0};        // 前回の速度誤差
float vel_prop_prev[NUMMOTOR] = {0};
float vel_integral[NUMMOTOR] = {0};          // 速度積分項
float vel_output[NUMMOTOR] = {0};            // 速度PID出力
float vel_out[NUMMOTOR] = {0};

unsigned long lastPidTime = 0; // PID制御の時間計測用

//------------PIDゲイン-----------//
float kp_pos = 0.8;
float ki_pos = 0.01;
float kd_pos = 0.02;//0.02;

float kp_vel = 0.3;
float ki_vel = 0.0;
float kd_vel = 0.05;  // 微分は控えめに



//--------------関数作成-------------//
float pid(float setpoint, float input, float &error_prev, float &integral, float kp, float ki, float kd, float dt);
//setpoint:目標値,input:現在の値,&error_prev:前回の誤差,&integral:積分値
float constrain_double(float val, float min_val, float max_val);// 範囲制限

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


void send_cur(float cur[NUMMOTOR]) {
  constexpr float MAX_CUR = 10;
  constexpr int MAX_CUR_VAL = 10000;

  uint8_t send_data[8] = {};
  for (int i = 0; i < NUMMOTOR; i++) {
        float val = cur[i] * (MAX_CUR_VAL / MAX_CUR);
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

  CAN.setPins(2, 4);//rx.tx
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
    while(packetSize) {  // while → if
     int id = CAN.packetId();
       if (id >= 0x201 && id < 0x201 + NUMMOTOR) {
                int motor_index = id - 0x201;
                uint8_t rx[8];
                for (int i = 0; i < 8; i++)
                    rx[i] = CAN.read();

                // エンコーダ・速度・電流取得
                encoder_count[motor_index] = (rx[0] << 8) | rx[1];
                rpm[motor_index] = (rx[2] << 8) | rx[3];
                current[motor_index] = (rx[4] << 8) | rx[5];

             // --- 初回オフセット設定 --- //
      if (!offset_ok[motor_index]) {
                    encoder_offset[motor_index] = encoder_count[motor_index];
                    last_encoder_count[motor_index] = -1;
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

            last_encoder_count[motor_index] = encoder_count[motor_index];
            total_encoder[motor_index] = rotation_count[motor_index] * ENCODER_MAX + encoder_count[motor_index];
            angle[motor_index] = total_encoder[motor_index] * (360.0 / (8192.0 * gear_ratio));
            vel_input[motor_index] = (rpm[motor_index] / gear_ratio);
        }
        packetSize = CAN.parsePacket(); // 次の受信も処理
    }
  
   for (int i = 0; i < NUMMOTOR; i++) {
   pos_output[i] = pid(target_angle, angle[i], pos_error_prev[i], pos_integral[i], kp_pos, ki_pos, kd_pos, dt);
   vel_out[i] = pid_vel(pos_output[i], vel_input[i], vel_error_prev[i], vel_prop_prev[i],vel_output[i], kp_vel, ki_vel, kd_vel, dt);
   

    motor_output_current_A[i] = vel_out[i]; //constrain_double(pos_output, -current_limit_A, current_limit_A);
}
  //motor_output_current_A = 1.0;
  // 2. コマンド送信
  send_cur(motor_output_current_A);

  // 3. デバッグ出力
  
  //Serial.print("pos:\t"); Serial.println(angle);
  // Serial.println(vel_input);
  // Serial.print("\t");
  // Serial.print(target_angle);
  // Serial.print("\tvel:\t");
  // Serial.print(rpm);
   //Serial.print("\t");
   //Serial.println(angle);
  //Serial.println(target_angle-angle);
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
