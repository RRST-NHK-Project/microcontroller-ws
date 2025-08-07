#include<mcp_can.h>
#include<SPI.h>
//----------------CAN-----------------//
long unsigned int rxId; //モータのcanIDを読み取る
unsigned char len = 0;  //受信したCANメッセージの**データ長（バイト数）**を格納canは8バイト
unsigned char rxBuf[8]; //CANメッセージの**受信データ本体（最大8バイト）**を一時的に格納。
//rxBuf[0~1]:角度,rxBuf[2~3]:rpm,rxBuf[4~5]:電流
byte txBuf[8] = {0};  //CANで送信するデータの格納
unsigned long Pre_millis = 0; //前回受信した時間の記録
int16_t last_encoder_count = -1;        // 前回のカウント（初回は未定）
const int16_t encoder_max = 8191;       // 1回転のカウント数
const double gear_ratio = 36.0; // エンコーダ:モーター軸

MCP_CAN CAN0(5); //CSピン

//------------PIDゲイン-----------//
double kp_pos = 1.5;
double ki_pos = 0.0;
double kd_pos = 0.5;

double kp_vel = 0.0;
double ki_vel = 0.0;
double kd_vel = 0.0;


//----------設定値-------------//
double pos_setpoint = 0.0;  //目標角度(カウント)
double pos_setpoint_deg = 180.0; //角度指定
double pos_input = 0.0; //現在のエンコーダの値
double pos_error_prev = 0.0;        // 前回の角度誤差
double pos_integral = 0.0;          // 角度積分項
double pos_output = 0;            // 角度PID出力（目標速度）


double vel_input = 0.0;             // 現在の速度（rpm）
double vel_error_prev = 0.0;        // 前回の速度誤差
double vel_integral = 0.0;          // 速度積分項
double vel_output = 0.0;            // 速度PID出力（目標電流）

double motor_output_current_A = 0.0; // 出力電流指令（A）

unsigned long lastPidTime = 0; // PID制御の時間計測用

int16_t last_encoder = -1;        // 前回の生エンコーダ値（0〜8191）
int32_t total_encoder_count = 0;         // 累積カウント値（-∞〜∞）
int32_t round_cnt = 0;  //回転した回数
int16_t offset_pulse = 0;   //初期角度


unsigned long lastUpdateTime = 0;   // 最終更新時間(ms)
const unsigned long updateInterval = 0;  // 最低更新間隔(ms)
unsigned long now = 0;


//--------------関数作成-------------//
double pid_control(double setpoint, double input, double &error_prev, double &integral, double kp, double ki, double kd, double dt);
//setpoint:目標値,input:現在の値,&error_prev:前回の誤差,&integral:積分値
double constrain_double(double val, double min_val, double max_val);// 範囲制限

void setup()
{
  calibrate_offset();

  Serial.begin(115200);
  while(!Serial); //初期化まち

 // Serial.print("CAN接続中");
 // SPI初期化（ESP32用にピン指定）
  SPI.begin(18, 19, 23, 5);  // SCK, MISO, MOSI, SS（CS）

 if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
  {
   // Serial.print("接続成功");
    CAN0.setMode(MCP_NORMAL); // 通常動作モードに設定
  }
  else
  {
   // Serial.println("接続失敗");
    while (1);  // 初期化失敗時は停止
  }

  Pre_millis = millis();
  lastPidTime = millis();
  
   // 初期オフセットを記録（起動時のエンコーダ値を基準にする）
  while (!CAN0.checkReceive()); // 最初のメッセージ待ち
  CAN0.readMsgBuf(&rxId, &len, rxBuf);
  offset_pulse = (rxBuf[0] << 8) | rxBuf[1]; // 起動時角度

  last_encoder = offset_pulse;
  total_encoder_count = 0; while (!CAN0.checkReceive());  // 最初の受信を待つ
  CAN0.readMsgBuf(&rxId, &len, rxBuf);
  if (len >= 2) {
    offset_pulse = (rxBuf[0] << 8) | rxBuf[1];
    last_encoder = offset_pulse;
    total_encoder_count = 0;
    round_cnt = 0;
  }
 //  Serial.println("角度\t0度\t目標角度");

 xTaskCreateUniversal(
            CAN_Read_Task,
            "CAN_Read_Task",
            4096,
            NULL,
            1, // 優先度、最大25？
            NULL,
            APP_CPU_NUM);
    


 xTaskCreateUniversal(
            PID_Task,
            "PID_Task",
            4096,
            NULL,
            2, // 優先度、最大25？
            NULL,
            APP_CPU_NUM);
    


}

void CAN_Read_Task(void *pvParameters) {
  while(1){
    now = millis();
  
   //---CAN受信処理---//
   // --- CANメッセージ受信処理 ---
   if (CAN0.checkReceive() == CAN_MSGAVAIL)
   {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    if (len >= 7)
    {
            
    //  Serial.print((rxBuf[0] << 8) | rxBuf[1]); // 生のエンコーダ値（モーター軸）
    //  Serial.print("\t");
    //  Serial.println(motor_output_current_A);

    }
    else
    {
      //Serial.println("でーた大きすぎるて"); // バッファ長が想定外
    }
   }
   else
   {
   // Serial.println("CANメッセージ未受信");
   }

   // --- CAN送信（20ms周期） ---
   if (millis() - Pre_millis > 20)
   {
   double current_limit_A = 10.0;
   motor_output_current_A = constrain_double(motor_output_current_A, -current_limit_A, current_limit_A);

   int16_t motor_output_current_Byte = (int16_t)(motor_output_current_A * 16384.0 / current_limit_A);

   txBuf[0] = (motor_output_current_Byte >> 8) & 0xFF; //流れる電流のデータ
   txBuf[1] = motor_output_current_Byte & 0xFF;
   for (int i = 2; i < 8; i++) txBuf[i] = 0;

   byte result = CAN0.sendMsgBuf(0x200, 0, 8, txBuf);
   if (result != CAN_OK)
    //Serial.println("CAN send fail");

   Pre_millis = millis();
   }
    double motor_angle_deg = total_encoder_count * (360.0 / (8191.0 * gear_ratio));
      pos_input = motor_angle_deg;

  }
}

void PID_Task(void *pvParameters) {
   
  while(1){
    if(len >= 7){
     // 角度、RPMをバッファから復元
      uint16_t pulse = (rxBuf[0] << 8) | rxBuf[1];
      int16_t rpm   = (rxBuf[2] << 8) | rxBuf[3];      
      //pos_input = (double)angle; // 現在位置
      vel_input = (double)rpm;   // 現在速度
     // 最低更新間隔を満たしていれば更新
      if (now - lastUpdateTime >= updateInterval)
      {
        update_total_pulse(pulse);
        lastUpdateTime = now;
      }

 
      // 出力軸の累積角度（degree）を算出（ギア比補正付き）
     
      // PID周期（秒）を算出
      unsigned long now = millis();
      double dt = (now - lastPidTime) / 1000.0;
      if (dt <= 0) dt = 0.05/1000000000; // dtが0にならないようにする
      lastPidTime = now;

      // --- カスケードPID制御 ---//
      // 位置制御（目標角度→目標速度）
      pos_output = pid_control(pos_setpoint, pos_input, pos_error_prev, pos_integral, kp_pos, ki_pos, kd_pos, dt);
      
      if(pos_output<5 && pos_output>-5)//なくてもいい
      pos_output=0;
      // 速度制御（目標速度→出力電流）
     //vel_output = pid_control(vel_output, vel_input, vel_error_prev, vel_integral, kp_vel, ki_vel, kd_vel, dt);
      
      
      // 出力制限（A単位。実際にはC610仕様上 ±10A以内にすべき）
      double current_limit_A = 10;
      double output_limit = 10;
      
      motor_output_current_A = constrain_double(pos_output, -current_limit_A, current_limit_A);
     
     
      //motor_output_current_A=0.1;
      // // デバッグ出力

    }

  }
}


void loop()
{
 delay(5); // デバッグ出力抑制
}
  
//-------関数------//


// --- 汎用PID制御関数 ---//
double pid_control(double setpoint, double input, double &error_prev, double &integral,
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

void update_total_pulse(uint16_t pulse) {

  int32_t delta = pulse - last_encoder;  // int32_tに拡張

  // 1回転未満のジャンプに対する補正
  if (delta > 6000) {
    round_cnt--;
  }
  else if (delta < -6000) {
    round_cnt++;
  }

  // 累積カウント値を計算
  total_encoder_count = round_cnt * encoder_max + pulse - offset_pulse;

  last_encoder = pulse;
}

void calibrate_offset() {
  uint16_t pulse_sum = 0;
  const int sample_count = 10;
  
  for (int i = 0; i < sample_count; i++) {
    while (!CAN0.checkReceive());
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    pulse_sum += (rxBuf[0] << 8) | rxBuf[1];
    delay(5);
  }

  offset_pulse = pulse_sum / sample_count;
  last_encoder = offset_pulse;
}