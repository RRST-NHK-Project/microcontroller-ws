/*
7/31
m2006のエンコーダの部分で位置制御をしようとしていたからうまいこといかへんかった。
ギア比分エンコーダを回転させないいけない
*/
#include<mcp_can.h>
#include<SPI.h>

//----------------CAN-----------------//
long unsigned int rxId; //モータのcanIDを読み取る
unsigned char len = 0;  //受信したCANメッセージの**データ長（バイト数）**を格納canは8バイト
unsigned char rxBuf[8]; //CANメッセージの**受信データ本体（最大8バイト）**を一時的に格納。
//rxBuf[0~1]:角度,rxBuf[2~3]:rpm,rxBuf[4~5]:電流
byte txBuf[8] = {0};  //CANで送信するデータの格納
unsigned long Pre_millis = 0; //前回受信した時間の記録

MCP_CAN CAN0(10); //CSピン

//------------PIDゲイン-----------//
double kp_pos = 0.00;
double ki_pos = 0.00;
double kd_pos = 0.00;

double kp_vel = 0.0;
double ki_vel = 0.0;
double kd_vel = 0.0;


//----------設定値-------------//
double pos_setpoint = 0.0;  //目標角度
double pos_setpoint_deg = 90.0; //角度指定
double pos_input = 0.0; //現在のエンコーダの値
double pos_error_prev = 0.0;        // 前回の角度誤差
double pos_integral = 0.0;          // 角度積分項
double pos_output = 0.0;            // 角度PID出力（目標速度）


double vel_input = 0.0;             // 現在の速度（rpm）
double vel_error_prev = 0.0;        // 前回の速度誤差
double vel_integral = 0.0;          // 速度積分項
double vel_output = 0.0;            // 速度PID出力（目標電流）

double motor_output_current_A = 0.0; // 出力電流指令（A）

unsigned long lastPidTime = 0; // PID制御の時間計測用

//--------------関数作成-------------//
int16_t fmap(double x,double in_min,double in_max,int16_t out_min,int16_t out_max); //x:目標角度
double pid_control(double setpoint, double input, double &error_prev, double &integral, double Kp, double Ki, double Kd, double dt);
//setpoint:目標値,input:現在の値,&error_prev:前回の誤差,&integral:積分値
double constrain_double(double val, double min_val, double max_val);// 範囲制限
double deg_to_count(double deg);// 度→カウント変換
double count_to_deg(double count);// カウント→度変換

void setup()
{
  Serial.begin(115200);
  while(!Serial); //初期化まち

  Serial.print("CAN接続中");

 if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
  {
    Serial.print("接続成功");
    CAN0.setMode(MCP_NORMAL); // 通常動作モードに設定
  }
  else
  {
    Serial.println("接続失敗");
    while (1);  // 初期化失敗時は停止
  }

  Pre_millis = millis();
  lastPidTime = millis();
}

void loop()
{
  //角度指定
  pos_setpoint = deg_to_count(pos_setpoint_deg);

  //---CAN受信処理---//
   // --- CANメッセージ受信処理 ---
  if (CAN0.checkReceive() == CAN_MSGAVAIL)
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    if (len >= 7)
    {
      // 角度、RPMをバッファから復元
      int16_t angle = (rxBuf[0] << 8) | rxBuf[1];
      int16_t rpm   = (rxBuf[2] << 8) | rxBuf[3];

      pos_input = (double)angle; // 現在位置
      vel_input = (double)rpm;   // 現在速度

      // PID周期（秒）を算出
      unsigned long now = millis();
      double dt = (now - lastPidTime) / 1000.0;
      if (dt <= 0) dt = 0.001; // dtが0にならないようにする
      lastPidTime = now;

      // --- カスケードPID制御 ---//
      // 位置制御（目標角度→目標速度）
      pos_output = pid_control(pos_setpoint, pos_input, pos_error_prev, pos_integral, kp_pos, ki_pos, kd_pos, dt);
      // 速度制御（目標速度→出力電流）
      vel_output = pid_control(pos_output, vel_input, vel_error_prev, vel_integral, kp_vel, ki_vel, kd_vel, dt);
      // 出力制限（A単位。実際にはC610仕様上 ±10A以内にすべき）
      motor_output_current_A = constrain_double(vel_output, -1000.0, 1000.0);

      // デバッグ出力
      Serial.print("角度\t");
      Serial.print(count_to_deg(pos_input));
      Serial.print("\tRPM\t");
      Serial.print(rpm);
      Serial.print("\t電流\t");
      Serial.println(motor_output_current_A);
    }
    else
    {
      Serial.println("でーた大きすぎるて"); // バッファ長が想定外
    }
  }
  else
  {
    Serial.println("CANメッセージ未受信");
  }

     // --- CAN送信（20ms周期） ---
  if (millis() - Pre_millis > 20)
  {
    // 電流指令（A）をint16_t値（-16384〜+16384）に変換
    int16_t motor_output_current_Byte = fmap(motor_output_current_A, -10, 10, -10000, 10000);
    txBuf[0] = (motor_output_current_Byte >> 8) & 0xFF;
    txBuf[1] = motor_output_current_Byte & 0xFF;
    for (int i = 2; i < 8; i++) txBuf[i] = 0; // 他のモーターは使っていないので0

    // CAN送信（ID: 0x200、標準フレーム）
    byte result = CAN0.sendMsgBuf(0x200, 0, 8, txBuf);
    if (result != CAN_OK)
      Serial.println("CAN send fail");

    Pre_millis = millis(); // 送信タイマー更新
  }

  delay(10); // デバッグ出力抑制
}
  
//-------関数------//


// --- 汎用PID制御関数 ---//
double pid_control(double setpoint, double input, double &error_prev, double &integral,
                   double Kp, double Ki, double Kd, double dt)
{
  double error = setpoint - input;
  integral += error * dt;
  double derivative = (error - error_prev) / dt;
  error_prev = error;

  return Kp * error + Ki * integral + Kd * derivative;
}

// --- double対応map関数（範囲を再マッピング） ---
int16_t fmap(double x, double in_min, double in_max, int16_t out_min, int16_t out_max)
{
  if (x < in_min) x = in_min;
  if (x > in_max) x = in_max;
  return (int16_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// --- 値を上下限で制限 ---
double constrain_double(double val, double min_val, double max_val)
{
  if (val < min_val) return min_val;
  if (val > max_val) return max_val;
  return val;
}

// --- 度数法→カウント変換（0〜360° → 0〜8192） ---
double deg_to_count(double deg)
{
  return deg * (8192.0 / 360.0);
}

// --- カウント→度数法変換 ---
double count_to_deg(double count)
{
  return count * (360.0 / 8192.0);
}

