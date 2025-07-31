#include <mcp_can.h>
#include <SPI.h>

// --- CAN関連 ---
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
byte txBuf[8] = {0};
unsigned long Pre_millis = 0;

MCP_CAN CAN0(10); // UNOのCSピン=10

// --- PID制御パラメータ ---
double Kp_pos = 0.01;
double Ki_pos = 0.0;
double Kd_pos = 0.001;

double Kp_vel = 0.001;
double Ki_vel = 0.0;
double Kd_vel = 0.0001;



// --- PID変数 ---
double pos_setpoint = 0.0;       // カウントで指定
double pos_setpoint_deg = 90.0;  // ← 角度で指定（例：90°）
double pos_input = 0.0;
double pos_error_prev = 0.0;
double pos_integral = 0.0;
double pos_output = 0.0;

double vel_input = 0.0;
double vel_error_prev = 0.0;
double vel_integral = 0.0;
double vel_output = 0.0;

double motor_output_current_A = 0.0;

unsigned long lastPidTime = 0;

// --- 関数プロトタイプ ---
int16_t fmap(double x, double in_min, double in_max, int16_t out_min, int16_t out_max);
double pid_control(double setpoint, double input, double &error_prev, double &integral, double Kp, double Ki, double Kd, double dt);
double constrain_double(double val, double min_val, double max_val);
double deg_to_count(double deg);
double count_to_deg(double count);

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Initializing CAN...");

  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
  {
    Serial.println("CAN0 Init OK");
    CAN0.setMode(MCP_NORMAL);
  }
  else
  {
    Serial.println("CAN0 Init Failed");
    while (1);
  }

  Pre_millis = millis();
  lastPidTime = millis();
}

void loop()
{
  //Serial.println("loop running...");

  // 角度指定 → カウント値に変換して使用
  pos_setpoint = deg_to_count(pos_setpoint_deg);

  // --- CAN受信処理 ---
  if (CAN0.checkReceive() == CAN_MSGAVAIL)
  {
    //Serial.println("CAN MSG available!");
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    if (len >= 7)
    {
      int16_t angle = (rxBuf[0] << 8) | rxBuf[1];
      int16_t rpm = (rxBuf[2] << 8) | rxBuf[3];

      pos_input = (double)angle;
      vel_input = (double)rpm;

      // PID制御の時間差
      unsigned long now = millis();
      double dt = (now - lastPidTime) / 1000.0;
      if (dt <= 0) dt = 0.001;
      lastPidTime = now;

      // カスケードPID制御
      pos_output = pid_control(pos_setpoint, pos_input, pos_error_prev, pos_integral, Kp_pos, Ki_pos, Kd_pos, dt);
      vel_output = pid_control(pos_output, vel_input, vel_error_prev, vel_integral, Kp_vel, Ki_vel, Kd_vel, dt);
      motor_output_current_A = constrain_double(vel_output, -1000.0, 1000.0);

      Serial.print("Angle [deg]: ");
      Serial.print(count_to_deg(pos_input));
      Serial.print(", RPM: ");
      Serial.print(rpm);
      Serial.print(", Current Cmd: ");
      Serial.println(motor_output_current_A);
    }
    else
    {
      Serial.println("Invalid data length!");
    }
  }
  else
  {
    Serial.println("No CAN message...");
  }

  // --- CAN送信（20msごと） ---
  if (millis() - Pre_millis > 20)
  {
    int16_t motor_output_current_Byte = fmap(motor_output_current_A, -1000, 1000, -16384, 16384);
    txBuf[0] = (motor_output_current_Byte >> 8) & 0xFF;
    txBuf[1] = motor_output_current_Byte & 0xFF;
    for (int i = 2; i < 8; i++) txBuf[i] = 0;

    byte result = CAN0.sendMsgBuf(0x200, 0, 8, txBuf);
    if (result == CAN_OK)
      //Serial.println("CAN send success");
    ;
    else
      Serial.println("CAN send fail");

    Pre_millis = millis();
  }

  delay(10); // デバッグ出力が多すぎないように
}

// void loop()
// {
//   // --- 固定速度の目標値を設定 ---
//   double vel_setpoint_rpm = 10000.0; // ★ここで目標速度を変えて調整できます

//   // --- CAN受信処理 ---
//   if (CAN0.checkReceive() == CAN_MSGAVAIL)
//   {
//     CAN0.readMsgBuf(&rxId, &len, rxBuf);

//     if (len >= 7)
//     {
//       int16_t angle = (rxBuf[0] << 8) | rxBuf[1];
//       int16_t rpm = (rxBuf[2] << 8) | rxBuf[3];

//       vel_input = (double)rpm;

//       unsigned long now = millis();
//       double dt = (now - lastPidTime) / 1000.0;
//       if (dt <= 0) dt = 0.001;
//       lastPidTime = now;

//       // ★ 速度ループのみ実行（位置ループはスキップ）
//       vel_output = pid_control(vel_setpoint_rpm, vel_input, vel_error_prev, vel_integral, Kp_vel, Ki_vel, Kd_vel, dt);
//       motor_output_current_A = constrain_double(vel_output, -40.0, 40.0);

//       Serial.print("RPM: ");
//       Serial.print(vel_input);
//       Serial.print(", Target: ");
//       Serial.print(vel_setpoint_rpm);
//       Serial.print(", Current Cmd: ");
//       Serial.println(motor_output_current_A);
//     }
//   }

//   // --- CAN送信（20msごと） ---
//   if (millis() - Pre_millis > 20)
//   {
//     int16_t motor_output_current_Byte = fmap(motor_output_current_A, -40, 40, -16384, 16384);
//     txBuf[0] = (motor_output_current_Byte >> 8) & 0xFF;
//     txBuf[1] = motor_output_current_Byte & 0xFF;
//     for (int i = 2; i < 8; i++) txBuf[i] = 0;

//     byte result = CAN0.sendMsgBuf(0x200, 0, 8, txBuf);
//     if (result != CAN_OK)
//       Serial.println("CAN send fail");

//     Pre_millis = millis();
//   }

//   delay(10);
// }


// --- PID制御 ---
double pid_control(double setpoint, double input, double &error_prev, double &integral, double Kp, double Ki, double Kd, double dt)
{
  double error = setpoint - input;
  integral += error * dt;
  double derivative = (error - error_prev) / dt;
  error_prev = error;

  return Kp * error + Ki * integral + Kd * derivative;
}

// --- マッピング関数 ---
int16_t fmap(double x, double in_min, double in_max, int16_t out_min, int16_t out_max)
{
  if (x < in_min) x = in_min;
  if (x > in_max) x = in_max;
  return (int16_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// --- 制限関数 ---
double constrain_double(double val, double min_val, double max_val)
{
  if (val < min_val) return min_val;
  if (val > max_val) return max_val;
  return val;
}

// --- 度数法↔カウント変換 ---
double deg_to_count(double deg)
{
  return deg * (8192.0 / 360.0);
}

double count_to_deg(double count)
{
  return count * (360.0 / 8192.0);
}
