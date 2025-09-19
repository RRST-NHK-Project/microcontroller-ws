/*
For NHK-Robocon-2026??
ESP32用microROSプログラム。ROSノードからのマイコンIO操作を行う。
ライブラリのインストールが必要(https://github.com/micro-ROS/micro_ros_arduino)
ボードマネージャーはesp32 by Espressif Systemsを選択
Bluetooth経由でのワイヤレスデバッグ機能付き(重いから削除予定　→　2025/09/01削除済み)
2025, NHK-Project, RRST
*/

/*
ToDo
CANの統合
*/

#include <Arduino.h>
#include <CAN.h>
#include <esp32-hal-ledc.h>

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
// **複数のESPを使用する場合はIDを変更** //
#define ID 0
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
// **使用する基板に合わせてモードを変更** //
#define MODE 5
/*
0:基板テスト用（ROSと接続せずに基板のテストのみを行う）※実機で「絶対」に実行しないこと　※テストモードについては下記参照
1:MD専用
2:ロボマス関連データ
3:サーボ・ソレノイドバルブ
4:サーボ・ソレノイドバルブ・スイッチ＋エンコーダー（Pub,Subを同時にしたときの遅延問題が解決できていないため未実装）
5:ロボマス制御
*/
// **テスト内容変更** //
#define TEST_MODE 5
/*
0:何もせず待機
1:MDテスト
2:ENC/SWテスト
3:SERVO/SVテスト
4:未実装
5:CANテスト
*/
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
//**テストモード(MODE 0)について** //
/*
テストモードはROSと接続せずに基板のテストやデバッグを行うためのモードです。
MODEを0に変更することで有効化され、TEST_MODEを変更することでテスト内容を変更します。
書き込み後シリアルモニターでEnterキーを押すとテストモードの動作が開始します。
テスト内容によっては機体の暴走や破損につながるので機体搭載時には「絶対」に実行しないでください。
*/
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
// ROSとの接続にはエージェントが必要です。下記コマンドでエージェント立ち上げを行ってください。USBポートは適宜変更してください。
// sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -v6
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

// microROS関連
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32_multi_array.h>

// ワイヤレスデバッグ
// #include "BluetoothSerial.h"
// BluetoothSerial SerialBT;

//  パルスカウンタ関連
#include "driver/pcnt.h"

// 各アクチュエータの総数を定義
#define MD 8
#define SERVO 8
#define SV 7

// 受信配列の要素数を定義
#define MAX_ARRAY_SIZE 25
/*
0:予備
1~8:MD
9~16:サーボ
17~23:ソレノイドバルブ
24:予備
*/

// パルスカウンタの上限・下限の定義
#define COUNTER_H_LIM 32767
#define COUNTER_L_LIM -32768
#define PCNT_FILTER_VALUE 1023 // 0~1023, 1 = 12.5ns

// MD出力の上限値
#define MD_PWM_MAX 255

// ピンの定義 //
// エンコーダ
#define ENC1_A 12
#define ENC1_B 13
#define ENC2_A 14
#define ENC2_B 15
#define ENC3_A 21
#define ENC3_B 22
#define ENC4_A 23
#define ENC4_B 25

// MD PWM
#define MD1P 21
#define MD2P 22
#define MD3P 23
#define MD4P 25
#define MD5P 26
#define MD6P 27
#define MD7P 2
#define MD8P 4

// MD DIR
#define MD1D 12
#define MD2D 13
#define MD3D 14
#define MD4D 15
#define MD5D 32
#define MD6D 33
#define MD7D 5
#define MD8D 19

// サーボ
#define SERVO1 16
#define SERVO2 17
#define SERVO3 18
#define SERVO4 19
#define SERVO5 21
#define SERVO6 22
#define SERVO7 23
#define SERVO8 25

// ソレノイドバルブ
#define SV1 2
#define SV2 4
#define SV3 5
#define SV4 12
#define SV5 13
#define SV6 14
#define SV7 15

// スイッチ
#define SW1 26 
#define SW2 27
#define SW3 32
#define SW4 33

//ロボマス
#define CAN_RX 4
#define CAN_TX 5
// PWM関連の設定値を定義
// MD用
#define MD_PWM_FREQ 20000   // MDのPWM周波数
#define MD_PWM_RESOLUTION 8 // MDのPWM分解能（8ビット）

// サーボ用
#define SERVO_PWM_FREQ 50       // サーボPWM周波数
#define SERVO_PWM_RESOLUTION 16 // サーボPWM分解能（16ビット）

#define SERVO_PWM_PERIOD_US (1000000.0 / SERVO_PWM_FREQ) // 周波数から周期を計算
#define SERVO_PWM_MAX_DUTY ((1 << SERVO_PWM_RESOLUTION) - 1)
#define SERVO_PWM_SCALE (SERVO_PWM_MAX_DUTY / SERVO_PWM_PERIOD_US)

#define SERVO1_MIN_US 500
#define SERVO1_MAX_US 2500
#define SERVO1_MIN_DEG 0
#define SERVO1_MAX_DEG 270

#define SERVO2_MIN_US 500
#define SERVO2_MAX_US 2500
#define SERVO2_MIN_DEG 0
#define SERVO2_MAX_DEG 180

#define SERVO3_MIN_US 500
#define SERVO3_MAX_US 2500
#define SERVO3_MIN_DEG 0
#define SERVO3_MAX_DEG 180

#define SERVO4_MIN_US 500
#define SERVO4_MAX_US 2500
#define SERVO4_MIN_DEG 0
#define SERVO4_MAX_DEG 180

#define SERVO5_MIN_US 500
#define SERVO5_MAX_US 2500
#define SERVO5_MIN_DEG 0
#define SERVO5_MAX_DEG 180

#define SERVO6_MIN_US 500
#define SERVO6_MAX_US 2500
#define SERVO6_MIN_DEG 0
#define SERVO6_MAX_DEG 180

#define SERVO7_MIN_US 500
#define SERVO7_MAX_US 2500
#define SERVO7_MIN_DEG 0
#define SERVO7_MAX_DEG 180

#define SERVO8_MIN_US 500
#define SERVO8_MAX_US 2500
#define SERVO8_MIN_DEG 0
#define SERVO8_MAX_DEG 180

// *********以下CAN関連********* //

//---------定義--------//
#define ENCODER_MAX  8192// エンコーダの最大
#define HALF_ENCODER 4096 //= ENCODER_MAX / 2;
#define gear_ratio 36.0 // 減速比
#define NUM_MOTORS 4 // 使用するモータの台数（最大4台）

//-----状態量-----//
struct Motor {
  int16_t encoder = 0;
  int16_t rpm = 0;
  int16_t current = 0;
  int16_t last_encoder = -1;
  int32_t rotation_count = 0;
  int32_t total_encoder = 0;
  float angle = 0.0f;
  float corrected_angle = 0;
  float vel = 0.0f;
  bool offset_ok = false;
  int encoder_offset = 0;


  // PID用
  float pos_error_prev = 0.0f;
  float pos_integral = 0.0f;
  float vel_prop_prev = 0.0f;
  float vel_output = 0.0f;
  float target_angle = 0.0f;
  float target_rpm = 0.0f;
  float output_current = 0.0f;
  float change = 0.0f;
};

Motor motors[NUM_MOTORS];

int16_t encoder_count = 0;
int16_t rpm = 0;
int16_t last_encoder_count = -1; // 前回の角度（0〜8191）
int32_t rotation_count = 0;      // 回転数（±）
int32_t total_encoder_count = 0; // 累積カウント（8192カウント/回転）
float angle = 0.0f;              // 出力軸角度
float vel = 0.0f;                // 出力速度
bool offset_ok = false;
int encoder_offset = 0;

int32_t current_position = 0; // 累積角度カウント
float motor_output_current_A = 0.0;
float limit = 19520;
float current_limit_A = 10.0f; // 最大出力電流（例：5A）

//------設定値-----//
float target_angle = 0; // 目標角度
float target_rpm = 0.0; //現在のエンコーダの値
float pos_error_prev = 0.0; // 前回の角度誤差
float pos_integral = 0.0;   // 角度積分項
float pos_output = 0;       // 角度PID出力（目標速度）

float vel_input = 0.0;      // 現在の速度
float vel_prop_prev = 0.0; // 前回の速度誤差
float vel_integral = 0.0;   // 速度積分項
float vel_output = 0;       // 速度PID出力

unsigned long lastPidTime = 0; // PID制御の時間計測用

//------------PIDゲイン-----------//
float kp_pos = 0.4; // 0.4;
float ki_pos = 0.01;
float kd_pos = 0.04; // 0.02;

float kp_th = 0.4; // 0.4;
float ki_th = 0.01;
float kd_th = 0.04; // 0.02;

float kp_vel = 1.0;
float ki_vel = 0.0;
float kd_vel = 0.05;

//------------関数の定義-----------//

void send_cur(float cur[NUM_MOTORS]) {
    constexpr float MAX_CUR = 10;
    constexpr int MAX_CUR_VAL = 1000;//10000;

    auto conv = [&](float c) {
    float val = c * (MAX_CUR_VAL / MAX_CUR);
    if (val < -MAX_CUR_VAL) val = -MAX_CUR_VAL;
    else if (val > MAX_CUR_VAL) val = MAX_CUR_VAL;
    return (int16_t)val;
  };

    uint8_t send_data[8] = {};

    // 電流のデータ送信
    for (int i=0; i<NUM_MOTORS; i++) {
    int16_t transmit_val = conv(cur[i]);
    send_data[i*2]     = (transmit_val >> 8) & 0xFF;
    send_data[i*2 + 1] = transmit_val & 0xFF;
  }

    CAN.beginPacket(0x200);
    CAN.write(send_data, 8);
    CAN.endPacket();
}

float pid(float setpoint, float input, float &error_prev, float &integral,
          float kp, float ki, float kd, float dt) {
    float error = setpoint - input;
    integral += ((error + error_prev) * dt / 2.0f); // 台形積分
    float derivative = (error - error_prev) / dt;
    error_prev = error;

    return kp * error + ki * integral + kd * derivative;
}
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

float constrain_double(float val, float min_val, float max_val) {
    if (val < min_val)
        return min_val;
    if (val > max_val)
        return max_val;
    return val;
}

// *********CAN関連ここまで********* //

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
// rcl_timer_t timer;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ノード名とトピック名の定義（ID付き）
String node_name = "esp32node_" + String(ID, DEC);
String publisher_topic_name = "from_esp32_" + String(ID, DEC);
String subscriber_topic_name = "to_esp32_" + String(ID, DEC);

// 受信データ格納用のバッファ
int32_t buffer[MAX_ARRAY_SIZE];

// 受信データ格納用
volatile int32_t received_data[MAX_ARRAY_SIZE]; // 受信データ
volatile size_t received_size = 0;              // 受信データのサイズ

volatile int32_t received_out_data[MAX_ARRAY_SIZE]; // 受信データ


// エンコーダのカウント格納用
int16_t count[4] = {0};

// スイッチの状態格納用
bool sw_state[4] = {false};

// #define RCCHECK(fn)             \
//     {                           \
//         if ((fn) != RCL_RET_OK) \
//             error_loop();       \
//     }

// 詳細デバッグ
#define RCCHECK(fn)                                                                           \
    do {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                               \
        if ((temp_rc) != RCL_RET_OK) {                                                        \
            /* デバッグ出力 */                                                                \
            /* SerialBT.printf("RCL error at %s:%d -> %d\n", __FILE__, __LINE__, temp_rc); */ \
            error_loop();                                                                     \
        }                                                                                     \
    } while (0)

// エラー発生時のループ
void error_loop() {
    while (1) {
        // このエラーが表示される場合は、シリアルモニターを閉じているか確認
        // SerialBT.println("RCL Error!");
        delay(1000);
    }
}

// コールバック内でグローバル変数にコピー
void subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;
    size_t len = msg->data.size;
    if (len > MAX_ARRAY_SIZE)
        len = MAX_ARRAY_SIZE;

    for (size_t i = 0; i < len; i++) {
        received_data[i] = msg->data.data[i];
    }
    received_size = len;
}

void setup() {
    // SerialBT.begin("ESP32_" + String(ID, DEC)); // Bluetoothの初期化

    // MODEに応じた初期化
    switch (MODE) {
    case 0:
        Serial.begin(115200);
        mode0_init();
        break;
    case 1:
        ros_init();
        mode1_init();
        break;
    case 2:
        ros_init();
        mode2_init();
        break;
    case 3:
        ros_init();
        mode3_init();
        break;
    case 4:
        ros_init();
        mode4_init();
        break;
    case 5:
        ros_init();
        mode5_init();
        break;
    default:;
        ;
    }
}

void loop() {
    if (MODE != 0) {
        RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
        vTaskDelay(1);
    }
}

// micro-ROSの初期化
void ros_init() {
    delay(2000);

    set_microros_transports();
    allocator = rcl_get_default_allocator();

    // Agentと接続できるまでリトライ
    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        // SerialBT.println("Waiting for agent...");
        delay(1000); // 1秒待つ
    }

    // Nodeの初期化
    RCCHECK(rclc_node_init_default(&node, node_name.c_str(), "", &support));

    // Subscriberの初期化
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        subscriber_topic_name.c_str()));

    // Publisherの初期化
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        publisher_topic_name.c_str()));

    std_msgs__msg__Int32MultiArray__init(&msg);
    msg.data.data = buffer;
    msg.data.size = 0;
    msg.data.capacity = MAX_ARRAY_SIZE;

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // 以下のサービスの数でexecutorのサイズを変える。

    // Executorにサービスを追加

    //subscするときには必要
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
    
    // RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void MD_Output_Task(void *pvParameters) {
    while (1) {
        // 以下メインの処理

        // デバッグ用
        // if (received_data[0] == 1) {
        //     SerialBT.print("Received: ");
        //     for (size_t i = 0; i < received_size; i++) {
        //         SerialBT.print(received_data[i]);
        //         SerialBT.print(", ");
        //     }
        //     SerialBT.println();
        // }

        // MD出力の制限
        received_data[1] = constrain(received_data[1], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[2] = constrain(received_data[2], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[3] = constrain(received_data[3], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[4] = constrain(received_data[4], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[5] = constrain(received_data[5], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[6] = constrain(received_data[6], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[7] = constrain(received_data[7], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[8] = constrain(received_data[8], -MD_PWM_MAX, MD_PWM_MAX);

        // ピンの操作
        digitalWrite(MD1D, received_data[1] > 0 ? HIGH : LOW);
        digitalWrite(MD2D, received_data[2] > 0 ? HIGH : LOW);
        digitalWrite(MD3D, received_data[3] > 0 ? HIGH : LOW);
        digitalWrite(MD4D, received_data[4] > 0 ? HIGH : LOW);
        digitalWrite(MD5D, received_data[5] > 0 ? HIGH : LOW);
        digitalWrite(MD6D, received_data[6] > 0 ? HIGH : LOW);
        digitalWrite(MD7D, received_data[7] > 0 ? HIGH : LOW);
        digitalWrite(MD8D, received_data[8] > 0 ? HIGH : LOW);

        // analogWrite(MD1P, abs(received_data[1]));
        // analogWrite(MD2P, abs(received_data[2]));
        // analogWrite(MD3P, abs(received_data[3]));
        // analogWrite(MD4P, abs(received_data[4]));

        ledcWrite(MD1P, abs(received_data[1]));
        ledcWrite(MD2P, abs(received_data[2]));
        ledcWrite(MD3P, abs(received_data[3]));
        ledcWrite(MD4P, abs(received_data[4]));
        ledcWrite(MD5P, abs(received_data[5]));
        ledcWrite(MD6P, abs(received_data[6]));
        ledcWrite(MD7P, abs(received_data[7]));
        ledcWrite(MD8P, abs(received_data[8]));

        vTaskDelay(1); // ウォッチドッグタイマのリセット(必須)
    }
}

void ENC_Read_Task(void *pvParameters) {
    while (1) {
        // SerialBT.println("Reading encoders...");
        pcnt_get_counter_value(PCNT_UNIT_0, &count[0]);
        pcnt_get_counter_value(PCNT_UNIT_1, &count[1]);
        pcnt_get_counter_value(PCNT_UNIT_2, &count[2]);
        pcnt_get_counter_value(PCNT_UNIT_3, &count[3]);

        // デバッグ用
        // if (received_data[0] == 1) {
        //     SerialBT.printf("%d, %d, %d, %d\n", count[0], count[1], count[2], count[3]);
        // }

        msg.data.size = 8;
        msg.data.data[0] = count[0];
        msg.data.data[1] = count[1];
        msg.data.data[2] = count[2];
        msg.data.data[3] = count[3];
        RCCHECK(rcl_publish(&publisher, &msg, NULL));

        vTaskDelay(1); // ウォッチドッグタイマのリセット(必須)
    }
}

void ENC_SW_Read_Publish_Task(void *pvParameters) {
    while (1) {

        // パルスカウンタの値を取得
        pcnt_get_counter_value(PCNT_UNIT_0, &count[0]);
        pcnt_get_counter_value(PCNT_UNIT_1, &count[1]);
        pcnt_get_counter_value(PCNT_UNIT_2, &count[2]);
        pcnt_get_counter_value(PCNT_UNIT_3, &count[3]);

        // スイッチの状態を取得
        sw_state[0] = (digitalRead(SW1) == HIGH);
        sw_state[1] = (digitalRead(SW2) == HIGH);
        sw_state[2] = (digitalRead(SW3) == HIGH);
        sw_state[3] = (digitalRead(SW4) == HIGH);

        msg.data.data[0] = count[0];
        msg.data.data[1] = count[1];
        msg.data.data[2] = count[2];
        msg.data.data[3] = count[3];
        msg.data.data[4] = sw_state[0];
        msg.data.data[5] = sw_state[1];
        msg.data.data[6] = sw_state[2];
        msg.data.data[7] = sw_state[3];

        // Publish
        if (MODE != 0) {
            RCCHECK(rcl_publish(&publisher, &msg, NULL));
        }

        vTaskDelay(1); // ウォッチドッグタイマのリセット(必須)
    }
}

void ROBOMAS_ENC_SW_Read_Publish_Task(void *pvParameters) {
    while (1) {

        // パルスカウンタの値を取得
        pcnt_get_counter_value(PCNT_UNIT_0, &count[0]);
        pcnt_get_counter_value(PCNT_UNIT_1, &count[1]);
        pcnt_get_counter_value(PCNT_UNIT_2, &count[2]);
        pcnt_get_counter_value(PCNT_UNIT_3, &count[3]);

        // スイッチの状態を取得
        sw_state[0] = (digitalRead(SW1) == HIGH);
        sw_state[1] = (digitalRead(SW2) == HIGH);
        sw_state[2] = (digitalRead(SW3) == HIGH);
        sw_state[3] = (digitalRead(SW4) == HIGH);

        // 1. CAN受信
        int packetSize = CAN.parsePacket();
        while (packetSize) {
            int id = CAN.packetId();               // 複数パケットも処理
            if (id >= 0x201 && id < 0x201 + NUM_MOTORS) {
                int idx = id - 0x201; // モータ番号(0から)
                uint8_t rx[8];
                for (int i=0;i<8;i++) rx[i] = CAN.read();

                motors[idx].encoder = (rx[0] << 8) | rx[1];
                motors[idx].rpm = (rx[2] << 8) | rx[3];
                motors[idx].current = (rx[4] << 8) | rx[5];

                // 初回オフセット設定
                if (!motors[idx].offset_ok) {
                    motors[idx].encoder_offset = motors[idx].encoder;
                    motors[idx].last_encoder = -1;
                    motors[idx].rotation_count = 0;
                    motors[idx].total_encoder = 0;
                    motors[idx].pos_integral = 0;
                    motors[idx].pos_error_prev = 0;
                    motors[idx].vel_output = 0;
                    motors[idx].vel_prop_prev = 0;
                    motors[idx].offset_ok = true;
                   // Serial.printf("Motor %d Offset set!\n", idx+1);
                }

                int enc_relative = motors[idx].encoder - motors[idx].encoder_offset;
                if (enc_relative < 0) enc_relative += ENCODER_MAX;

                if (motors[idx].last_encoder != -1) {
                    int diff = motors[idx].encoder - motors[idx].last_encoder;
                    if (diff > HALF_ENCODER) {
                        motors[idx].rotation_count--;
                    }else if (diff < -HALF_ENCODER){
                     motors[idx].rotation_count++;
                    }
                }
                motors[idx].last_encoder = motors[idx].encoder;
                motors[idx].total_encoder = motors[idx].rotation_count * ENCODER_MAX + motors[idx].encoder;
                motors[idx].angle = motors[idx].total_encoder * (360.0 / (8192.0 * gear_ratio));
                motors[idx].vel = (motors[idx].rpm / gear_ratio);
            }
            packetSize = CAN.parsePacket(); // 次の受信も処理
        }
      


        msg.data.data[0] = count[0];
        msg.data.data[1] = count[1];
        msg.data.data[2] = count[2];
        msg.data.data[3] = count[3];
        msg.data.data[4] = sw_state[0];
        msg.data.data[5] = sw_state[1];
        msg.data.data[6] = sw_state[2];
        msg.data.data[7] = sw_state[3];
        msg.data.data[8] = motors[0].angle;
        msg.data.data[9] = motors[1].angle;
        msg.data.data[10] = motors[2].angle;
        msg.data.data[11] = motors[3].angle;
        msg.data.data[12] = motors[0].rpm;
        msg.data.data[13] = motors[1].rpm;
        msg.data.data[14] = motors[2].rpm;
        msg.data.data[15] = motors[3].rpm;
        msg.data.data[16] = motors[0].current;
        msg.data.data[17] = motors[1].current;
        msg.data.data[18] = motors[2].current;
        msg.data.data[19] = motors[3].current;
        
        // Publish
        if (MODE != 0) {
            RCCHECK(rcl_publish(&publisher, &msg, NULL));
        }

        vTaskDelay(1); // ウォッチドッグタイマのリセット(必須)
    }
}


void Servo_Output_Task(void *pvParameters) {
    // for文で書きかえたいところ、、、
    while (1) {
        // サーボ1
        int angle1 = received_data[9];
        if (angle1 < SERVO1_MIN_DEG)
            angle1 = SERVO1_MIN_DEG;
        if (angle1 > SERVO1_MAX_DEG)
            angle1 = SERVO1_MAX_DEG;
        int us1 = map(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG, SERVO1_MIN_US, SERVO1_MAX_US);
        int duty1 = (int)(us1 * SERVO_PWM_SCALE);
        ledcWrite(SERVO1, duty1);

        // サーボ2
        int angle2 = received_data[10];
        if (angle2 < SERVO2_MIN_DEG)
            angle2 = SERVO2_MIN_DEG;
        if (angle2 > SERVO2_MAX_DEG)
            angle2 = SERVO2_MAX_DEG;
        int us2 = map(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG, SERVO2_MIN_US, SERVO2_MAX_US);
        int duty2 = (int)(us2 * SERVO_PWM_SCALE);
        ledcWrite(SERVO2, duty2);

        // サーボ3
        int angle3 = received_data[11];
        if (angle3 < SERVO3_MIN_DEG)
            angle3 = SERVO3_MIN_DEG;
        if (angle3 > SERVO3_MAX_DEG)
            angle3 = SERVO3_MAX_DEG;
        int us3 = map(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG, SERVO3_MIN_US, SERVO3_MAX_US);
        int duty3 = (int)(us3 * SERVO_PWM_SCALE);
        ledcWrite(SERVO3, duty3);

        // サーボ4
        int angle4 = received_data[12];
        if (angle4 < SERVO4_MIN_DEG)
            angle4 = SERVO4_MIN_DEG;
        if (angle4 > SERVO4_MAX_DEG)
            angle4 = SERVO4_MAX_DEG;
        int us4 = map(angle4, SERVO4_MIN_DEG, SERVO4_MAX_DEG, SERVO4_MIN_US, SERVO4_MAX_US);
        int duty4 = (int)(us4 * SERVO_PWM_SCALE);
        ledcWrite(SERVO4, duty4);

        // サーボ5
        int angle5 = received_data[13];
        if (angle5 < SERVO5_MIN_DEG)
            angle5 = SERVO5_MIN_DEG;
        if (angle5 > SERVO5_MAX_DEG)
            angle5 = SERVO5_MAX_DEG;
        int us5 = map(angle5, SERVO5_MIN_DEG, SERVO5_MAX_DEG, SERVO5_MIN_US, SERVO5_MAX_US);
        int duty5 = (int)(us5 * SERVO_PWM_SCALE);
        ledcWrite(SERVO5, duty5);

        // サーボ6
        int angle6 = received_data[14];
        if (angle6 < SERVO6_MIN_DEG)
            angle6 = SERVO6_MIN_DEG;
        if (angle6 > SERVO6_MAX_DEG)
            angle6 = SERVO6_MAX_DEG;
        int us6 = map(angle6, SERVO6_MIN_DEG, SERVO6_MAX_DEG, SERVO6_MIN_US, SERVO6_MAX_US);
        int duty6 = (int)(us6 * SERVO_PWM_SCALE);
        ledcWrite(SERVO6, duty6);

        // サーボ7
        int angle7 = received_data[15];
        if (angle7 < SERVO7_MIN_DEG)
            angle7 = SERVO7_MIN_DEG;
        if (angle7 > SERVO7_MAX_DEG)
            angle7 = SERVO7_MAX_DEG;
        int us7 = map(angle7, SERVO7_MIN_DEG, SERVO7_MAX_DEG, SERVO7_MIN_US, SERVO7_MAX_US);
        int duty7 = (int)(us7 * SERVO_PWM_SCALE);
        ledcWrite(SERVO7, duty7);

        // サーボ8
        int angle8 = received_data[16];
        if (angle8 < SERVO8_MIN_DEG)
            angle8 = SERVO8_MIN_DEG;
        if (angle8 > SERVO8_MAX_DEG)
            angle8 = SERVO8_MAX_DEG;
        int us8 = map(angle8, SERVO8_MIN_DEG, SERVO8_MAX_DEG, SERVO8_MIN_US, SERVO8_MAX_US);
        int duty8 = (int)(us8 * SERVO_PWM_SCALE);
        ledcWrite(SERVO8, duty8);

        vTaskDelay(1); // ウォッチドッグタイマのリセット(必須)
    }
}

void SV_Task(void *pvParameters) {
    while (1) {

        digitalWrite(SV1, received_data[17] ? HIGH : LOW);
        digitalWrite(SV2, received_data[18] ? HIGH : LOW);
        digitalWrite(SV3, received_data[19] ? HIGH : LOW);
        digitalWrite(SV4, received_data[20] ? HIGH : LOW);
        digitalWrite(SV5, received_data[21] ? HIGH : LOW);
        digitalWrite(SV6, received_data[22] ? HIGH : LOW);
        digitalWrite(SV7, received_data[23] ? HIGH : LOW);

        vTaskDelay(1); // ウォッチドッグタイマのリセット(必須)
    }
}

void SW_Task(void *pvParameters) {
    while (1) {
        // スイッチの状態を取得
        sw_state[0] = (digitalRead(SW1) == HIGH);
        sw_state[1] = (digitalRead(SW2) == HIGH);
        sw_state[2] = (digitalRead(SW3) == HIGH);
        sw_state[3] = (digitalRead(SW4) == HIGH);

        // デバッグ用
        // if (received_data[0] == 1) {
        //     SerialBT.printf("%d, %d, %d, %d\n", sw_state[0], sw_state[1], sw_state[2], sw_state[3]);
        // }

        msg.data.size = 8;
        msg.data.data[4] = sw_state[0];
        msg.data.data[5] = sw_state[1];
        msg.data.data[6] = sw_state[2];
        msg.data.data[7] = sw_state[3];
        RCCHECK(rcl_publish(&publisher, &msg, NULL));

        vTaskDelay(1); // ウォッチドッグタイマのリセット(必須)
    }
}

void CAN_Task(void *pvParameters) {
    //static変数追加（前回値を記録するため）
    // static int last_corrected_angle = 0;
     static bool last_MANUALMODE = false;  // 前回のモードを保存
    
       //static bool last_sw = false;
       static bool last_zlimit = false;
       static bool last_rlimit = false;

    while (1) {
        bool MANUALMODE = received_data[0];
        // ---- MANUALモードのとき ----
        
        motors[0].target_rpm   = received_data[5];     
        motors[1].target_rpm   = received_data[6];
        motors[2].target_rpm   = received_data[7];
    
    // ---- AUTOモードのとき ----
        motors[0].change = received_data[1]; 
        motors[1].change = received_data[2];


        motors[2].change = received_data[3] * 142 / 15;

        bool change = received_data[8];
       static  bool last_change = false;
        
        bool zlimit = received_out_data[1];
        bool rlimit = received_out_data[2];
        bool re_count = received_out_data[3];
        
if (last_change != change && change == true) {
        // MANUAL → AUTO(位置制御) に変わった瞬間
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].target_angle = motors[i].target_angle + motors[i].change; 
            
        }
}
last_change = change;  // 状態更新

       
        // モード切り替え検出
        

        unsigned long now = millis();
        float dt = (now - lastPidTime) / 1000.0;
        if (dt <= 0)
            dt = 0.000001f; // dtが0にならないようにする
        lastPidTime = now;

        // 1. CAN受信
        int packetSize = CAN.parsePacket();
        while (packetSize) {
            int id = CAN.packetId();               // 複数パケットも処理
            if (id >= 0x201 && id < 0x201 + NUM_MOTORS) {
                int idx = id - 0x201; // モータ番号(0から)
                uint8_t rx[8];
                for (int i=0;i<8;i++) rx[i] = CAN.read();

                motors[idx].encoder = (rx[0] << 8) | rx[1];
                motors[idx].rpm = (rx[2] << 8) | rx[3];
                motors[idx].current = (rx[4] << 8) | rx[5];
                // 初回オフセット設定
                if (!motors[idx].offset_ok) {
                    motors[idx].encoder_offset = motors[idx].encoder;
                    motors[idx].last_encoder = -1;
                    motors[idx].rotation_count = 0;
                    motors[idx].total_encoder = 0;
                    motors[idx].pos_integral = 0;
                    motors[idx].pos_error_prev = 0;
                    motors[idx].vel_output = 0;
                    motors[idx].vel_prop_prev = 0;
                    motors[idx].offset_ok = true;
                   // Serial.printf("Motor %d Offset set!\n", idx+1);
                }

                int enc_relative = motors[idx].encoder - motors[idx].encoder_offset;
                if (enc_relative < 0) enc_relative += ENCODER_MAX;

                if (motors[idx].last_encoder != -1) {
                    int diff = motors[idx].encoder - motors[idx].last_encoder;
                    if (diff > HALF_ENCODER) {
                        motors[idx].rotation_count--;
                    }else if (diff < -HALF_ENCODER){
                     motors[idx].rotation_count++;
                    }
                    motors[2].rotation_count = motors[2].rotation_count* ((re_count + 1)%2);
                }
                motors[idx].last_encoder = motors[idx].encoder;
                motors[idx].total_encoder = motors[idx].rotation_count * ENCODER_MAX + motors[idx].encoder;
                motors[idx].angle = motors[idx].total_encoder * (360.0 / (8192.0 * gear_ratio));
                motors[idx].vel = (motors[idx].rpm / gear_ratio);
                  
                 

                   // 原点補正処理
           
                }
                
            packetSize = CAN.parsePacket(); // 次の受信も処理
        }
       float cur_cmd[NUM_MOTORS];
  // --- PID計算 & 電流決定(最大は1) ---
    
if (last_MANUALMODE == true && MANUALMODE == false) {
        // MANUAL → AUTO(位置制御) に変わった瞬間
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].target_angle = motors[i].angle;  // 現在位置を目標にする
            motors[i].pos_integral = 0;                // PID積分リセット
            motors[i].pos_error_prev = 0;              // 誤差リセット
        }
}
last_MANUALMODE = MANUALMODE;  // 状態更新

  if (MANUALMODE == false){
     

  //差動の制御  
  for (int i=0; i<2; i++) {
    float pos_out = pid(motors[i].target_angle, motors[i].angle,
                        motors[i].pos_error_prev, motors[i].pos_integral,
                        kp_pos, ki_pos, kd_pos, dt);
    motors[i].output_current = constrain_double(pos_out, -current_limit_A, current_limit_A);
    cur_cmd[i] = motors[i].output_current;
  }
  //θの制御
  float th_out = pid(motors[2].target_angle, motors[2].angle,
                        motors[2].pos_error_prev, motors[2].pos_integral,
                        kp_th, ki_th, kd_th, dt);
    motors[2].output_current = constrain_double(th_out, -current_limit_A, current_limit_A);
    cur_cmd[2] = motors[2].output_current;

  }else if(MANUALMODE == true){



  for(int i=0; i<NUM_MOTORS; i++) {
  float vel_out = pid_vel(motors[i].target_rpm, motors[i].vel,
                        motors[i].pos_error_prev,motors[i].vel_prop_prev, motors[i].vel_output,
                        kp_vel, ki_vel, kd_vel, dt);
    motors[i].output_current = constrain_double(vel_out, -current_limit_A, current_limit_A);
    cur_cmd[i] = motors[i].output_current;
    
    // if (cur_cmd[2]<=0.03 && cur_cmd[2]>=-0.03)
    // cur_cmd[2] = 0;
  }
   
  
  }
  // --- 電流指令送信（2台分まとめて） ---
  send_cur(cur_cmd);
  
  //サーボ--------
    int angle1 = 90;//(motors[2].angle *15/142) ;
        if (angle1 < SERVO1_MIN_DEG)
            angle1 = SERVO1_MIN_DEG;
        if (angle1 > SERVO1_MAX_DEG)
            angle1 = SERVO1_MAX_DEG;
        if (motors[2].angle * 15 / 142 > -20 && motors[2].angle * 15 / 142 < 20)
            angle1 = 135;

        int us1 = map(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG, SERVO1_MIN_US, SERVO1_MAX_US);
        int duty1 = (int)(us1 * SERVO_PWM_SCALE);
        ledcWrite(SERVO1, duty1);
    //-----------

        
        // Serial.print("pos:\t"); Serial.println(angle);
        // Serial.println(target_angle - angle);

        delay(1);
    }
}

void enc_init() {
    // プルアップを有効化
    gpio_set_pull_mode((gpio_num_t)ENC1_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC1_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC2_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC2_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC3_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC3_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC4_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC4_B, GPIO_PULLUP_ONLY);

    // プルダウンを有効化
    // gpio_set_pull_mode((gpio_num_t)ENC1_A, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC1_B, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC2_A, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC2_B, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC3_A, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC3_B, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC4_A, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC4_B, GPIO_PULLDOWN_ONLY);

    // パルスカウンタの設定
    pcnt_config_t pcnt_config1 = {};
    pcnt_config1.pulse_gpio_num = ENC1_A;
    pcnt_config1.ctrl_gpio_num = ENC1_B;
    pcnt_config1.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config1.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config1.pos_mode = PCNT_COUNT_INC;
    pcnt_config1.neg_mode = PCNT_COUNT_DEC;
    pcnt_config1.counter_h_lim = COUNTER_H_LIM;
    pcnt_config1.counter_l_lim = COUNTER_L_LIM;
    pcnt_config1.unit = PCNT_UNIT_0;
    pcnt_config1.channel = PCNT_CHANNEL_0;

    // pcnt_config_t pcnt_config2 = {};
    // pcnt_config2.pulse_gpio_num = ENC1_B;
    // pcnt_config2.ctrl_gpio_num = ENC1_A;
    // pcnt_config2.lctrl_mode = PCNT_MODE_REVERSE;
    // pcnt_config2.hctrl_mode = PCNT_MODE_KEEP;
    // pcnt_config2.pos_mode = PCNT_COUNT_INC;
    // pcnt_config2.neg_mode = PCNT_COUNT_DEC;
    // pcnt_config2.counter_h_lim = COUNTER_H_LIM;
    // pcnt_config2.counter_l_lim = COUNTER_L_LIM;
    // pcnt_config2.unit = PCNT_UNIT_0;
    // pcnt_config2.channel = PCNT_CHANNEL_1;

    pcnt_config_t pcnt_config3 = {};
    pcnt_config3.pulse_gpio_num = ENC2_A;
    pcnt_config3.ctrl_gpio_num = ENC2_B;
    pcnt_config3.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config3.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config3.pos_mode = PCNT_COUNT_INC;
    pcnt_config3.neg_mode = PCNT_COUNT_DEC;
    pcnt_config3.counter_h_lim = COUNTER_H_LIM;
    pcnt_config3.counter_l_lim = COUNTER_L_LIM;
    pcnt_config3.unit = PCNT_UNIT_1;
    pcnt_config3.channel = PCNT_CHANNEL_0;

    // pcnt_config_t pcnt_config4 = {};
    // pcnt_config4.pulse_gpio_num = ENC2_B;
    // pcnt_config4.ctrl_gpio_num = ENC2_A;
    // pcnt_config4.lctrl_mode = PCNT_MODE_REVERSE;
    // pcnt_config4.hctrl_mode = PCNT_MODE_KEEP;
    // pcnt_config4.pos_mode = PCNT_COUNT_INC;
    // pcnt_config4.neg_mode = PCNT_COUNT_DEC;
    // pcnt_config4.counter_h_lim = COUNTER_H_LIM;
    // pcnt_config4.counter_l_lim = COUNTER_L_LIM;
    // pcnt_config4.unit = PCNT_UNIT_1;
    // pcnt_config4.channel = PCNT_CHANNEL_1;

    pcnt_config_t pcnt_config5 = {};
    pcnt_config5.pulse_gpio_num = ENC3_A;
    pcnt_config5.ctrl_gpio_num = ENC3_B;
    pcnt_config5.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config5.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config5.pos_mode = PCNT_COUNT_INC;
    pcnt_config5.neg_mode = PCNT_COUNT_DEC;
    pcnt_config5.counter_h_lim = COUNTER_H_LIM;
    pcnt_config5.counter_l_lim = COUNTER_L_LIM;
    pcnt_config5.unit = PCNT_UNIT_2;
    pcnt_config5.channel = PCNT_CHANNEL_0;

    // pcnt_config_t pcnt_config6 = {};
    // pcnt_config6.pulse_gpio_num = ENC3_B;
    // pcnt_config6.ctrl_gpio_num = ENC3_A;
    // pcnt_config6.lctrl_mode = PCNT_MODE_REVERSE;
    // pcnt_config6.hctrl_mode = PCNT_MODE_KEEP;
    // pcnt_config6.pos_mode = PCNT_COUNT_INC;
    // pcnt_config6.neg_mode = PCNT_COUNT_DEC;
    // pcnt_config6.counter_h_lim = COUNTER_H_LIM;
    // pcnt_config6.counter_l_lim = COUNTER_L_LIM;
    // pcnt_config6.unit = PCNT_UNIT_2;
    // pcnt_config6.channel = PCNT_CHANNEL_1;

    pcnt_config_t pcnt_config7 = {};
    pcnt_config7.pulse_gpio_num = ENC4_A;
    pcnt_config7.ctrl_gpio_num = ENC4_B;
    pcnt_config7.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config7.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config7.pos_mode = PCNT_COUNT_INC;
    pcnt_config7.neg_mode = PCNT_COUNT_DEC;
    pcnt_config7.counter_h_lim = COUNTER_H_LIM;
    pcnt_config7.counter_l_lim = COUNTER_L_LIM;
    pcnt_config7.unit = PCNT_UNIT_3;
    pcnt_config7.channel = PCNT_CHANNEL_0;

    // pcnt_config_t pcnt_config8 = {};
    // pcnt_config8.pulse_gpio_num = ENC4_B;
    // pcnt_config8.ctrl_gpio_num = ENC4_A;
    // pcnt_config8.lctrl_mode = PCNT_MODE_REVERSE;
    // pcnt_config8.hctrl_mode = PCNT_MODE_KEEP;
    // pcnt_config8.pos_mode = PCNT_COUNT_INC;
    // pcnt_config8.neg_mode = PCNT_COUNT_DEC;
    // pcnt_config8.counter_h_lim = COUNTER_H_LIM;
    // pcnt_config8.counter_l_lim = COUNTER_L_LIM;
    // pcnt_config8.unit = PCNT_UNIT_3;
    // pcnt_config8.channel = PCNT_CHANNEL_1;

    // パルスカウンタの初期化
    pcnt_unit_config(&pcnt_config1);
    // pcnt_unit_config(&pcnt_config2);
    pcnt_unit_config(&pcnt_config3);
    // pcnt_unit_config(&pcnt_config4);
    pcnt_unit_config(&pcnt_config5);
    // pcnt_unit_config(&pcnt_config6);
    pcnt_unit_config(&pcnt_config7);
    // pcnt_unit_config(&pcnt_config8);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_pause(PCNT_UNIT_2);
    pcnt_counter_pause(PCNT_UNIT_3);

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_2);
    pcnt_counter_clear(PCNT_UNIT_3);

    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_2);
    pcnt_counter_resume(PCNT_UNIT_3);

    // チャタリング防止のフィルターを有効化
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_filter_enable(PCNT_UNIT_1);
    pcnt_filter_enable(PCNT_UNIT_2);
    pcnt_filter_enable(PCNT_UNIT_3);

    // フィルター値を設定
    pcnt_set_filter_value(PCNT_UNIT_0, PCNT_FILTER_VALUE);
    pcnt_set_filter_value(PCNT_UNIT_1, PCNT_FILTER_VALUE);
    pcnt_set_filter_value(PCNT_UNIT_2, PCNT_FILTER_VALUE);
    pcnt_set_filter_value(PCNT_UNIT_3, PCNT_FILTER_VALUE);
}

// 各モードの初期化関数
void mode1_init() {
    // モード1用の初期化
    // SerialBT.println("Mode 1 Initialized");
    // PWMの初期化
    ledcAttach(MD1P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD2P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD3P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD4P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD5P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD6P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD7P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD8P, MD_PWM_FREQ, MD_PWM_RESOLUTION);

    // 受信＆ピン操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        MD_Output_Task,
        "MD_Output_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        PRO_CPU_NUM);
}

void mode2_init() {
    // モード2用の初期化
    // SerialBT.println("Mode 2 Initialized");

    // エンコーダの初期化
    enc_init();

    // スイッチのピンを入力に設定し内蔵プルアップ抵抗を有効化
    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW3, INPUT_PULLUP);
    pinMode(SW4, INPUT_PULLUP);


    Serial.begin(115200);
    while (!Serial)
        ;

    CAN.setPins(CAN_RX, CAN_TX); // rx.tx
    if (!CAN.begin(1000E3)) {
        Serial.println("Starting CAN failed!");
        while (1)
            ;
    }

    // // エンコーダ取得のスレッド（タスク）の作成
    // xTaskCreateUniversal(
    //     ENC_Read_Task,
    //     "ENC_Read_Task",
    //     4096,
    //     NULL,
    //     2, // 優先度、最大25？
    //     NULL,
    //     APP_CPU_NUM);

    // // スイッチ取得のスレッド（タスク）の作成
    // xTaskCreateUniversal(
    //     SW_Task,
    //     "SW_Task",
    //     4096,
    //     NULL,
    //     2, // 優先度、最大25？
    //     NULL,
    //     APP_CPU_NUM);
    msg.data.data = (int32_t *)malloc(sizeof(int32_t) * 20);
    msg.data.size = 20;
    msg.data.capacity = 20;

    xTaskCreateUniversal(
        ROBOMAS_ENC_SW_Read_Publish_Task,
        "ROBOMAS_ENC_SW_Read_Publish_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);
}

void mode3_init() {
    // モード3用の初期化
    // SerialBT.println("Mode 3 Initialized");

    // サーボのPWMの初期化
    ledcAttach(SERVO1, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO2, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO3, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO4, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO5, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO6, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO7, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO8, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);

    pinMode(SV1, OUTPUT);
    pinMode(SV2, OUTPUT);
    pinMode(SV3, OUTPUT);
    pinMode(SV4, OUTPUT);
    pinMode(SV5, OUTPUT);
    pinMode(SV6, OUTPUT);
    pinMode(SV7, OUTPUT);

    // 内蔵プルアップを有効化
    // gpio_set_pull_mode((gpio_num_t)SV1, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)SV2, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)SV3, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)SV4, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)SV5, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)SV6, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)SV7, GPIO_PULLUP_ONLY);

    // サーボ操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        Servo_Output_Task,
        "Servo_Output_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    // ソレノイド操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        SV_Task,
        "SV_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);
}

void mode4_init() {
    // モード4用の初期化
    // SerialBT.println("Mode 4 Initialized");
    // Pub,Subを同時にしたときの遅延問題が解決できていないため未使用
    // とりあえず書いておく

    // エンコーダの初期化
    enc_init();

    // サーボのPWMの初期化
    ledcAttach(SERVO1, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO2, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO3, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO4, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO5, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO6, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO7, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO8, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);

    pinMode(SV1, OUTPUT);
    pinMode(SV2, OUTPUT);
    pinMode(SV3, OUTPUT);
    pinMode(SV4, OUTPUT);
    pinMode(SV5, OUTPUT);
    pinMode(SV6, OUTPUT);
    pinMode(SV7, OUTPUT);

    // // 内蔵プルアップを有効化
    // gpio_set_pull_mode((gpio_num_t)SV1, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)SV2, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)SV3, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)SV4, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)SV5, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)SV6, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)SV7, GPIO_PULLUP_ONLY);

    // エンコーダ取得のスレッド（タスク）の作成
    xTaskCreateUniversal(
        ENC_Read_Task,
        "ENC_Read_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    // サーボ操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        Servo_Output_Task,
        "Servo_Output_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    // ソレノイド操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        SV_Task,
        "SV_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    // スイッチ取得のスレッド（タスク）の作成
    xTaskCreateUniversal(
        SW_Task,
        "SW_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);
}

void mode5_init() {
    // サーボのPWMの初期化
    ledcAttach(SERVO1, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    pinMode(SV1, OUTPUT);

    
    Serial.begin(115200);
    while (!Serial)
        ;

    CAN.setPins(CAN_RX, CAN_TX); // rx.tx
    if (!CAN.begin(1000E3)) {
        Serial.println("Starting CAN failed!");
        while (1)
            ;
    }

    xTaskCreateUniversal(
        CAN_Task,
        "CAN_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    // ソレノイド操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        SV_Task,
        "SV_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);
}

// テストモード　※実機で「絶対」に実行するな！
// シリアルモニターからEnterが押されるまで待機する
void mode0_init() {
    // デバッグ・テスト用の初期化
    Serial.println("Debug/Test Mode Initialized.");
    Serial.println("Press Enter to continue...");
    // そのうち書く

    // テストモードの安全装置
    while (1) {
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') { // Enterが押されたら抜ける
                break;
            }
        }
    }

    switch (TEST_MODE) {
    case 0:
        // なにもしない
        Serial.println("MODE_DUMMY");
        while (1) {
            ;
        }
        break;
    case 1:
        // MDのテスト
        Serial.println("MD_TEST");
        mode1_init();
        while (1) {
            Serial.println("MD:20%");
            for (int i = 1; i <= 8; i++) {
                received_data[i] = 20;
            }
            delay(1000);

            Serial.println("MD:0%");
            for (int i = 1; i <= 8; i++) {
                received_data[i] = 0;
            }
            delay(1000);

            Serial.println("MD:-20%");
            for (int i = 1; i <= 8; i++) {
                received_data[i] = -20;
            }
            delay(1000);
        }
        break;
    case 2:
        // エンコーダ、スイッチのテスト
        Serial.println("ENC/SW_TEST");
        enc_init();
        mode2_init();
        while (1) {
            for (int i = 0; i < 4; i++) {
                Serial.print("count[");
                Serial.print(i);
                Serial.print("] = ");
                Serial.print(count[i]);
                Serial.print("\t sw_state[");
                Serial.print(i);
                Serial.print("] = ");
                Serial.println(sw_state[i]);
            }
            Serial.println("------");
        }
        break;
    case 3:
        // サーボ、ソレノイドのテスト
        Serial.println("SERVO/SV_TEST");
        mode3_init();
        while (1) {
            for (int i = 9; i <= 16; i++) {
                Serial.print("SERVO");
                Serial.print(i - 8);
                Serial.println(" Sweep");
                for (int angle = 0; angle <= 180; angle += 10) {
                    received_data[i] = angle;
                    delay(500);
                }
                for (int angle = 180; angle >= 0; angle -= 10) {
                    received_data[i] = angle;
                    delay(500);
                }
            }
            for (int i = 17; i <= 23; i++) {
                Serial.print("SV");
                Serial.print(i - 16);
                Serial.println(" ON");
                received_data[i] = 1;
                delay(1000);
                Serial.print("SV");
                Serial.print(i - 16);
                Serial.println(" OFF");
                received_data[i] = 0;
                delay(1000);
            }
        }
        break;

    case 5:
        // サーボ、ソレノイドのテスト
        Serial.println("CAN_TEST");
        mode5_init();
        while (1) {
            unsigned long now = millis();
            float dt = (now - lastPidTime) / 1000.0;
            if (dt <= 0)
                dt = 0.000001f; // dtが0にならないようにする
            lastPidTime = now;

            // 1. CAN受信
            int packetSize = CAN.parsePacket();
            while (packetSize) {               // 複数パケットも処理
                if (CAN.packetId() == 0x201) { // モータID=1
                    uint8_t rx[8];
                    for (int i = 0; i < 8; i++)
                        rx[i] = CAN.read();
                    encoder_count = (rx[0] << 8) | rx[1];
                    rpm = (rx[2] << 8) | rx[3];

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
                    if (enc_relative < 0)
                        enc_relative += ENCODER_MAX; // wrap-around補正

                    if (last_encoder_count != -1) {
                        int diff = encoder_count - last_encoder_count;
                        if (diff > HALF_ENCODER)
                            rotation_count--;
                        else if (diff < -HALF_ENCODER)
                            rotation_count++;
                    }

                    last_encoder_count = encoder_count;
                    total_encoder_count = rotation_count * ENCODER_MAX + encoder_count;
                    angle = total_encoder_count * (360.0 / (8192.0 * gear_ratio));
                    vel_input = (rpm / gear_ratio) * 360.0 / 60.0;
                }
                packetSize = CAN.parsePacket(); // 次の受信も処理
            }
            float pos_output = pid(target_angle, angle, pos_error_prev, pos_integral, kp_pos, ki_pos, kd_pos, dt);
            // float vel_output = pid(pos_output, vel_input, vel_error_prev, vel_integral, kp_vel, ki_vel, kd_vel, dt);
            motor_output_current_A = constrain_double(pos_output, -current_limit_A, current_limit_A);
            // motor_output_current_A = 0.3;
            // 2. コマンド送信
            //send_cur(cur_cmd]);

            // 3. デバッグ出力
            // Serial.print("pos:\t"); Serial.println(angle);
            Serial.println(target_angle - angle);

            delay(1);
        }
        break;

    default:
        Serial.println("Invalid MODE for Test Mode. Enter 0, 1, 2, or 3.");
        while (1) {
            ;
        }
        break;
    }
}

/*
フローチャート（全体構造）

 ┌───────────────┐
 │    setup()    │
 └─────┬─────────┘
       │
       ▼
 ┌─────────────────────────────┐
 │ MODEごとに初期化処理を分岐 │
 │ case 0: mode0_init()        │
 │ case 1: ros_init()+mode1_init() │
 │ case 2: ros_init()+mode2_init() │
 │ case 3: ros_init()+mode3_init() │
 │ case 4: ros_init()+mode4_init() │
 └─────────────────────────────┘
       │
       ▼
 ┌───────────────┐
 │    loop()     │
 └─────┬─────────┘
       │ MODE != 0 ?
       ▼
 ┌──────────────────────────┐
 │ rclc_executor_spin_some()│
 │ vTaskDelay(1)            │
 └──────────────────────────┘
       │
       └───→ 繰り返し

================================================

補助タスク（xTaskCreateUniversalで並列実行されるもの）

- MD_Output_Task(): モータードライバPWM出力
- ENC_Read_Task(): エンコーダ値読み取り → Publish
- ENC_SW_Read_Publish_Task(): エンコーダ＋スイッチ値読み取り → Publish
- Servo_Output_Task(): サーボ角度をPWM出力
- SV_Task(): ソレノイドON/OFF制御
- SW_Task(): スイッチ状態読み取り → Publish

================================================

エラー処理
- RCCHECK() マクロで失敗すると error_loop() に入り無限ループ
GPTくんに書いてもらいました！便利！
*/
