/*
For NHK-Robocon-2026??
ESP32用microROSプログラム。ROSノードからのマイコンIO操作を行う。
Bluetooth経由でのワイヤレスデバッグ機能付き(重いから削除予定)→削除済み
2025, NHK-Project, RRST
*/
#include <Arduino.h>
#include <esp32-hal-ledc.h>

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
// **複数のESPを使用する場合はIDを変更** //
#define ID 0
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
// **使用する基板に合わせてモードを変更** //
#define MODE 3
/*
0:デバッグ・テスト用
1:MD専用
2:エンコーダー専用
3:サーボ・ソレノイドバルブ・スイッチ
4:サーボ・ソレノイドバルブ・スイッチ＋エンコーダー
*/
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

// microROS関連
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32_multi_array.h>

// ワイヤレスデバッグ
//#include "BluetoothSerial.h"
//BluetoothSerial SerialBT;

//  パルスカウンタ関連
#include "driver/pcnt.h"

// 各アクチュエータの総数を定義
#define MD 8
#define SERVO 8
#define SV 7

// 受信配列の要素数を事前に定義
#define MAX_ARRAY_SIZE 25

// パルスカウンタの上限・下限の定義
#define COUNTER_H_LIM 32767
#define COUNTER_L_LIM -32768
#define PCNT_FILTER_VALUE 1023  // 0~1023, 1 = 12.5ns

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


// PWM関連の設定値を定義
//MD用
#define MD_PWM_FREQ 20000    // MDのPWM周波数
#define MD_PWM_RESOLUTION 8  // MDのPWM分解能（8ビット）
//

// サーボ用
#define SERVO_PWM_FREQ 50        // サーボPWM周波数
#define SERVO_PWM_RESOLUTION 16  // サーボPWM分解能（16ビット）

#define SERVO_PWM_PERIOD_US (1000000.0 / SERVO_PWM_FREQ)  // 周波数から周期を計算
#define SERVO_PWM_MAX_DUTY ((1 << SERVO_PWM_RESOLUTION) - 1)
#define SERVO_PWM_SCALE (SERVO_PWM_MAX_DUTY / SERVO_PWM_PERIOD_US)

#define SERVO1_MIN_US 500
#define SERVO1_MAX_US 2500
#define SERVO1_MIN_DEG 0
#define SERVO1_MAX_DEG 180

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
//

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
// rcl_timer_t timer;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// ノード名とトピック名の定義（ID付き）
String node_name = "esp32node_" + String(ID, DEC);
String publisher_topic_name = "from_esp32_" + String(ID, DEC);
String subscriber_topic_name = "to_esp32_" + String(ID, DEC);

// 受信データ格納用のバッファ
int32_t buffer[MAX_ARRAY_SIZE];

// 受信データ格納用
volatile int32_t received_data[MAX_ARRAY_SIZE];  // 受信データ
volatile size_t received_size = 0;               // 受信データのサイズ

// エンコーダのカウント格納用
int16_t count[4] = { 0 };

// スイッチの状態格納用
bool sw_state[4] = { false };

// #define RCCHECK(fn)             \
//     {                           \
//         if ((fn) != RCL_RET_OK) \
//             error_loop();       \
//     }

// 詳細デバッグ用マクロ
#define RCCHECK(fn) \
  do { \
    rcl_ret_t temp_rc = (fn); \
    if (temp_rc != RCL_RET_OK) { \
      /* デバッグ出力 */ \
      /* SerialBT.printf("RCL error at %s:%d -> %d\n", __FILE__, __LINE__, temp_rc); */ \
      error_loop(); \
    } \
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
void mode0_init() {
  // デバッグ・テスト用の初期化
  //SerialBT.println("Debug/Test Mode Initialized");
  // そのうち書く
}

void mode1_init() {
  // モード1用の初期化
  //SerialBT.println("Mode 1 Initialized");
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
    2,  // 優先度、最大25？
    NULL,
    PRO_CPU_NUM);
}

void mode2_init() {
  // モード2用の初期化
  //SerialBT.println("Mode 2 Initialized");

  // エンコーダの初期化
  enc_init();

  // エンコーダ取得のスレッド（タスク）の作成
  xTaskCreateUniversal(
    ENC_Read_Task,
    "ENC_Read_Task",
    4096,
    NULL,
    2,  // 優先度、最大25？
    NULL,
    APP_CPU_NUM);
}

void mode3_init() {
  // モード3用の初期化
  //SerialBT.println("Mode 3 Initialized");

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

  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);


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
    2,  // 優先度、最大25？
    NULL,
    APP_CPU_NUM);

  // IO操作のスレッド（タスク）の作成
  xTaskCreateUniversal(
    IO_Task,
    "IO_Task",
    4096,
    NULL,
    2,  // 優先度、最大25？
    NULL,
    APP_CPU_NUM);

  // SW操作のスレッド（タスク）の作成
  xTaskCreateUniversal(
    SW_Task,
    "SW_Task",
    4096,
    NULL,
    2,  // 優先度、最大25？
    NULL,
    APP_CPU_NUM);
}

void mode4_init() {
  // モード4用の初期化
  //SerialBT.println("Mode 4 Initialized");
  //エンコーダとIO操作が同時にできるようになってから実装する予定
  //とりあえず書いておく

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
    2,  // 優先度、最大25？
    NULL,
    APP_CPU_NUM);

  // サーボ操作のスレッド（タスク）の作成
  xTaskCreateUniversal(
    Servo_Output_Task,
    "Servo_Output_Task",
    4096,
    NULL,
    2,  // 優先度、最大25？
    NULL,
    APP_CPU_NUM);

  // IO操作のスレッド（タスク）の作成
  xTaskCreateUniversal(
    IO_Task,
    "IO_Task",
    4096,
    NULL,
    2,  // 優先度、最大25？
    NULL,
    APP_CPU_NUM);
}

void setup() {
  //SerialBT.begin("ESP32_" + String(ID, DEC));  // Bluetoothの初期化
  delay(2000);

  set_microros_transports();
  allocator = rcl_get_default_allocator();

  // Agentと接続できるまでリトライ
  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    // SerialBT.println("Waiting for agent...");
    delay(1000);  // 1秒待つ
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

  // Timer（20ms周期） ← 修正
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,  // ここを executor.context → &support に変更
    RCL_MS_TO_NS(20),
    publisher_timer_callback));

  // Executorの初期化
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));  // サービスの数でサイズを変える

  // Executorにサブスクリプションとタイマーを追加
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Executorをタスク化
  xTaskCreatePinnedToCore(executor_task, "executor_task", 8192, NULL, 2, NULL, APP_CPU_NUM);


  switch (MODE) {
    case 0:
      //SerialBT.println("Mode: Debug/Test");
      mode0_init();
      break;
    case 1:
      //SerialBT.println("Mode: MD Control");
      mode1_init();
      break;
    case 2:
      //SerialBT.println("Mode: Encoder Control");
      mode2_init();
      break;
    case 3:
      //SerialBT.println("Mode: Servo/Solenoid/Switch Control");
      mode3_init();
      break;
    case 4:
      //SerialBT.println("Mode: Servo/Solenoid/Switch + Encoder Control");
      mode4_init();
      break;
    default:;
      ;
      //SerialBT.println("Unknown Mode");
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
    if (received_data[0] == 1) {
      //SerialBT.printf("%d, %d, %d, %d\n", count[0], count[1], count[2], count[3]);
    }

    msg.data.size = 4;
    msg.data.data[0] = count[0];
    msg.data.data[1] = count[1];
    msg.data.data[2] = count[2];
    msg.data.data[3] = count[3];
    RCCHECK(rcl_publish(&publisher, &msg, NULL));

    vTaskDelay(1);  // ウォッチドッグタイマのリセット(必須)
  }
}

void MD_Output_Task(void *pvParameters) {
  while (1) {
    // 以下メインの処理

    // デバッグ用
    // if (received_data[0] == 1) {
    //   SerialBT.print("Received: ");
    //   for (size_t i = 0; i < received_size; i++) {
    //     SerialBT.print(received_data[i]);
    //     SerialBT.print(", ");
    //   }
    //   SerialBT.println();
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

    vTaskDelay(1);  // ウォッチドッグタイマのリセット(必須)
  }
}

void Servo_Output_Task(void *pvParameters) {
  //for文で書きかえたいところ、、、
  while (1) {
    // サーボ1
    int angle1 = received_data[0];
    if (angle1 < SERVO1_MIN_DEG) angle1 = SERVO1_MIN_DEG;
    if (angle1 > SERVO1_MAX_DEG) angle1 = SERVO1_MAX_DEG;
    int us1 = map(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG, SERVO1_MIN_US, SERVO1_MAX_US);
    int duty1 = (int)(us1 * SERVO_PWM_SCALE);
    ledcWrite(0, duty1);

    // サーボ2
    int angle2 = received_data[1];
    if (angle2 < SERVO2_MIN_DEG) angle2 = SERVO2_MIN_DEG;
    if (angle2 > SERVO2_MAX_DEG) angle2 = SERVO2_MAX_DEG;
    int us2 = map(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG, SERVO2_MIN_US, SERVO2_MAX_US);
    int duty2 = (int)(us2 * SERVO_PWM_SCALE);
    ledcWrite(1, duty2);

    // サーボ3
    int angle3 = received_data[2];
    if (angle3 < SERVO3_MIN_DEG) angle3 = SERVO3_MIN_DEG;
    if (angle3 > SERVO3_MAX_DEG) angle3 = SERVO3_MAX_DEG;
    int us3 = map(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG, SERVO3_MIN_US, SERVO3_MAX_US);
    int duty3 = (int)(us3 * SERVO_PWM_SCALE);
    ledcWrite(2, duty3);

    // サーボ4
    int angle4 = received_data[3];
    if (angle4 < SERVO4_MIN_DEG) angle4 = SERVO4_MIN_DEG;
    if (angle4 > SERVO4_MAX_DEG) angle4 = SERVO4_MAX_DEG;
    int us4 = map(angle4, SERVO4_MIN_DEG, SERVO4_MAX_DEG, SERVO4_MIN_US, SERVO4_MAX_US);
    int duty4 = (int)(us4 * SERVO_PWM_SCALE);
    ledcWrite(3, duty4);

    // サーボ5
    int angle5 = received_data[4];
    if (angle5 < SERVO5_MIN_DEG) angle5 = SERVO5_MIN_DEG;
    if (angle5 > SERVO5_MAX_DEG) angle5 = SERVO5_MAX_DEG;
    int us5 = map(angle5, SERVO5_MIN_DEG, SERVO5_MAX_DEG, SERVO5_MIN_US, SERVO5_MAX_US);
    int duty5 = (int)(us5 * SERVO_PWM_SCALE);
    ledcWrite(4, duty5);

    // サーボ6
    int angle6 = received_data[5];
    if (angle6 < SERVO6_MIN_DEG) angle6 = SERVO6_MIN_DEG;
    if (angle6 > SERVO6_MAX_DEG) angle6 = SERVO6_MAX_DEG;
    int us6 = map(angle6, SERVO6_MIN_DEG, SERVO6_MAX_DEG, SERVO6_MIN_US, SERVO6_MAX_US);
    int duty6 = (int)(us6 * SERVO_PWM_SCALE);
    ledcWrite(5, duty6);

    // サーボ7
    int angle7 = received_data[6];
    if (angle7 < SERVO7_MIN_DEG) angle7 = SERVO7_MIN_DEG;
    if (angle7 > SERVO7_MAX_DEG) angle7 = SERVO7_MAX_DEG;
    int us7 = map(angle7, SERVO7_MIN_DEG, SERVO7_MAX_DEG, SERVO7_MIN_US, SERVO7_MAX_US);
    int duty7 = (int)(us7 * SERVO_PWM_SCALE);
    ledcWrite(6, duty7);

    // サーボ8
    int angle8 = received_data[7];
    if (angle8 < SERVO8_MIN_DEG) angle8 = SERVO8_MIN_DEG;
    if (angle8 > SERVO8_MAX_DEG) angle8 = SERVO8_MAX_DEG;
    int us8 = map(angle8, SERVO8_MIN_DEG, SERVO8_MAX_DEG, SERVO8_MIN_US, SERVO8_MAX_US);
    int duty8 = (int)(us8 * SERVO_PWM_SCALE);
    ledcWrite(7, duty8);

    //vTaskDelay(1);  // ウォッチドッグタイマのリセット(必須)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void IO_Task(void *pvParameters) {
  while (1) {

    digitalWrite(SV1, received_data[18] ? HIGH : LOW);
    digitalWrite(SV2, received_data[19] ? HIGH : LOW);
    digitalWrite(SV3, received_data[20] ? HIGH : LOW);
    digitalWrite(SV4, received_data[21] ? HIGH : LOW);
    digitalWrite(SV5, received_data[22] ? HIGH : LOW);
    digitalWrite(SV6, received_data[23] ? HIGH : LOW);
    digitalWrite(SV7, received_data[24] ? HIGH : LOW);

    //vTaskDelay(1);  // ウォッチドッグタイマのリセット(必須)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void SW_Task(void *pvParameters) {
  while (1) {
    // スイッチの状態を取得
    sw_state[0] = (digitalRead(SW1) == HIGH);
    sw_state[1] = (digitalRead(SW2) == HIGH);
    sw_state[2] = (digitalRead(SW3) == HIGH);
    sw_state[3] = (digitalRead(SW4) == HIGH);


    // // デバッグ用
    // if (received_data[0] == 1) {
    //   SerialBT.printf("%d, %d, %d, %d\n", sw_state[0], sw_state[1], sw_state[2], sw_state[3]);
    // }
    // msg.data.size = 8;
    // msg.data.data[4] = sw_state[0];
    // msg.data.data[5] = sw_state[1];
    // msg.data.data[6] = sw_state[2];
    // msg.data.data[7] = sw_state[3];
    // RCCHECK(rcl_publish(&publisher, &msg, NULL));

    //vTaskDelay(1);  // ウォッチドッグタイマのリセット(必須)
    vTaskDelay(pdMS_TO_TICKS(10));

    //vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void executor_task(void *arg) {
  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// void publisher_task(void *arg) {
//   while (1) {
//     msg.data.size = 8;
//     msg.data.data[4] = sw_state[0];
//     msg.data.data[5] = sw_state[1];
//     msg.data.data[6] = sw_state[2];
//     msg.data.data[7] = sw_state[3];

//     RCCHECK(rcl_publish(&publisher, &msg, NULL));
//     vTaskDelay(pdMS_TO_TICKS(20));
//   }
// }

// Timer callback
void publisher_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (!timer) return;

  // メッセージ作成
  msg.data.size = 8;
  msg.data.data[4] = sw_state[0];
  msg.data.data[5] = sw_state[1];
  msg.data.data[6] = sw_state[2];
  msg.data.data[7] = sw_state[3];

  RCCHECK(rcl_publish(&publisher, &msg, NULL));
}


void loop() {
  vTaskDelay(1000);
}
