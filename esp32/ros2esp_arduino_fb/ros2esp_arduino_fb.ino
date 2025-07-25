/*
2025, RRST-NHK-Project
ros2espパッケージ、マイコン側プログラム
microROSで受信したデータをもとにピン操作
ワイヤレスデバッグ(Bluetooth Serial)に対応。スマホアプリもしくはTeratermでデバッグ可能。
4MB版のESPでは容量が不足するためTools/PartitionSchemeからNO OTA(2MB APP/2MB SPIFFS)を選択すること。

TODO:時間経過でスタックするバグの修正
*/

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
// **複数のESPを使用する場合はIDを変更** //
#define ID 0
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

// microROS関連
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32_multi_array.h>

// ワイヤレスデバッグ
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

//  パルスカウンタ関連
#include "driver/pcnt.h"

// 受信配列の要素数を事前に定義
#define MAX_ARRAY_SIZE 19

// パルスカウンタの上限・下限の定義
#define COUNTER_H_LIM 32767
#define COUNTER_L_LIM -32768

// MD出力の上限値
#define MD_PWM_MAX 255

// ピンの定義 //
// エンコーダ
#define ENC1_A 18
#define ENC1_B 17
#define ENC2_A 16
#define ENC2_B 2
#define ENC3_A 15
#define ENC3_B 19
#define ENC4_A 22
#define ENC4_B 23
// MD PWM
#define MD1P 32
#define MD2P 33
#define MD3P 27
#define MD4P 14
// MD DIR
#define MD1D 25
#define MD2D 26
#define MD3D 12
#define MD4D 13

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_timer_t timer;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ノード名とトピック名の定義（ID付き）
String node_name = "esp32_micro_ros_node_" + String(ID, DEC);
String publisher_topic_name = "from_esp32" + String(ID, DEC);
String subscriber_topic_name = "to_esp32" + String(ID, DEC);

// 受信データ格納用のバッファ
int32_t buffer[MAX_ARRAY_SIZE];

// 受信データ格納用
volatile int32_t received_data[MAX_ARRAY_SIZE]; // 受信データ
volatile size_t received_size = 0;              // 受信データのサイズ

// エンコーダのカウント格納用
int16_t count[4] = {0};

#define RCCHECK(fn)             \
    {                           \
        if ((fn) != RCL_RET_OK) \
            error_loop();       \
    }

// エラー発生時のループ
void error_loop() {
    while (1) {
        // このエラーが表示される場合は、シリアルモニターを閉じているか確認
        SerialBT.println("RCL Error!");
        delay(1000);
    }
}

// Publisherのコールバック関数
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        msg.data.size = 4;
        msg.data.data[0] = count[0];
        msg.data.data[1] = count[1];
        msg.data.data[2] = count[2];
        msg.data.data[3] = count[3];
        RCCHECK(rcl_publish(&publisher, &msg, NULL));
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
    SerialBT.begin("ESP32" + String(ID, DEC)); // Bluetoothの初期化
    delay(2000);

    // パルスカウンタの定義
    // プルアップを有効化
    gpio_set_pull_mode((gpio_num_t)ENC1_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC1_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC2_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC2_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC3_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC3_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC4_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC4_B, GPIO_PULLUP_ONLY);

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

    pcnt_config_t pcnt_config2 = {};
    pcnt_config2.pulse_gpio_num = ENC1_B;
    pcnt_config2.ctrl_gpio_num = ENC1_A;
    pcnt_config2.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config2.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config2.pos_mode = PCNT_COUNT_INC;
    pcnt_config2.neg_mode = PCNT_COUNT_DEC;
    pcnt_config2.counter_h_lim = COUNTER_H_LIM;
    pcnt_config2.counter_l_lim = COUNTER_L_LIM;
    pcnt_config2.unit = PCNT_UNIT_0;
    pcnt_config2.channel = PCNT_CHANNEL_1;

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

    pcnt_config_t pcnt_config4 = {};
    pcnt_config4.pulse_gpio_num = ENC2_B;
    pcnt_config4.ctrl_gpio_num = ENC2_A;
    pcnt_config4.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config4.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config4.pos_mode = PCNT_COUNT_INC;
    pcnt_config4.neg_mode = PCNT_COUNT_DEC;
    pcnt_config4.counter_h_lim = COUNTER_H_LIM;
    pcnt_config4.counter_l_lim = COUNTER_L_LIM;
    pcnt_config4.unit = PCNT_UNIT_1;
    pcnt_config4.channel = PCNT_CHANNEL_1;

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

    pcnt_config_t pcnt_config6 = {};
    pcnt_config6.pulse_gpio_num = ENC3_B;
    pcnt_config6.ctrl_gpio_num = ENC3_A;
    pcnt_config6.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config6.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config6.pos_mode = PCNT_COUNT_INC;
    pcnt_config6.neg_mode = PCNT_COUNT_DEC;
    pcnt_config6.counter_h_lim = COUNTER_H_LIM;
    pcnt_config6.counter_l_lim = COUNTER_L_LIM;
    pcnt_config6.unit = PCNT_UNIT_2;
    pcnt_config6.channel = PCNT_CHANNEL_1;

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

    pcnt_config_t pcnt_config8 = {};
    pcnt_config8.pulse_gpio_num = ENC4_B;
    pcnt_config8.ctrl_gpio_num = ENC4_A;
    pcnt_config8.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config8.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config8.pos_mode = PCNT_COUNT_INC;
    pcnt_config8.neg_mode = PCNT_COUNT_DEC;
    pcnt_config8.counter_h_lim = COUNTER_H_LIM;
    pcnt_config8.counter_l_lim = COUNTER_L_LIM;
    pcnt_config8.unit = PCNT_UNIT_3;
    pcnt_config8.channel = PCNT_CHANNEL_1;

    // パルスカウンタの初期化
    pcnt_unit_config(&pcnt_config1);
    pcnt_unit_config(&pcnt_config2);
    pcnt_unit_config(&pcnt_config3);
    pcnt_unit_config(&pcnt_config4);
    pcnt_unit_config(&pcnt_config5);
    pcnt_unit_config(&pcnt_config6);
    pcnt_unit_config(&pcnt_config7);
    pcnt_unit_config(&pcnt_config8);

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

    set_microros_transports();
    allocator = rcl_get_default_allocator();

    // Agentと接続できるまでリトライ
    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        SerialBT.println("Waiting for agent...");
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

    // タイマーの初期化
    const unsigned int timer_timeout = 10; // 10msごとにコールバックを呼び出す
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    std_msgs__msg__Int32MultiArray__init(&msg);
    msg.data.data = buffer;
    msg.data.size = 0;
    msg.data.capacity = MAX_ARRAY_SIZE;

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // ピンの初期化 //
    // MD DIR
    pinMode(MD1P, OUTPUT);
    pinMode(MD2P, OUTPUT);
    pinMode(MD3P, OUTPUT);
    pinMode(MD4P, OUTPUT);
    // MD PWM
    pinMode(MD1D, OUTPUT);
    pinMode(MD2D, OUTPUT);
    pinMode(MD3D, OUTPUT);
    pinMode(MD4D, OUTPUT);

    // エンコーダ取得のスレッド（タスク）の作成
    xTaskCreateUniversal(
        ENC_Read_Task,
        "ENC_Read_Task",
        8192,
        NULL,
        1,
        NULL,
        APP_CPU_NUM);
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
            SerialBT.printf("%d, %d, %d, %d\n", count[0], count[1], count[2], count[3]);
        }
        // delay(10);
        delay(1); // ウォッチドッグタイマのリセット(必須)
    }
}

void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));

    // 以下メインの処理

    // デバッグ用
    if (received_data[0] == 1) {
        SerialBT.print("Received: ");
        for (size_t i = 0; i < received_size; i++) {
            SerialBT.print(received_data[i]);
            SerialBT.print(", ");
        }
        SerialBT.println();
    }

    // MD出力の制限
    received_data[1] = constrain(received_data[1], -MD_PWM_MAX, MD_PWM_MAX);
    received_data[2] = constrain(received_data[2], -MD_PWM_MAX, MD_PWM_MAX);
    received_data[3] = constrain(received_data[3], -MD_PWM_MAX, MD_PWM_MAX);
    received_data[4] = constrain(received_data[4], -MD_PWM_MAX, MD_PWM_MAX);

    // ピンの操作
    digitalWrite(MD1D, received_data[1] > 0 ? HIGH : LOW);
    digitalWrite(MD2D, received_data[2] > 0 ? HIGH : LOW);
    digitalWrite(MD3D, received_data[3] > 0 ? HIGH : LOW);
    digitalWrite(MD4D, received_data[4] > 0 ? HIGH : LOW);

    analogWrite(MD1P, abs(received_data[1]));
    analogWrite(MD2P, abs(received_data[2]));
    analogWrite(MD3P, abs(received_data[3]));
    analogWrite(MD4P, abs(received_data[4]));

    // delay(10);
}
