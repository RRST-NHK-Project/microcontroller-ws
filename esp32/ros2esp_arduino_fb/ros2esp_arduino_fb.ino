#include "driver/pcnt.h"
#include <ESP32Encoder.h>

/*
2025, RRST-NHK-Project
ros2espパッケージ、マイコン側プログラム
microROSで受信したデータをもとにピン操作
ワイヤレスデバッグ(Bluetooth Serial)に対応。スマホアプリもしくはTeratermでデバッグ可能。
４MB版のESPでは容量が不足するためTools/PartitionSchemeからNO OTA(2MB APP/2MB SPIFFS)を選択すること。

TODO:時間経過でスタックするバグの修正
TODO:エンコーダの取得部分の追加

*/

// microROS関連
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32_multi_array.h>

// ワイヤレスデバッグで使う
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#define MAX_ARRAY_SIZE 19

#define COUNTER_H_LIM 32767
#define COUNTER_L_LIM -32768

// ピンの定義 //

#define ENC1_A 18
#define ENC1_B 17
#define ENC2_A 16
#define ENC2_B 2
#define ENC3_A 15
#define ENC3_B 36
// #define ENC5_A 21
// #define ENC5_B 22
// #define ENC6_A 19
// #define ENC6_B 18

#define MD1P 32
#define MD2P 33
#define MD3P 27
#define MD4P 14
#define MD5P 23
#define MD6P 22

// MD DIR
#define MD1D 25
#define MD2D 26
#define MD3D 12
#define MD4D 13
#define MD5D 21
#define MD6D 19

// パルスカウンタの定義
gpio_set_pull_mode(ENC1_A, GPIO_PULLUP_ONLY);
gpio_set_pull_mode(ENC1_B, GPIO_PULLUP_ONLY);
gpio_set_pull_mode(ENC2_A, GPIO_PULLUP_ONLY);
gpio_set_pull_mode(ENC2_B, GPIO_PULLUP_ONLY);
gpio_set_pull_mode(ENC3_A, GPIO_PULLUP_ONLY);
gpio_set_pull_mode(ENC3_B, GPIO_PULLUP_ONLY);
gpio_set_pull_mode(ENC4_A, GPIO_PULLUP_ONLY);
gpio_set_pull_mode(ENC4_B, GPIO_PULLUP_ONLY);

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
pcnt_config2.pulse_gpio_num = PULSE_PIN_DT;
pcnt_config2.ctrl_gpio_num = PULSE_PIN_CLK;
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
pcnt_config4.pulse_gpio_num = ENC2_A;
pcnt_config4.ctrl_gpio_num = ENC2_B;
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
pcnt_config6.pulse_gpio_num = ENC3_A;
pcnt_config6.ctrl_gpio_num = ENC3_B;
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
pcnt_config8.pulse_gpio_num = ENC4_A;
pcnt_config8.ctrl_gpio_num = ENC4_B;
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

rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
int32_t buffer[MAX_ARRAY_SIZE];

// 受信データ格納用（グローバル）
volatile int32_t received_data[MAX_ARRAY_SIZE]; // 最新の受信データ
volatile size_t received_size = 0;              // 受信データのサイズ

#define RCCHECK(fn)             \
    {                           \
        if ((fn) != RCL_RET_OK) \
            error_loop();       \
    }

void error_loop() {
    while (1) {
        SerialBT.println("RCL Error!");
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
    SerialBT.begin("ESP32");
    delay(2000);

    set_microros_transports();

    allocator = rcl_get_default_allocator();

    // 通信状態表示LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    // ★ エージェントと接続できるまでリトライ
    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        SerialBT.println("Waiting for agent...");
        delay(1000); // 1秒待つ
    }

    RCCHECK(rclc_node_init_default(&node, "micro_ros_esp_node_00", "", &support));
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "mr_swerve_drive"));

    std_msgs__msg__Int32MultiArray__init(&msg);
    msg.data.data = buffer;
    msg.data.size = 0;
    msg.data.capacity = MAX_ARRAY_SIZE;

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    // ピンの初期化 //

    // MD DIR
    pinMode(MD1P, OUTPUT);
    pinMode(MD2P, OUTPUT);
    pinMode(MD3P, OUTPUT);
    pinMode(MD4P, OUTPUT);
    pinMode(MD5P, OUTPUT);
    pinMode(MD6P, OUTPUT);

    // MD PWM
    pinMode(MD1D, OUTPUT);
    pinMode(MD2D, OUTPUT);
    pinMode(MD3D, OUTPUT);
    pinMode(MD4D, OUTPUT);
    pinMode(MD5D, OUTPUT);
    pinMode(MD6D, OUTPUT);
}

void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));

    digitalWrite(LED, HIGH);

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

    // ピンの操作

    // ここまで

    // delay(10);
}
