/*
2025, RRST-NHK-Project
ros2espパッケージ、マイコン側プログラム
microROSで受信したデータをもとにピン操作
ワイヤレスデバッグ(Bluetooth Serial)に対応しています。ROS側からオンオフ切り替えできます。スマホアプリもしくはTeratermでデバッグ可能です。
４MB版のESPでは容量が不足するためTools/PartitionSchemeからNO OTA(2MB APP/2MB SPIFFS)を選択してください。

TODO:時間経過でスタックするバグの修正

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

// ピンの定義 //

// 通信状態表示
#define LED 2

// #define MD1P PA_0

// サーボ
#define SERVO1 32
#define SERVO2 33
#define SERVO3 25
#define SERVO4 26
#define SERVO5 27
#define SERVO6 14
#define SERVO7_SV7 13
#define SERVO8_SV8 12

// MOSFET（ソレノイド・表示灯）
#define SV1 23
#define SV2 22
#define SV3 21
#define SV4 19
#define SV5 18
#define SV6 17

#define MD5P 2
#define MD6P 16

#define MD5D 4
#define MD6D 5

rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
int32_t buffer[MAX_ARRAY_SIZE];

// ★ 受信データ格納用（グローバル）
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

// ★ コールバック内でグローバル変数にコピー
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

    // サーボ
    pinMode(SERVO1, OUTPUT);
    pinMode(SERVO2, OUTPUT);
    pinMode(SERVO3, OUTPUT);
    pinMode(SERVO4, OUTPUT);
    pinMode(SERVO5, OUTPUT);
    pinMode(SERVO6, OUTPUT);
    pinMode(SERVO7_SV7, OUTPUT);
    pinMode(SERVO8_SV8, OUTPUT);

    // トランジスタ(ソレノイド・表示灯)
    pinMode(SV1, OUTPUT);
    pinMode(SV2, OUTPUT);
    pinMode(SV3, OUTPUT);
    pinMode(SV4, OUTPUT);
    pinMode(SV5, OUTPUT);
    pinMode(SV6, OUTPUT);

    pinMode(MD5P, OUTPUT);
    pinMode(MD6P, OUTPUT);
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
