/*====================================================================
<ros_defs.cpp>
・microROS関連の変数や関数の実装ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "ros_defs.h"

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
// rcl_timer_t timer;
std_msgs__msg__Int16MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ノード名とトピック名の定義（ID付き）
String node_name = "esp32node_" + String(ID, DEC);
String publisher_topic_name = "from_esp32_" + String(ID, DEC);
String subscriber_topic_name = "to_esp32_" + String(ID, DEC);

// エラー発生時のループ
void error_loop() {
    while (1) {
        // このエラーが表示される場合は、シリアルモニターを閉じているか確認
        delay(1000);
    }
}

// コールバック内でグローバル変数にコピー
void subscription_callback(const void *msgin) {
    const std_msgs__msg__Int16MultiArray *msg = (const std_msgs__msg__Int16MultiArray *)msgin;
    size_t len = msg->data.size;
    if (len > MAX_ARRAY_SIZE)
        len = MAX_ARRAY_SIZE;

    for (size_t i = 0; i < len; i++) {
        received_data[i] = msg->data.data[i];
    }
    received_size = len;
}

// micro-ROSの初期化
void ros_init() {
    delay(2000);

    set_microros_transports();
    allocator = rcl_get_default_allocator();

    xTaskCreateUniversal(
        LED_Blink100_Task,
        "LED_Blink100_Task",
        2048,
        NULL,
        1, // 優先度、最大25？
        &led_blink100_handle,
        APP_CPU_NUM);

    // Agentと接続できるまでリトライ
    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        delay(1000); // 1秒待つ
    }

    // Nodeの初期化
    RCCHECK(rclc_node_init_default(&node, node_name.c_str(), "", &support));

    // Subscriberの初期化
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        subscriber_topic_name.c_str()));

    // Publisherの初期化
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        publisher_topic_name.c_str()));

    std_msgs__msg__Int16MultiArray__init(&msg);
    msg.data.data = buffer;
    msg.data.size = 0;
    msg.data.capacity = MAX_ARRAY_SIZE;

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // 以下のサービスの数でexecutorのサイズを変える。

    // Executorにサービスを追加
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    vTaskDelete(led_blink100_handle);
    led_blink100_handle = NULL;
}

// micro-ROSの初期化（Wi-Fi）
void ros_wifi_init() {
    delay(2000);

    set_microros_wifi_transports("lnx-dev", "lnxmaster", "10.133.142.170", 8888);
    allocator = rcl_get_default_allocator();

    xTaskCreateUniversal(
        LED_Blink100_Task,
        "LED_Blink100_Task",
        2048,
        NULL,
        1, // 優先度、最大25？
        &led_blink100_handle,
        APP_CPU_NUM);

    // Agentと接続できるまでリトライ
    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        delay(1000); // 1秒待つ
    }

    // Nodeの初期化
    RCCHECK(rclc_node_init_default(&node, node_name.c_str(), "", &support));

    // Subscriberの初期化
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        subscriber_topic_name.c_str()));

    // Publisherの初期化
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        publisher_topic_name.c_str()));

    std_msgs__msg__Int16MultiArray__init(&msg);
    msg.data.data = buffer;
    msg.data.size = 0;
    msg.data.capacity = MAX_ARRAY_SIZE;

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // 以下のサービスの数でexecutorのサイズを変える。

    // Executorにサービスを追加
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    vTaskDelete(led_blink100_handle);
    led_blink100_handle = NULL;
}

// /* ros_defs.cpp — 修正版 */
// #include "ros_defs.h"
// #include "esp_attr.h"
// #include <freertos/FreeRTOS.h>
// #include <freertos/semphr.h>
// #include <freertos/task.h>
// #include <stdio.h> // sprintf

// // micro-ROS / rcl 宣言（既存）
// rcl_subscription_t subscriber;
// rcl_publisher_t publisher;
// std_msgs__msg__Int16MultiArray msg;
// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;

// // ----- ノード名・トピック名（C文字列で保持） -----
// // Arduino String を直接渡すとポインタ寿命で不具合が出るため固定配列にする
// static char node_name_c[48];
// static char publisher_topic_c[48];
// static char subscriber_topic_c[48];

// // タスクハンドル
// static TaskHandle_t microros_executor_handle = NULL;

// // 共有バッファ用のスピンロック（ESP32向け）
// static portMUX_TYPE received_data_mux = portMUX_INITIALIZER_UNLOCKED;

// // エラー発生時のループ
// void error_loop() {
//     while (1) {
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

// // コールバック：共有バッファへ安全にコピー（短く）
// void subscription_callback(const void *msgin) {
//     const std_msgs__msg__Int16MultiArray *incoming = (const std_msgs__msg__Int16MultiArray *)msgin;
//     size_t len = incoming->data.size;
//     if (len > MAX_ARRAY_SIZE)
//         len = MAX_ARRAY_SIZE;

//     portENTER_CRITICAL(&received_data_mux);
//     for (size_t i = 0; i < len; i++) {
//         received_data[i] = incoming->data.data[i];
//     }
//     received_size = len;
//     portEXIT_CRITICAL(&received_data_mux);
// }

// // executor を回す専用タスク（**これを1つだけ作る**）
// void microros_executor_task(void *arg) {
//     const uint32_t SPIN_MS = 5;
//     while (1) {
//         // 非ブロッキングで短時間だけ処理
//         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SPIN_MS));
//         // 他タスクに譲る
//         vTaskDelay(pdMS_TO_TICKS(1));
//     }
// }

// // 共通の初期化（内部で node/topic 名を作る）
// static void ros_common_init(const char *transport_name) {
//     // node名とtopic名を安全なC文字列で作る（IDはグローバルで定義済み想定）
//     snprintf(node_name_c, sizeof(node_name_c), "esp32node_%d", ID);
//     snprintf(publisher_topic_c, sizeof(publisher_topic_c), "from_esp32_%d", ID);
//     snprintf(subscriber_topic_c, sizeof(subscriber_topic_c), "to_esp32_%d", ID);

//     allocator = rcl_get_default_allocator();

//     // LEDブリンクなどを一時的に起動しているならそのまま（既存）
//     xTaskCreateUniversal(
//         LED_Blink100_Task,
//         "LED_Blink100_Task",
//         2048,
//         NULL,
//         1,
//         &led_blink100_handle,
//         APP_CPU_NUM);

//     // Agentと接続できるまでリトライ
//     while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }

//     // Nodeの初期化（安全なC文字列を渡す）
//     RCCHECK(rclc_node_init_default(&node, node_name_c, "", &support));

//     // Subscriberの初期化
//     RCCHECK(rclc_subscription_init_default(
//         &subscriber,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
//         subscriber_topic_c));

//     // Publisherの初期化
//     RCCHECK(rclc_publisher_init_default(
//         &publisher,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
//         publisher_topic_c));

//     // message バッファ設定（buffer はグローバル・静的であること）
//     std_msgs__msg__Int16MultiArray__init(&msg);
//     msg.data.data = buffer;
//     msg.data.size = 0;
//     msg.data.capacity = MAX_ARRAY_SIZE;

//     // Executor 初期化（subscription数に合わせる）
//     RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
//     RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

//     // LEDタスク停止（既存処理）
//     if (led_blink100_handle != NULL) {
//         vTaskDelay(pdMS_TO_TICKS(50)); // 少し待ってから削除
//         vTaskDelete(led_blink100_handle);
//         led_blink100_handle = NULL;
//     }

//     // executor用タスクを1つだけ作成（core0に固定、スタック大きめ）
//     xTaskCreateUniversal(
//         microros_executor_task,
//         "microros_executor",
//         8192, // stack bytes
//         NULL,
//         6, // priority (CANタスクより高めに)
//         &microros_executor_handle,
//         0 // core 0
//     );
// }

// // micro-ROSの初期化（シリアル/USBトランスポート用）
// void ros_init() {
//     delay(2000);
//     set_microros_transports(); // シリアル版 transport をセット（既存）
//     ros_common_init("serial");
// }

// // micro-ROSの初期化（Wi-Fi）
// void ros_wifi_init() {
//     delay(2000);
//     // 例: set_microros_wifi_transports("SSID","PASS","AGENT_IP", PORT);
//     set_microros_wifi_transports("lnx-dev", "lnxmaster", "10.133.142.170", 8888);
//     ros_common_init("wifi");
// }
