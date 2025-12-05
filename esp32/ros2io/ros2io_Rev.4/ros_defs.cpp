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
    1,  // 優先度、最大25？
    &led_blink100_handle,
    APP_CPU_NUM);

  // Agentと接続できるまでリトライ
  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    delay(1000);  // 1秒待つ
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

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));  // 以下のサービスの数でexecutorのサイズを変える。

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
    1,  // 優先度、最大25？
    &led_blink100_handle,
    APP_CPU_NUM);

  // Agentと接続できるまでリトライ
  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    delay(1000);  // 1秒待つ
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

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));  // 以下のサービスの数でexecutorのサイズを変える。

  // Executorにサービスを追加
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  vTaskDelete(led_blink100_handle);
  led_blink100_handle = NULL;
}