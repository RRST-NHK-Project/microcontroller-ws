/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include "config.h"
#include "defs.h"
#include "output_task.h"

// microROS関連
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32_multi_array.h>

extern rcl_subscription_t subscriber;
extern rcl_publisher_t publisher;
// rcl_timer_t timer;
extern std_msgs__msg__Int32MultiArray msg;
extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;

// ノード名とトピック名の定義（ID付き）
extern String node_name;
extern String publisher_topic_name;
extern String subscriber_topic_name;

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

void error_loop();
void subscription_callback(const void *msgin);
void ros_init();
void ros_wifi_init();
void ros_can_init();
