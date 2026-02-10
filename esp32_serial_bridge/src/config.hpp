/*====================================================================
<config.h>
書き込み前にここでIDと動作モードを設定してください．
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once
#include <Arduino.h>

// IDの設定，ROS側からマイコンを識別するために使用，すべてのマイコンで異なる値にすること
#define DEVICE_ID 0x02

// モードの設定，どれか一つをコメントアウト解除する
// #define MODE_OUTPUT
// #define MODE_INPUT
// #define MODE_IO
#define MODE_ROBOMAS
// #define MODE_DEBUG

// 以下の設定は必要に応じて変更（通常は変更不要）
#define ENABLE_LED 1          // 状態表示LEDを有効にする場合1に設定
#define ENABLE_EXTRA_TR_PIN 0 // TR6,TR7を有効にする場合1に設定、サーボとのピン競合に注意
