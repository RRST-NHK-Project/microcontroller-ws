/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void serialTask(void *); // 送受信を同時に行うタスク
void txTask(void *);     // 送信タスク
void rxTask(void *);     // 受信タスク
void send_frame();
void receive_frame();