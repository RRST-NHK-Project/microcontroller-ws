/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include <Arduino.h>

void send_cur_all(float cur_array[NUM_MOTOR]);

// 関数のプロトタイプ宣言
void M3508_Task(void *);
void M3508_RX(void *);
