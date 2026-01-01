/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void Input_Task(void *); // 出力タスク
void Input_init();
void ENC_Input();
void SW_Input();