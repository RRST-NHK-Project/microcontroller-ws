/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void Output_Task(void *); // 出力タスク
void Output_init();
void MD_Output();
void Servo_Output();
void TR_Output();