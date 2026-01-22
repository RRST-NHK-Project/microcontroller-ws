/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void Pin_Ctrl_Task(void *); // 入出力タスク
void Input_Task(void *);    // 入力タスク
void Output_Task(void *);   // 出力タスク
void MD_Output();
void Output_init();
void Servo_Output();
void TR_Output();
void Input_init();
void ENC_Input();
void SW_Input();
