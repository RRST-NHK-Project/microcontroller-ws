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
void IO_Task(void *);       // 入出力タスク
void PID_Task(void *);      // PID制御タスク
void MD_Output();
void Output_init();
void Servo_Output();
void TR_Output();
void Input_init();
void ENC_Input();
void SW_Input();
void ENCx4_SWx4_init();
void ENCx2_SWx8_init();
void ENCx2_init();
void pid_control();
void IO_MD_Output();
void IO_ENC_Input();
void IO_SW_Input();
void IO_init();
