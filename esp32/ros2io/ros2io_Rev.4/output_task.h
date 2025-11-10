/*====================================================================
<output_task.h>
・出力系タスク（ロボマスを除くアクチュエータ類）の定義ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void Output_Task(void *pvParameters);
void LED_Blink100_Task(void *pvParameters);
void LED_PWM_Task(void *pvParameters);
