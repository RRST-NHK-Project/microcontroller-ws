/*====================================================================
<led_task.h>
・LED状態表示タスクの定義ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void LED_Blink100_Task(void *pvParameters);
void LED_PWM_Task(void *pvParameters);
