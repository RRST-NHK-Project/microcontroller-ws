/*====================================================================
<input_task.h>
・入力系タスク（センサ類）の定義ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void enc_init_all();
void enc_init_half();
void ENC_PRI_Read_Publish_Task(void *pvParameters);
void SW_PRI_Read_Publish_Task(void *pvParameters);
