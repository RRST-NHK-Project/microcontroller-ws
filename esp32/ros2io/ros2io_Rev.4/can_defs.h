/*====================================================================
<can_defs.h>
・CAN関連の変数や関数の定義ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include "defs.h"
#include <Arduino.h>
#include <CAN.h>
//  パルスカウンタ関連
#include "driver/pcnt.h"

// ********* CAN関連 ********* //
// 複数モータ対応CAN送信関数
void send_cur_all(float cur_array[NUM_MOTOR]);

// ********* CAN関連ここまで ********* //

void ROBOMAS_ENC_SW_Read_Publish_Task(void *pvParameters);

void robomas_can_receive(void *pvParameters);

void C610_FB_Task(void *pvParameters);

void CR25_Task(void *pvParameters);

void CAN_Pb(void *pvParameters);

void C620_Task(void *pvParameters);

void C620_Task_v2(void *pvParameters);

void C620_debug(void *pvParameters);

void SV_Task(void *pvParameters);
