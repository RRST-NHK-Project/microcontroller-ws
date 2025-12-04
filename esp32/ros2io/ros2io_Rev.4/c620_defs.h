/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include "twai.h"
#include "can_defs.h"
#include "defs.h"
#include "input_task.h"
#include "ros_defs.h"
#include <Arduino.h>
#include <CAN.h>
//  パルスカウンタ関連
#include "driver/pcnt.h"

// ********* CAN関連ここまで ********* //

void C620_Task(void *pvParameters);

void C620_Task_v2(void *pvParameters);

void C620_debug(void *pvParameters);
