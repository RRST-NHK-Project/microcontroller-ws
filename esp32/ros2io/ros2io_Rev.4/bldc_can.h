
/*====================================================================
<bldc_can.h>
・

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include "bldc_can.h"

void BLDC_CAN_init();
void BLDC_CAN_Send_Task(void *pvParameters);
void BLDC_CAN_8bit_Receive_Task(void *pvParameters);
void BLDC_CAN_16bit_Receive_Task(void *pvParameters);
