#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void enc_init_all();
void enc_init_half();
void ENC_PRI_Read_Publish_Task(void *pvParameters);
void SW_PRI_Read_Publish_Task(void *pvParameters);
