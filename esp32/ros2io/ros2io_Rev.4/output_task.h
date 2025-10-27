#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void Output_Task(void *pvParameters);
void LED_Blink100_Task(void *pvParameters);
void LED_PWM_Task(void *pvParameters);
