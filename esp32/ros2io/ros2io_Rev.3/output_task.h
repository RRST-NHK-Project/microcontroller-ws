#pragma once

#include <Arduino.h>

// 関数のプロトタイプ宣言
void MD_Output_Task(void *pvParameters);
void Servo_Output_Task(void *pvParameters);
void SV_Task(void *pvParameters);
void LED_Blink100_Task(void *pvParameters);
void LED_PWM_Task(void *pvParameters);
