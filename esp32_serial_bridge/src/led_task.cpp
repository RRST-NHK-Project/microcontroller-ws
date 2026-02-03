/*====================================================================
<led_task.cpp>
・LED状態表示タスクの実装ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include <Arduino.h>
#include <defs.hpp>
#include <led_task.hpp>

void LED_Blink100_Task(void *pvParameters) {
    while (1) {
        digitalWrite(LED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(LED, LOW);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ledcWrite,vTaskDelayに書き換え予定
void LED_PWM_Task(void *pvParameters) {
    while (1) {
        for (int val = 0; val <= 255; val++) {
            ledcWrite(8, val);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        for (int val = 255; val >= 0; val--) {
            ledcWrite(8, val);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}