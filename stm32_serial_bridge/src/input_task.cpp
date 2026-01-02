/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "frame_data.hpp"
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <SimpleFOC.h>
#include <defs.hpp>

constexpr uint32_t INPUT_PERIOD_MS = 20; // 更新周期（ミリ秒）

void Input_init();
void ENC_Input();
void SW_Input();

// ================= エンコーダー 定義 =================
// (A, B, CPR)
// Simple FOCのEncoderクラスを転用
Encoder enc1(ENC1_A, ENC1_B, 2048);
Encoder enc2(ENC2_A, ENC2_B, 2048);
Encoder enc3(ENC3_A, ENC3_B, 2048);
Encoder enc4(ENC4_A, ENC4_B, 2048);
Encoder enc5(ENC5_A, ENC5_B, 2048);
Encoder enc6(ENC6_A, ENC6_B, 2048);
Encoder enc7(ENC7_A, ENC7_B, 2048);
Encoder enc8(ENC8_A, ENC8_B, 2048);

// ================= 割り込みハンドラ =================
void enc1A() { enc1.handleA(); }
void enc1B() { enc1.handleB(); }

void enc2A() { enc2.handleA(); }
void enc2B() { enc2.handleB(); }

void enc3A() { enc3.handleA(); }
void enc3B() { enc3.handleB(); }

void enc4A() { enc4.handleA(); }
void enc4B() { enc4.handleB(); }

void enc5A() { enc5.handleA(); }
void enc5B() { enc5.handleB(); }

void enc6A() { enc6.handleA(); }
void enc6B() { enc6.handleB(); }

void enc7A() { enc7.handleA(); }
void enc7B() { enc7.handleB(); }

void enc8A() { enc8.handleA(); }
void enc8B() { enc8.handleB(); }

// ================= TASK =================

void Input_Task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    Input_init();

    while (1) {
        // ENC_Input();
        SW_Input();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(INPUT_PERIOD_MS));
    }
}

// ================= 関数 =================
// マイコンや基板の不具合に対応するためにfor文は使わない

void Input_init() {

    // エンコーダー 初期化、割り込み有効化
    enc1.init();
    enc1.enableInterrupts(enc1A, enc1B);

    enc2.init();
    enc2.enableInterrupts(enc2A, enc2B);

    enc3.init();
    enc3.enableInterrupts(enc3A, enc3B);

    enc4.init();
    enc4.enableInterrupts(enc4A, enc4B);

    enc5.init();
    enc5.enableInterrupts(enc5A, enc5B);

    enc6.init();
    enc6.enableInterrupts(enc6A, enc6B);

    enc7.init();
    enc7.enableInterrupts(enc7A, enc7B);

    enc8.init();
    enc8.enableInterrupts(enc8A, enc8B);

    // SW ピン初期化
    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW3, INPUT_PULLUP);
    pinMode(SW4, INPUT_PULLUP);
    pinMode(SW5, INPUT_PULLUP);
    pinMode(SW6, INPUT_PULLUP);
    pinMode(SW7, INPUT_PULLUP);
    pinMode(SW8, INPUT_PULLUP);
}

void ENC_Input() {
    // ENC入力処理
    taskENTER_CRITICAL();
    Tx_16Data[1] = (int16_t)(enc1.getAngle() * 1000.0f);
    Tx_16Data[2] = (int16_t)(enc2.getAngle() * 1000.0f);
    Tx_16Data[3] = (int16_t)(enc3.getAngle() * 1000.0f);
    Tx_16Data[4] = (int16_t)(enc4.getAngle() * 1000.0f);
    Tx_16Data[5] = (int16_t)(enc5.getAngle() * 1000.0f);
    Tx_16Data[6] = (int16_t)(enc6.getAngle() * 1000.0f);
    Tx_16Data[7] = (int16_t)(enc7.getAngle() * 1000.0f);
    Tx_16Data[8] = (int16_t)(enc8.getAngle() * 1000.0f);
    taskEXIT_CRITICAL();
}

void SW_Input() {
    // SW入力処理
    Tx_16Data[9] = !digitalRead(SW1);
    Tx_16Data[10] = !digitalRead(SW2);
    Tx_16Data[11] = !digitalRead(SW3);
    Tx_16Data[12] = !digitalRead(SW4);
    Tx_16Data[13] = !digitalRead(SW5);
    Tx_16Data[14] = !digitalRead(SW6);
    Tx_16Data[15] = !digitalRead(SW7);
    Tx_16Data[16] = !digitalRead(SW8);
}
