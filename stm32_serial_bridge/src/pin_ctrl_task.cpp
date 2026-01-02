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

constexpr uint32_t CTRL_PERIOD_MS = 20; // ピン更新周期（ミリ秒）

void MD_Output();
void Output_init();
void Servo_Output();
void TR_Output();
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

// 入力・出力統合タスク（現状動かない）
void Pin_Ctrl_Task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    Output_init();
    Input_init();

    while (1) {
        MD_Output();
        Servo_Output();
        TR_Output();
        ENC_Input();
        SW_Input();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

void Output_Task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    Output_init();

    while (1) {
        MD_Output();
        // Servo_Output();
        TR_Output();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

void Input_Task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    Input_init();

    while (1) {
        ENC_Input();
        SW_Input();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

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
    // taskENTER_CRITICAL();
    Tx_16Data[1] = (int16_t)(enc1.getAngle() * 1000.0f);
    Tx_16Data[2] = (int16_t)(enc2.getAngle() * 1000.0f);
    Tx_16Data[3] = (int16_t)(enc3.getAngle() * 1000.0f);
    Tx_16Data[4] = (int16_t)(enc4.getAngle() * 1000.0f);
    Tx_16Data[5] = (int16_t)(enc5.getAngle() * 1000.0f);
    Tx_16Data[6] = (int16_t)(enc6.getAngle() * 1000.0f);
    Tx_16Data[7] = (int16_t)(enc7.getAngle() * 1000.0f);
    Tx_16Data[8] = (int16_t)(enc8.getAngle() * 1000.0f);
    // taskEXIT_CRITICAL();
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

// ================= 関数 =================
// マイコンや基板の不具合に対応するためにfor文は使っていない

void Output_init() {

    // MD DIR ピン初期化
    pinMode(MD1D, OUTPUT);
    pinMode(MD2D, OUTPUT);
    pinMode(MD3D, OUTPUT);
    pinMode(MD4D, OUTPUT);
    pinMode(MD5D, OUTPUT);
    pinMode(MD6D, OUTPUT);
    pinMode(MD7D, OUTPUT);
    pinMode(MD8D, OUTPUT);

    // TR ピン初期化
    pinMode(TR1, OUTPUT);
    pinMode(TR2, OUTPUT);
    pinMode(TR3, OUTPUT);
    pinMode(TR4, OUTPUT);
    pinMode(TR5, OUTPUT);
    pinMode(TR6, OUTPUT);
    pinMode(TR7, OUTPUT);
}

void MD_Output() {

    // MD出力の制限
    Rx_16Data[1] = constrain(Rx_16Data[1], -MD_PWM_MAX, MD_PWM_MAX);
    Rx_16Data[2] = constrain(Rx_16Data[2], -MD_PWM_MAX, MD_PWM_MAX);
    Rx_16Data[3] = constrain(Rx_16Data[3], -MD_PWM_MAX, MD_PWM_MAX);
    Rx_16Data[4] = constrain(Rx_16Data[4], -MD_PWM_MAX, MD_PWM_MAX);
    Rx_16Data[5] = constrain(Rx_16Data[5], -MD_PWM_MAX, MD_PWM_MAX);
    Rx_16Data[6] = constrain(Rx_16Data[6], -MD_PWM_MAX, MD_PWM_MAX);
    Rx_16Data[7] = constrain(Rx_16Data[7], -MD_PWM_MAX, MD_PWM_MAX);
    Rx_16Data[8] = constrain(Rx_16Data[8], -MD_PWM_MAX, MD_PWM_MAX);

    // 通信状態の表示を実装予定
    digitalWrite(F446RE_BUILTIN_LED, Rx_16Data[1] > 0 ? HIGH : LOW);
    digitalWrite(F446RE_BUILTIN_LED, HIGH);

    // // ピンの操作
    digitalWrite(MD1D, Rx_16Data[1] > 0 ? HIGH : LOW);
    digitalWrite(MD2D, Rx_16Data[2] > 0 ? HIGH : LOW);
    digitalWrite(MD3D, Rx_16Data[3] > 0 ? HIGH : LOW);
    digitalWrite(MD4D, Rx_16Data[4] > 0 ? HIGH : LOW);
    digitalWrite(MD5D, Rx_16Data[5] > 0 ? HIGH : LOW);
    digitalWrite(MD6D, Rx_16Data[6] > 0 ? HIGH : LOW);
    digitalWrite(MD7D, Rx_16Data[7] > 0 ? HIGH : LOW);
    digitalWrite(MD8D, Rx_16Data[8] > 0 ? HIGH : LOW);

    // PWM周波数、分解能の切り替え
    // analogWriteFrequency(MD_PWM_FREQ);
    // analogWriteResolution(MD_PWM_RESOLUTION);

    analogWrite(MD1P, abs(Rx_16Data[1]));
    analogWrite(MD2P, abs(Rx_16Data[2]));
    analogWrite(MD3P, abs(Rx_16Data[3]));
    analogWrite(MD4P, abs(Rx_16Data[4]));
    analogWrite(MD5P, abs(Rx_16Data[5]));
    analogWrite(MD6P, abs(Rx_16Data[6]));
    analogWrite(MD7P, abs(Rx_16Data[7]));
    analogWrite(MD8P, abs(Rx_16Data[8]));
}

void Servo_Output() {

    // PWM周波数、分解能の切り替え
    analogWriteFrequency(SERVO_PWM_FREQ);
    analogWriteResolution(SERVO_PWM_RESOLUTION);

    // サーボ1
    int angle1 = Rx_16Data[9];
    if (angle1 < SERVO1_MIN_DEG)
        angle1 = SERVO1_MIN_DEG;
    if (angle1 > SERVO1_MAX_DEG)
        angle1 = SERVO1_MAX_DEG;
    int us1 = map(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG, SERVO1_MIN_US, SERVO1_MAX_US);
    int duty1 = (int)(us1 * SERVO_PWM_SCALE);
    analogWrite(SERVO1, duty1);

    // サーボ2
    int angle2 = Rx_16Data[10];
    if (angle2 < SERVO2_MIN_DEG)
        angle2 = SERVO2_MIN_DEG;
    if (angle2 > SERVO2_MAX_DEG)
        angle2 = SERVO2_MAX_DEG;
    int us2 = map(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG, SERVO2_MIN_US, SERVO2_MAX_US);
    int duty2 = (int)(us2 * SERVO_PWM_SCALE);
    analogWrite(SERVO2, duty2);

    // サーボ3
    int angle3 = Rx_16Data[11];
    if (angle3 < SERVO3_MIN_DEG)
        angle3 = SERVO3_MIN_DEG;
    if (angle3 > SERVO3_MAX_DEG)
        angle3 = SERVO3_MAX_DEG;
    int us3 = map(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG, SERVO3_MIN_US, SERVO3_MAX_US);
    int duty3 = (int)(us3 * SERVO_PWM_SCALE);
    analogWrite(SERVO3, duty3);

    // サーボ4
    int angle4 = Rx_16Data[12];
    if (angle4 < SERVO4_MIN_DEG)
        angle4 = SERVO4_MIN_DEG;
    if (angle4 > SERVO4_MAX_DEG)
        angle4 = SERVO4_MAX_DEG;
    int us4 = map(angle4, SERVO4_MIN_DEG, SERVO4_MAX_DEG, SERVO4_MIN_US, SERVO4_MAX_US);
    int duty4 = (int)(us4 * SERVO_PWM_SCALE);
    analogWrite(SERVO4, duty4);

    // サーボ5
    int angle5 = Rx_16Data[13];
    if (angle5 < SERVO5_MIN_DEG)
        angle5 = SERVO5_MIN_DEG;
    if (angle5 > SERVO5_MAX_DEG)
        angle5 = SERVO5_MAX_DEG;
    int us5 = map(angle5, SERVO5_MIN_DEG, SERVO5_MAX_DEG, SERVO5_MIN_US, SERVO5_MAX_US);
    int duty5 = (int)(us5 * SERVO_PWM_SCALE);
    analogWrite(SERVO5, duty5);

    // サーボ6
    int angle6 = Rx_16Data[14];
    if (angle6 < SERVO6_MIN_DEG)
        angle6 = SERVO6_MIN_DEG;
    if (angle6 > SERVO6_MAX_DEG)
        angle6 = SERVO6_MAX_DEG;
    int us6 = map(angle6, SERVO6_MIN_DEG, SERVO6_MAX_DEG, SERVO6_MIN_US, SERVO6_MAX_US);
    int duty6 = (int)(us6 * SERVO_PWM_SCALE);
    analogWrite(SERVO6, duty6);

    // サーボ7
    int angle7 = Rx_16Data[15];
    if (angle7 < SERVO7_MIN_DEG)
        angle7 = SERVO7_MIN_DEG;
    if (angle7 > SERVO7_MAX_DEG)
        angle7 = SERVO7_MAX_DEG;
    int us7 = map(angle7, SERVO7_MIN_DEG, SERVO7_MAX_DEG, SERVO7_MIN_US, SERVO7_MAX_US);
    int duty7 = (int)(us7 * SERVO_PWM_SCALE);
    analogWrite(SERVO7, duty7);

    // サーボ8
    int angle8 = Rx_16Data[16];
    if (angle8 < SERVO8_MIN_DEG)
        angle8 = SERVO8_MIN_DEG;
    if (angle8 > SERVO8_MAX_DEG)
        angle8 = SERVO8_MAX_DEG;
    int us8 = map(angle8, SERVO8_MIN_DEG, SERVO8_MAX_DEG, SERVO8_MIN_US, SERVO8_MAX_US);
    int duty8 = (int)(us8 * SERVO_PWM_SCALE);
    analogWrite(SERVO8, duty8);
}

void TR_Output() {
    digitalWrite(TR1, Rx_16Data[17] ? HIGH : LOW);
    digitalWrite(TR2, Rx_16Data[18] ? HIGH : LOW);
    digitalWrite(TR3, Rx_16Data[19] ? HIGH : LOW);
    digitalWrite(TR4, Rx_16Data[20] ? HIGH : LOW);
    digitalWrite(TR5, Rx_16Data[21] ? HIGH : LOW);
    digitalWrite(TR6, Rx_16Data[22] ? HIGH : LOW);
    digitalWrite(TR7, Rx_16Data[23] ? HIGH : LOW);
}
