/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "driver/pcnt.h"
#include "frame_data.hpp"
#include <Arduino.h>
#include <defs.hpp>

constexpr uint32_t CTRL_PERIOD_MS = 20; // ピン更新周期（ミリ秒）

void MD_Output();
void Output_init();
void Servo_Output();
void TR_Output();
void Input_init();
void ENC_Input();
void SW_Input();
void enc_init_all();
void enc_init_half();

// ================= エンコーダー 定義 =================

// ================= 割り込みハンドラ =================

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
        Servo_Output();
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
    enc_init_all();

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
    pcnt_get_counter_value(PCNT_UNIT_0, (int16_t *)&Tx_16Data[1]);
    pcnt_get_counter_value(PCNT_UNIT_1, (int16_t *)&Tx_16Data[2]);
    pcnt_get_counter_value(PCNT_UNIT_2, (int16_t *)&Tx_16Data[3]);
    pcnt_get_counter_value(PCNT_UNIT_3, (int16_t *)&Tx_16Data[4]);
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

    // MDの方向ピンを出力に設定
    pinMode(MD1D, OUTPUT);
    pinMode(MD2D, OUTPUT);
    pinMode(MD3D, OUTPUT);
    pinMode(MD4D, OUTPUT);

    // PWMの初期化
    ledcSetup(0, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcSetup(1, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcSetup(2, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcSetup(3, MD_PWM_FREQ, MD_PWM_RESOLUTION);

    ledcAttachPin(MD1P, 0);
    ledcAttachPin(MD2P, 1);
    ledcAttachPin(MD3P, 2);
    ledcAttachPin(MD4P, 3);

    // サーボのPWMの初期化
    ledcSetup(4, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcSetup(5, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcSetup(6, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcSetup(7, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);

    ledcAttachPin(SERVO1, 4);
    ledcAttachPin(SERVO2, 5);
    ledcAttachPin(SERVO3, 6);
    ledcAttachPin(SERVO4, 7);

    // トランジスタのピンを出力に設定
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

    // // ピンの操作
    digitalWrite(MD1D, Rx_16Data[1] > 0 ? HIGH : LOW);
    digitalWrite(MD2D, Rx_16Data[2] > 0 ? HIGH : LOW);
    digitalWrite(MD3D, Rx_16Data[3] > 0 ? HIGH : LOW);
    digitalWrite(MD4D, Rx_16Data[4] > 0 ? HIGH : LOW);

    // PWM周波数、分解能の切り替え
    // analogWriteFrequency(MD_PWM_FREQ);
    // analogWriteResolution(MD_PWM_RESOLUTION);

    ledcWrite(MD1P, abs(Rx_16Data[1]));
    ledcWrite(MD2P, abs(Rx_16Data[2]));
    ledcWrite(MD3P, abs(Rx_16Data[3]));
    ledcWrite(MD4P, abs(Rx_16Data[4]));
}

void Servo_Output() {

    // サーボ1
    int angle1 = Rx_16Data[9];
    if (angle1 < SERVO1_MIN_DEG)
        angle1 = SERVO1_MIN_DEG;
    if (angle1 > SERVO1_MAX_DEG)
        angle1 = SERVO1_MAX_DEG;
    int us1 = map(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG, SERVO1_MIN_US, SERVO1_MAX_US);
    int duty1 = (int)(us1 * SERVO_PWM_SCALE);
    ledcWrite(SERVO1, duty1);

    // サーボ2
    int angle2 = Rx_16Data[10];
    if (angle2 < SERVO2_MIN_DEG)
        angle2 = SERVO2_MIN_DEG;
    if (angle2 > SERVO2_MAX_DEG)
        angle2 = SERVO2_MAX_DEG;
    int us2 = map(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG, SERVO2_MIN_US, SERVO2_MAX_US);
    int duty2 = (int)(us2 * SERVO_PWM_SCALE);
    ledcWrite(SERVO2, duty2);

    // サーボ3
    int angle3 = Rx_16Data[11];
    if (angle3 < SERVO3_MIN_DEG)
        angle3 = SERVO3_MIN_DEG;
    if (angle3 > SERVO3_MAX_DEG)
        angle3 = SERVO3_MAX_DEG;
    int us3 = map(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG, SERVO3_MIN_US, SERVO3_MAX_US);
    int duty3 = (int)(us3 * SERVO_PWM_SCALE);
    ledcWrite(SERVO3, duty3);

    // サーボ4
    int angle4 = Rx_16Data[12];
    if (angle4 < SERVO4_MIN_DEG)
        angle4 = SERVO4_MIN_DEG;
    if (angle4 > SERVO4_MAX_DEG)
        angle4 = SERVO4_MAX_DEG;
    int us4 = map(angle4, SERVO4_MIN_DEG, SERVO4_MAX_DEG, SERVO4_MIN_US, SERVO4_MAX_US);
    int duty4 = (int)(us4 * SERVO_PWM_SCALE);
    ledcWrite(SERVO4, duty4);
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

void enc_init_all() {
    // プルアップを有効化
    gpio_set_pull_mode((gpio_num_t)ENC1_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC1_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC2_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC2_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC3_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC3_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC4_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC4_B, GPIO_PULLUP_ONLY);

    // パルスカウンタの設定
    pcnt_config_t pcnt_config1 = {};
    pcnt_config1.pulse_gpio_num = ENC1_A;
    pcnt_config1.ctrl_gpio_num = ENC1_B;
    pcnt_config1.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config1.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config1.pos_mode = PCNT_COUNT_INC;
    pcnt_config1.neg_mode = PCNT_COUNT_DEC;
    pcnt_config1.counter_h_lim = COUNTER_H_LIM;
    pcnt_config1.counter_l_lim = COUNTER_L_LIM;
    pcnt_config1.unit = PCNT_UNIT_0;
    pcnt_config1.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config2 = {};
    pcnt_config2.pulse_gpio_num = ENC1_B;
    pcnt_config2.ctrl_gpio_num = ENC1_A;
    pcnt_config2.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config2.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config2.pos_mode = PCNT_COUNT_INC;
    pcnt_config2.neg_mode = PCNT_COUNT_DEC;
    pcnt_config2.counter_h_lim = COUNTER_H_LIM;
    pcnt_config2.counter_l_lim = COUNTER_L_LIM;
    pcnt_config2.unit = PCNT_UNIT_0;
    pcnt_config2.channel = PCNT_CHANNEL_1;

    pcnt_config_t pcnt_config3 = {};
    pcnt_config3.pulse_gpio_num = ENC2_A;
    pcnt_config3.ctrl_gpio_num = ENC2_B;
    pcnt_config3.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config3.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config3.pos_mode = PCNT_COUNT_INC;
    pcnt_config3.neg_mode = PCNT_COUNT_DEC;
    pcnt_config3.counter_h_lim = COUNTER_H_LIM;
    pcnt_config3.counter_l_lim = COUNTER_L_LIM;
    pcnt_config3.unit = PCNT_UNIT_1;
    pcnt_config3.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config4 = {};
    pcnt_config4.pulse_gpio_num = ENC2_B;
    pcnt_config4.ctrl_gpio_num = ENC2_A;
    pcnt_config4.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config4.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config4.pos_mode = PCNT_COUNT_INC;
    pcnt_config4.neg_mode = PCNT_COUNT_DEC;
    pcnt_config4.counter_h_lim = COUNTER_H_LIM;
    pcnt_config4.counter_l_lim = COUNTER_L_LIM;
    pcnt_config4.unit = PCNT_UNIT_1;
    pcnt_config4.channel = PCNT_CHANNEL_1;

    pcnt_config_t pcnt_config5 = {};
    pcnt_config5.pulse_gpio_num = ENC3_A;
    pcnt_config5.ctrl_gpio_num = ENC3_B;
    pcnt_config5.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config5.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config5.pos_mode = PCNT_COUNT_INC;
    pcnt_config5.neg_mode = PCNT_COUNT_DEC;
    pcnt_config5.counter_h_lim = COUNTER_H_LIM;
    pcnt_config5.counter_l_lim = COUNTER_L_LIM;
    pcnt_config5.unit = PCNT_UNIT_2;
    pcnt_config5.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config6 = {};
    pcnt_config6.pulse_gpio_num = ENC3_B;
    pcnt_config6.ctrl_gpio_num = ENC3_A;
    pcnt_config6.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config6.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config6.pos_mode = PCNT_COUNT_INC;
    pcnt_config6.neg_mode = PCNT_COUNT_DEC;
    pcnt_config6.counter_h_lim = COUNTER_H_LIM;
    pcnt_config6.counter_l_lim = COUNTER_L_LIM;
    pcnt_config6.unit = PCNT_UNIT_2;
    pcnt_config6.channel = PCNT_CHANNEL_1;

    pcnt_config_t pcnt_config7 = {};
    pcnt_config7.pulse_gpio_num = ENC4_A;
    pcnt_config7.ctrl_gpio_num = ENC4_B;
    pcnt_config7.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config7.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config7.pos_mode = PCNT_COUNT_INC;
    pcnt_config7.neg_mode = PCNT_COUNT_DEC;
    pcnt_config7.counter_h_lim = COUNTER_H_LIM;
    pcnt_config7.counter_l_lim = COUNTER_L_LIM;
    pcnt_config7.unit = PCNT_UNIT_3;
    pcnt_config7.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config8 = {};
    pcnt_config8.pulse_gpio_num = ENC4_B;
    pcnt_config8.ctrl_gpio_num = ENC4_A;
    pcnt_config8.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config8.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config8.pos_mode = PCNT_COUNT_INC;
    pcnt_config8.neg_mode = PCNT_COUNT_DEC;
    pcnt_config8.counter_h_lim = COUNTER_H_LIM;
    pcnt_config8.counter_l_lim = COUNTER_L_LIM;
    pcnt_config8.unit = PCNT_UNIT_3;
    pcnt_config8.channel = PCNT_CHANNEL_1;

    // パルスカウンタの初期化
    pcnt_unit_config(&pcnt_config1);
    pcnt_unit_config(&pcnt_config2);
    pcnt_unit_config(&pcnt_config3);
    pcnt_unit_config(&pcnt_config4);
    pcnt_unit_config(&pcnt_config5);
    pcnt_unit_config(&pcnt_config6);
    pcnt_unit_config(&pcnt_config7);
    pcnt_unit_config(&pcnt_config8);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_pause(PCNT_UNIT_2);
    pcnt_counter_pause(PCNT_UNIT_3);

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_2);
    pcnt_counter_clear(PCNT_UNIT_3);

    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_2);
    pcnt_counter_resume(PCNT_UNIT_3);

    // チャタリング防止のフィルターを有効化
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_filter_enable(PCNT_UNIT_1);
    pcnt_filter_enable(PCNT_UNIT_2);
    pcnt_filter_enable(PCNT_UNIT_3);

    // フィルター値を設定
    pcnt_set_filter_value(PCNT_UNIT_0, PCNT_FILTER_VALUE);
    pcnt_set_filter_value(PCNT_UNIT_1, PCNT_FILTER_VALUE);
    pcnt_set_filter_value(PCNT_UNIT_2, PCNT_FILTER_VALUE);
    pcnt_set_filter_value(PCNT_UNIT_3, PCNT_FILTER_VALUE);
}

void enc_init_half() {
    // プルアップを有効化
    gpio_set_pull_mode((gpio_num_t)ENC1_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC1_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC2_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC2_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC3_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC3_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC4_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC4_B, GPIO_PULLUP_ONLY);

    // パルスカウンタの設定
    pcnt_config_t pcnt_config1 = {};
    pcnt_config1.pulse_gpio_num = ENC1_A;
    pcnt_config1.ctrl_gpio_num = ENC1_B;
    pcnt_config1.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config1.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config1.pos_mode = PCNT_COUNT_INC;
    pcnt_config1.neg_mode = PCNT_COUNT_DEC;
    pcnt_config1.counter_h_lim = COUNTER_H_LIM;
    pcnt_config1.counter_l_lim = COUNTER_L_LIM;
    pcnt_config1.unit = PCNT_UNIT_0;
    pcnt_config1.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config2 = {};
    pcnt_config2.pulse_gpio_num = ENC1_B;
    pcnt_config2.ctrl_gpio_num = ENC1_A;
    pcnt_config2.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config2.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config2.pos_mode = PCNT_COUNT_INC;
    pcnt_config2.neg_mode = PCNT_COUNT_DEC;
    pcnt_config2.counter_h_lim = COUNTER_H_LIM;
    pcnt_config2.counter_l_lim = COUNTER_L_LIM;
    pcnt_config2.unit = PCNT_UNIT_0;
    pcnt_config2.channel = PCNT_CHANNEL_1;

    pcnt_config_t pcnt_config3 = {};
    pcnt_config3.pulse_gpio_num = ENC2_A;
    pcnt_config3.ctrl_gpio_num = ENC2_B;
    pcnt_config3.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config3.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config3.pos_mode = PCNT_COUNT_INC;
    pcnt_config3.neg_mode = PCNT_COUNT_DEC;
    pcnt_config3.counter_h_lim = COUNTER_H_LIM;
    pcnt_config3.counter_l_lim = COUNTER_L_LIM;
    pcnt_config3.unit = PCNT_UNIT_1;
    pcnt_config3.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config4 = {};
    pcnt_config4.pulse_gpio_num = ENC2_B;
    pcnt_config4.ctrl_gpio_num = ENC2_A;
    pcnt_config4.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config4.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config4.pos_mode = PCNT_COUNT_INC;
    pcnt_config4.neg_mode = PCNT_COUNT_DEC;
    pcnt_config4.counter_h_lim = COUNTER_H_LIM;
    pcnt_config4.counter_l_lim = COUNTER_L_LIM;
    pcnt_config4.unit = PCNT_UNIT_1;
    pcnt_config4.channel = PCNT_CHANNEL_1;

    // パルスカウンタの初期化
    pcnt_unit_config(&pcnt_config1);
    pcnt_unit_config(&pcnt_config2);
    pcnt_unit_config(&pcnt_config3);
    pcnt_unit_config(&pcnt_config4);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_1);

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_1);

    // チャタリング防止のフィルターを有効化
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_filter_enable(PCNT_UNIT_1);

    // フィルター値を設定
    pcnt_set_filter_value(PCNT_UNIT_0, PCNT_FILTER_VALUE);
    pcnt_set_filter_value(PCNT_UNIT_1, PCNT_FILTER_VALUE);
}
