/*====================================================================
<mode_init.cpp>
・各モードの初期化関数の実装ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include <Arduino.h>

// microROS関連
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int16_multi_array.h>

// 自作ヘッダーファイル
#include "bldc_can.h"     //BLDC用CAN関連を管理
#include "can_defs.h"     //CAN関連を管理
#include "config.h"       //モードやIDを管理
#include "defs.h"         //定数を管理
#include "input_task.h"   //入力系のタスクを管理
#include "mode_init.h"    //各モードの初期化関数を管理
#include "output_task.h"  //出力系のタスクを管理
#include "ros_defs.h"     //microROS関連を管理
#include "robomas_twai.h" //twai関連を管理(ヘッダー名変えて)

// 各モードの初期化関数

void mode1_init()
{
    // モード1用の初期化

    // MDの方向ピンを出力に設定
    pinMode(MD1D, OUTPUT);
    pinMode(MD2D, OUTPUT);
    pinMode(MD3D, OUTPUT);
    pinMode(MD4D, OUTPUT);

    // PWMの初期化
    ledcAttach(MD1P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD2P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD3P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD4P, MD_PWM_FREQ, MD_PWM_RESOLUTION);

    // サーボのPWMの初期化
    ledcAttach(SERVO1, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO2, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO3, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO4, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);

    // トランジスタのピンを出力に設定
    pinMode(TR1, OUTPUT);
    pinMode(TR2, OUTPUT);
    pinMode(TR3, OUTPUT);
    pinMode(TR4, OUTPUT);
    pinMode(TR5, OUTPUT);
    pinMode(TR6, OUTPUT);
    pinMode(TR7, OUTPUT);

    // 受信＆ピン操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        Output_Task,
        "Output_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        PRO_CPU_NUM);
}

void mode2_init()
{
    // モード2用の初期化
    while (1)
    {
        ;
    }
}

void mode3_init()
{
    // モード3用の初期化

    // エンコーダの初期化
    enc_init_all();

    // スイッチのピンを入力に設定し内蔵プルアップ抵抗を有効化
    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW3, INPUT_PULLUP);
    pinMode(SW4, INPUT_PULLUP);

    // Rev.3からそのまま、そのうち変える
    msg.data.data = (int16_t *)malloc(sizeof(int16_t) * 8);
    msg.data.size = 8;
    msg.data.capacity = 8;

    xTaskCreateUniversal(
        ENC_PRI_Read_Publish_Task,
        "ENC_PRI_Read_Publish_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    xTaskCreateUniversal(
        LED_PWM_Task,
        "LED_PWM_Task",
        2048,
        NULL,
        1, // 優先度、最大25？
        &led_pwm_handle,
        APP_CPU_NUM);
}

void mode4_init()
{
    // エンコーダの初期化
    enc_init_half();

    // スイッチのピンを入力に設定し内蔵プルアップ抵抗を有効化
    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW3, INPUT_PULLUP);
    pinMode(SW4, INPUT_PULLUP);
    pinMode(SW5, INPUT_PULLUP);
    pinMode(SW6, INPUT_PULLUP);
    pinMode(SW7, INPUT_PULLUP);
    pinMode(SW8, INPUT_PULLUP);

    // Rev.3からそのまま、そのうち変える
    msg.data.data = (int16_t *)malloc(sizeof(int16_t) * 11);
    msg.data.size = 11;
    msg.data.capacity = 11;

    xTaskCreateUniversal(
        SW_PRI_Read_Publish_Task,
        "SW_PRI_Read_Publish_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    xTaskCreateUniversal(
        LED_PWM_Task,
        "LED_PWM_Task",
        2048,
        NULL,
        1, // 優先度、最大25？
        &led_pwm_handle,
        APP_CPU_NUM);
}

void mode5_init()
{

    // Serial1.setTxBufferSize(1024);
    // Serial1.begin(115200, SERIAL_8N1, 17, 18);
    while (!Serial)
        ;

    CAN.setPins(CAN_RX, CAN_TX); // rx.tx

    if (!CAN.begin(1000E3))
    {
        // Serial1.println("Starting CAN failed!");
        while (1)
            ;
    }

    //  xTaskCreateUniversal(
    //     CAN_Pb,
    //     "CAN_Pb",
    //     4096,
    //     NULL,
    //     5, // 優先度、最大25？
    //     NULL,
    //     APP_CPU_NUM);

    xTaskCreateUniversal(
        C620_Task_v2,
        "C620_Task_v2",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    xTaskCreateUniversal(
        ROBOMAS_ENC_SW_Read_Publish_Task,
        "ROBOMAS_ENC_SW_Read_Publish_Task",
        4096,
        NULL,
        1, // 優先度、最大25？
        NULL,
        0);

    xTaskCreateUniversal(
        LED_PWM_Task,
        "LED_PWM_Task",
        2048,
        NULL,
        1, // 優先度、最大25？
        &led_pwm_handle,
        APP_CPU_NUM);
}

void mode6_init()
{

    Serial1.setTxBufferSize(1024);
    Serial1.begin(115200, SERIAL_8N1, 17, 18);
    while (!Serial)
        ;

    // CAN.setPins(CAN_RX, CAN_TX); // rx.tx
    CAN.setPins(4, 5); // rx.tx

    if (!CAN.begin(1000E3))
    {
        Serial1.println("Starting CAN failed!");
        while (1)
            ;
    }
    ledcAttach(16, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    pinMode(SV1, OUTPUT);
    pinMode(26, INPUT_PULLUP);
    pinMode(27, INPUT_PULLUP);

    //  xTaskCreateUniversal(
    //     CAN_Pb,
    //     "CAN_Pb",
    //     4096,
    //     NULL,
    //     5, // 優先度、最大25？
    //     NULL,
    //     APP_CPU_NUM);

    xTaskCreateUniversal(
        CR25_Task,
        "CR25_Task",
        4096,
        NULL,
        1, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    // ソレノイド操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        SV_Task,
        "SV_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    xTaskCreateUniversal(
        LED_PWM_Task,
        "LED_PWM_Task",
        2048,
        NULL,
        1, // 優先度、最大25？
        &led_pwm_handle,
        APP_CPU_NUM);
}

void mode7_init()
{
    Serial.begin(115200);
    delay(1000);

    // TWAI 設定
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    {
        // Serial.println("TWAI install failed");
        while (1)
            ;
    }
    if (twai_start() != ESP_OK)
    {
        // Serial.println("TWAI start failed");
        while (1)
            ;
    }
    xTaskCreateUniversal(
        C620_twai,
        "C620_twai",
        4096,
        NULL,
        1, // 優先度、最大25？
        NULL,
        0);
}

void mode8_init()
{
    xTaskCreateUniversal(
        BLDC_CAN_Send_Task,
        "BLDC_CAN_Send_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        0); // コア0にすると安定したのでこのままで
}

void mode9_init()
{
    xTaskCreateUniversal(
        BLDC_CAN_16bit_Receive_Task,
        "BLDC_CAN_16bit_Receive_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);
}

// テストモード　※実機で「絶対」に実行するな！
// シリアルモニターからEnterが押されるまで待機する
void mode0_init()
{
    while (1)
    {
        // テストモード実装予定
        ;
    }
}