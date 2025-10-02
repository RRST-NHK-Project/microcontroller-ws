/*<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
ros2io Rev.3
Copyright © 2025 RRST-NHK-Project. All rights reserved.
<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>*/

/*
For NHK-Robocon-2026
ESP32用microROSプログラム。ROSメッセージからマイコンのIOを操作する。
ライブラリはros2io/librariesに格納。修正済みのライブラリを使用しているため同梱のZipを使用すること。
ボードマネージャーはesp32 by Espressif Systemsを選択
config.hでIDとMODEを設定する。
*/

/*
TODO:ROSからのモード選択機能の実装（保留中）
TODO:定数定義の統一
FIXME:Pub、Subの同時使用時の遅延問題
*/

#include <Arduino.h>
#include <CAN.h>
#include <esp32-hal-ledc.h>

// microROS関連
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32_multi_array.h>

//  パルスカウンタ関連
#include "driver/pcnt.h"

// 自作ヘッダーファイル
#include "can_defs.h"    //CAN関連を管理
#include "vel_pid.h"    //速度制御用PIDを管理
#include "config.h"      //モードやIDを管理
#include "defs.h"        //定数を管理
#include "input_task.h"  //入力系のタスクを管理
#include "output_task.h" //出力系のタスクを管理
#include "ros_defs.h"    //microROS関連を管理

void setup() {

    pinMode(LED, OUTPUT);

    // MODEに応じた初期化
    switch (MODE) {
    case 0:
        Serial.begin(115200);
        mode0_init();
        break;
    case 1:
        ros_init();
        mode1_init();
        break;
    case 2:
        ros_init();
        mode2_init();
        break;
    case 3:
        ros_init();
        mode3_init();
        break;
    case 4:
        ros_init();
        mode4_init();
        break;
    case 5:
        ros_init();
        mode5_init();
        break;
    case 6://実験用いらなくなったら消す
        ros_init();
        mode6_init();
        break;
    default:;
        ;
    }
}

void loop() {
    if (MODE != 0) {
        RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
        vTaskDelay(1);
    }
}

// 各モードの初期化関数
void mode1_init() {
    // モード1用の初期化

    // PWMの初期化
    ledcAttach(MD1P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD2P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD3P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD4P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD5P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD6P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD7P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD8P, MD_PWM_FREQ, MD_PWM_RESOLUTION);

    // 受信＆ピン操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        MD_Output_Task,
        "MD_Output_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        PRO_CPU_NUM);
}

void mode2_init() {
    // モード2用の初期化
    // SerialBT.println("Mode 2 Initialized");

    // エンコーダの初期化
    enc_init();

    // スイッチのピンを入力に設定し内蔵プルアップ抵抗を有効化
    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW3, INPUT_PULLUP);
    pinMode(SW4, INPUT_PULLUP);

    CAN.setPins(CAN_RX, CAN_TX); // rx.tx
    if (!CAN.begin(1000E3)) {
        while (1)
            ;
    }

    msg.data.data = (int32_t *)malloc(sizeof(int32_t) * 20);
    msg.data.size = 20;
    msg.data.capacity = 20;

    xTaskCreateUniversal(
        ROBOMAS_ENC_SW_Read_Publish_Task,
        "ROBOMAS_ENC_SW_Read_Publish_Task",
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

void mode3_init() {
    // モード3用の初期化
    // SerialBT.println("Mode 3 Initialized");

    // サーボのPWMの初期化
    ledcAttach(SERVO1, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO2, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO3, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO4, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO5, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO6, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO7, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO8, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);

    pinMode(SV1, OUTPUT);
    pinMode(SV2, OUTPUT);
    pinMode(SV3, OUTPUT);
    pinMode(SV4, OUTPUT);
    pinMode(SV5, OUTPUT);
    pinMode(SV6, OUTPUT);
    pinMode(SV7, OUTPUT);

    // サーボ操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        Servo_Output_Task,
        "Servo_Output_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
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
}

void mode4_init() {
    // モード4用の初期化
    // SerialBT.println("Mode 4 Initialized");
    // Pub,Subを同時にしたときの遅延問題が解決できていないため未使用
    // とりあえず書いておく

    // エンコーダの初期化
    CAN.setPins(CAN_RX, CAN_TX); // rx.tx
    if (!CAN.begin(1000E3)) {
        while (1)
            ;
    }

    msg.data.data = (int32_t *)malloc(sizeof(int32_t) * 20);
    msg.data.size = 20;
    msg.data.capacity = 20;

    xTaskCreateUniversal(
        C610_FB_Task,
        "C610_FB_Task",
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

void mode5_init() {

    CAN.setPins(CAN_RX, CAN_TX); // rx.tx
    if (!CAN.begin(1000E3)) {
        while (1)
            ;
    }

    xTaskCreateUniversal(
        C610_Task,
        "C610_Task",
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

void mode6_init() {
//ロボマスに速度制御用PIDを組み込んだ実験用モード
    CAN.setPins(CAN_RX, CAN_TX); // rx.tx
    if (!CAN.begin(1000E3)) {
        while (1)
            ;
    }

    xTaskCreateUniversal(
        C610_vel_Task,
        "C610_vel_Task",
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


// テストモード　※実機で「絶対」に実行するな！
// シリアルモニターからEnterが押されるまで待機する
void mode0_init() {
    // デバッグ・テスト用の初期化
    Serial.println("Debug/Test Mode Initialized.");
    Serial.println("Press Enter to continue...");

    xTaskCreateUniversal(
        LED_Blink100_Task,
        "LED_Blink100_Task",
        2048,
        NULL,
        1, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    // テストモードの安全装置
    while (1) {
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') { // Enterが押されたら抜ける
                break;
            }
        }
    }

    switch (TEST_MODE) {
    case 0:
        // なにもしない
        Serial.println("MODE_DUMMY");
        while (1) {
            ;
        }
        break;
    case 1:
        // MDのテスト
        Serial.println("MD_TEST");
        mode1_init();
        while (1) {
            Serial.println("MD:20%");
            for (int i = 1; i <= 8; i++) {
                received_data[i] = 20;
            }
            delay(1000);

            Serial.println("MD:0%");
            for (int i = 1; i <= 8; i++) {
                received_data[i] = 0;
            }
            delay(1000);

            Serial.println("MD:-20%");
            for (int i = 1; i <= 8; i++) {
                received_data[i] = -20;
            }
            delay(1000);
        }
        break;
    case 2:
        // エンコーダ、スイッチのテスト
        Serial.println("ENC/SW_TEST");
        enc_init();
        mode2_init();
        while (1) {
            for (int i = 0; i < 4; i++) {
                Serial.print("count[");
                Serial.print(i);
                Serial.print("] = ");
                Serial.print(count[i]);
                Serial.print("\t sw_state[");
                Serial.print(i);
                Serial.print("] = ");
                Serial.println(sw_state[i]);
            }
            Serial.println("------");
        }
        break;
    case 3:
        // サーボ、ソレノイドのテスト
        Serial.println("SERVO/SV_TEST");
        mode3_init();
        while (1) {
            for (int i = 9; i <= 16; i++) {
                Serial.print("SERVO");
                Serial.print(i - 8);
                Serial.println(" Sweep");
                for (int angle = 0; angle <= 180; angle += 10) {
                    received_data[i] = angle;
                    delay(500);
                }
                for (int angle = 180; angle >= 0; angle -= 10) {
                    received_data[i] = angle;
                    delay(500);
                }
            }
            for (int i = 17; i <= 23; i++) {
                Serial.print("SV");
                Serial.print(i - 16);
                Serial.println(" ON");
                received_data[i] = 1;
                delay(1000);
                Serial.print("SV");
                Serial.print(i - 16);
                Serial.println(" OFF");
                received_data[i] = 0;
                delay(1000);
            }
        }
        break;

    case 5:
        // CANのテスト
        Serial.println("CAN_TEST");
        mode5_init();
        while (1) {
            // unsigned long now = millis();
            // float dt = (now - lastPidTime) / 1000.0;
            // if (dt <= 0)
            //     dt = 0.000001f; // dtが0にならないようにする
            // lastPidTime = now;

            // // 1. CAN受信
            // int packetSize = CAN.parsePacket();
            // while (packetSize) {               // 複数パケットも処理
            //     if (CAN.packetId() == 0x201) { // モータID=1
            //         uint8_t rx[8];
            //         for (int i = 0; i < 8; i++)
            //             rx[i] = CAN.read();
            //         encoder_count = (rx[0] << 8) | rx[1];
            //         rpm = (rx[2] << 8) | rx[3];

            //         // --- 初回オフセット設定 --- //
            //         if (!offset_ok) {
            //             encoder_offset = encoder_count;
            //             last_encoder_count = -1;
            //             rotation_count = 0;
            //             total_encoder_count = 0;
            //             pos_integral = 0;
            //             pos_error_prev = 0;
            //             offset_ok = true;
            //             // Serial.println("Offset set!");
            //         }

            //         int enc_relative = encoder_count - encoder_offset;
            //         if (enc_relative < 0)
            //             enc_relative += ENCODER_MAX; // wrap-around補正

            //         if (last_encoder_count != -1) {
            //             int diff = encoder_count - last_encoder_count;
            //             if (diff > HALF_ENCODER)
            //                 rotation_count--;
            //             else if (diff < -HALF_ENCODER)
            //                 rotation_count++;
            //         }

            //         last_encoder_count = encoder_count;
            //         total_encoder_count = rotation_count * ENCODER_MAX + encoder_count;
            //         angle = total_encoder_count * (360.0 / (8192.0 * gear_ratio));
            //         vel_input = (rpm / gear_ratio) * 360.0 / 60.0;
            //     }
            //     packetSize = CAN.parsePacket(); // 次の受信も処理
            // }
            // float pos_output = pid(target_angle, angle, pos_error_prev, pos_integral, kp_pos, ki_pos, kd_pos, dt);
            // // float vel_output = pid(pos_output, vel_input, vel_error_prev, vel_integral, kp_vel, ki_vel, kd_vel, dt);
            // motor_output_current_A = constrain_double(pos_output, -current_limit_A, current_limit_A);
            // // motor_output_current_A = 0.3;
            // // 2. コマンド送信
            // send_cur(motor_output_current_A);

            // // 3. デバッグ出力
            // // Serial.print("pos:\t"); Serial.println(angle);
            // Serial.println(target_angle - angle);

            // delay(1);
        }
        break;

    default:
        Serial.println("Invalid MODE for Test Mode.");
        while (1) {
            ;
        }
        break;
    }
}