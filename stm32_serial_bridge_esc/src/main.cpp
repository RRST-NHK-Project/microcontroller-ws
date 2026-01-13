/*====================================================================
Project: stm32_serial_bridge
Target board: NUCLEO-F446RE

コメント整備中

Description:
  ROS 2・マイコン間の通信を行うserial_bridgeパッケージのマイコン側プログラム。
  PCから送られてくるバイナリデータを受信、デコードしマイコンのGPIO出力に反映させる。

  This is a simple serial communication example between a microcontroller
  and a PC using a custom binary protocol. The microcontroller sends and
  receives frames containing int16 data.

  Frame Structure:
  [START_BYTE][DEVICE_ID][LENGTH][DATA...][CHECKSUM]
    - START_BYTE: 0xAA
    - DEVICE_ID: 0x02
    - LENGTH: Number of data bytes (Tx16NUM * 2)
    - DATA: int16 data (big-endian)
    - CHECKSUM: XOR of all bytes except START_BYTE

  The microcontroller sends Tx16NUM int16 values to the PC and listens for
  incoming frames from the PC. Received frames are validated using the
  checksum and stored in Rx_16Data array.

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <SimpleFOC.h>
#include <defs.hpp>
#include <led_task.hpp>
#include <serial_task.hpp>

// モータ極数
BLDCMotor motor = BLDCMotor(7);

// ドライバ設定
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// 電流センサ設定
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// エンコーダ設定　(A,B相,PPR,Index)
Encoder encoder = Encoder(A_HALL1, A_HALL2, 2048, A_HALL3);

void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
void doIndex() { encoder.handleIndex(); }

Commander command = Commander(Serial);
void doTarget(char *cmd) { command.motion(&motor, cmd); }

void FOC_Init() {
    encoder.init();
    encoder.enableInterrupts(doA, doB, doIndex);

    motor.linkSensor(&encoder);

    // 電源電圧設定
    driver.voltage_power_supply = 24;

    driver.init();
    motor.linkDriver(&driver);

    currentSense.linkDriver(&driver);
    currentSense.init();
    currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);

    motor.voltage_sensor_align = 1;
    motor.velocity_index_search = 3;

    // モータの電圧の制限
    motor.voltage_limit = 12;
    // モータの速度の制限？
    motor.velocity_limit = 1000;
    // モータの電流の制限
    // motor.current_limit = 40;

    // 以下コントローラ設定、コメントアウトで速度、位置制御の切り替え
    // 速度制御
    motor.controller = MotionControlType::velocity;

    // 位置制御
    //  motor.controller = MotionControlType::angle;

    // トルク制御方式の設定
    motor.torque_controller = TorqueControlType::foc_current;

    // q軸,d軸のPIDゲイン設定（q軸,d軸ってなんですか？）
    motor.PID_current_q.P = motor.PID_current_d.P = 0.1;
    motor.PID_current_q.I = motor.PID_current_d.I = 10;
    motor.PID_current_q.D = motor.PID_current_d.D = 0;

    // 速度制御のPIDゲイン設定
    motor.PID_velocity.P = 0.5;
    motor.PID_velocity.I = 1;
    motor.PID_velocity.D = 0;

    // 速度制御の出力変化速度制限
    motor.PID_velocity.output_ramp = 1000;

    // LPFの設定
    motor.LPF_velocity.Tf = 0.01;

    // 位置制御のPゲイン設定、SimpleFOCの位置制御はP制御のみらしい
    motor.P_angle.P = 9;

    Serial.begin(115200);
    motor.useMonitoring(Serial);

    motor.init();
    motor.initFOC();
    command.add('T', doTarget, "target angle");

    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target angle using serial terminal:"));
    _delay(1000);
}

// ================= SETUP =================

void setup() {

    // プログラムが書き込めなくなるバグの応急処置
    delay(2000); // 安定待ち

    // ボーレートは実機テストしながら調整する予定
    Serial.begin(115200);

    pinMode(BG431B_BUILTIN_LED, OUTPUT);

    FOC_Init();

    // 以降FreeRTOSタスク関連

    xTaskCreate(
        serialTask,   // タスク関数
        "serialTask", // タスク名
        256,          // スタックサイズ（words）
        NULL,
        10, // 優先度
        NULL);

    vTaskStartScheduler();
}

// ================= LOOP =================

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    // メインループではFOC制御を実行
    motor.loopFOC();
    motor.move();
    command.run();
}
