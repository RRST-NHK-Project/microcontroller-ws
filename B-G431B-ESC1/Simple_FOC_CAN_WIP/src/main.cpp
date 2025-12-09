#include "acanfd-stm32/ACANFD_STM32.h"
#include <Arduino.h>
#include <SimpleFOC.h>

//-----------------------------------------------------------------//
// Simple FOC
//  モータ極数
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

//-----------------------------------------------------------------//

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(50);

    Serial.println("STM32 FDCAN Receiver (ESP32 TWAI compatible)");

    //-----------------------------------------------------------------//
    // FDCAN　初期化 (ESPに合わせてClassicCANにしたほうがいいかも)
    // CAN 設定: 500 kbps, NORMAL FD
    ACANFD_STM32_Settings settings(500 * 1000, DataBitRateFactor::x5);
    settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD;
    settings.mEnableRetransmission = false;

    // RX/TX ピン設定（接続したトランシーバに合わせる）
    settings.mTxPin = PB_9;  // TXは必要なら
    settings.mRxPin = PA_11; // ESP32 TXに接続

    // フィルター設定
    ACANFD_STM32_StandardFilters standardFilters;
    ACANFD_STM32_ExtendedFilters extendedFilters;

    // 標準ID 0x431 を受信
    standardFilters.addSingle(0x431, ACANFD_STM32_FilterAction::FIFO0);
    // 拡張ID 0x320431 を受信
    extendedFilters.addSingle(0x320431, ACANFD_STM32_FilterAction::FIFO0);

    // 非マッチフレームも受信
    settings.mNonMatchingStandardFrameReception = ACANFD_STM32_FilterAction::FIFO0;
    settings.mNonMatchingExtendedFrameReception = ACANFD_STM32_FilterAction::FIFO0;

    // FDCAN 初期化
    uint32_t errorCode = fdcan1.beginFD(settings, standardFilters, extendedFilters);
    if (errorCode == 0) {
        Serial.println("fdcan1 initialized successfully");
    } else {
        Serial.print("fdcan1 failed: 0x");
        Serial.println(errorCode, HEX);
        while (true)
            ;
    }
    //-----------------------------------------------------------------//

    //-----------------------------------------------------------------//
    // Simple FOC 初期化
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
    // motor.controller = MotionControlType::velocity;

    // 位置制御
    motor.controller = MotionControlType::angle;

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
    //-----------------------------------------------------------------//
}

void loop() {

    CANFDMessage rx_msg;
    int16_t msg_data[4];

    // Smple FOC ループ
    motor.loopFOC();
    motor.move();
    command.run();

    // CAN受信処理
    if (fdcan1.receiveFD0(rx_msg)) {
        // ID 表示
        if (rx_msg.ext) {
            Serial.print("Extended ID: 0x");
            Serial.print(rx_msg.id, HEX);
        } else {
            Serial.print("Standard ID: 0x");
            Serial.print(rx_msg.id, HEX);
        }

        // 受信データの復元
        msg_data[0] = (int16_t)(rx_msg.data[0] << 8 | rx_msg.data[1]);
        msg_data[1] = (int16_t)(rx_msg.data[2] << 8 | rx_msg.data[3]);
        msg_data[2] = (int16_t)(rx_msg.data[4] << 8 | rx_msg.data[5]);
        msg_data[3] = (int16_t)(rx_msg.data[6] << 8 | rx_msg.data[7]);

        float SCALE = 1.0f; // スケール変更
        float target_angle = (float)msg_data[0] * DEG_TO_RAD / SCALE;
        motor.target = target_angle;

        // DLC & データ表示
        Serial.print(" DLC: ");
        Serial.print(rx_msg.len);
        Serial.print(" Data: ");
        for (int i = 0; i < 4; i++) {
            Serial.print(msg_data[i]);
            Serial.print(" ");
        }
        Serial.println();
    }
}
