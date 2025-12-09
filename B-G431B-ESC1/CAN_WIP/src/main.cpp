#include "acanfd-stm32/ACANFD_STM32.h"
#include <Arduino.h>

//-----------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(50);

    Serial.println("STM32 FDCAN Receiver (ESP32 TWAI compatible)");

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
}

//-----------------------------------------------------------------

void loop() {
    CANFDMessage rx_msg;
    int16_t msg_data[4];

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
