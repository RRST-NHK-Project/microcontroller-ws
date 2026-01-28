/*====================================================================
<serial_task.cpp>
・シリアル通信まわりのタスク実装

PCから送信されるデータ構造（フレーム）は以下のとおり。

Frame Structure:
[START_BYTE][DEVICE_ID][LENGTH][DATA...][CHECKSUM]
- START_BYTE: 0xAA
- DEVICE_ID: 0x02
- LENGTH: Number of data bytes (Tx16NUM * 2)
- DATA: int16 data (big-endian)
- CHECKSUM: XOR of all bytes except START_BYTE

START_BYTE（1バイト）
   - フレームの開始を示す固定値（ROS側とマイコンで揃える）
   - 0xAA

ID（1バイト）
   - 送信元または宛先デバイスID（マイコンごとに異なる値を設定する）
   - 例: DEVICE_ID

LEN（1バイト）
   - データ部分（DATA）の長さ（バイト単位）
   - 例: Tx16NUM * 2（16bitデータをバイト数に換算）

DATA（可変長）
   - 16bit単位のデータ配列
   - Tx16NUM / Rx16NUM の数だけ送信
   - 1つの16bitデータは
        上位バイト: (data >> 8)
        下位バイト: (data & 0xFF)

CHECKSUM（1バイト）
   - フレームの整合性チェック用
   - 計算方法: ID ^ LEN ^ DATA[0] ^ DATA[1] ^ ... ^ DATA[n]

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "config.hpp"
#include "defs.hpp"
#include "frame_data.hpp"
#include <Arduino.h>

#define START_BYTE 0xAA

// ループバックの有効化設定（実装途中）
#define ENABLE_LOOPBACK 0 // 受信したデータをそのまま送り返す

// ================= TX =================

uint8_t Tx_8Data[1 + 1 + 1 + Tx16NUM * 2 + 1];
// START + ID + LEN + DATA + CHECKSUM

// ================= RX =================

// RAW フレーム保持（loopback 用）
uint8_t Rx_raw_frame[1 + 1 + 1 + Rx16NUM * 2 + 1];
uint8_t Rx_raw_len = 0;

enum RxState {
    WAIT_START,
    WAIT_ID,
    WAIT_LEN,
    WAIT_DATA,
    WAIT_CHECKSUM
};

RxState rx_state = WAIT_START;

uint8_t rx_id = 0;
uint8_t rx_len = 0;
uint8_t rx_buf[Rx16NUM * 2];
uint8_t rx_index = 0;
uint8_t rx_checksum = 0;

constexpr uint32_t TX_PERIOD_MS = 20; // 送信周期（ミリ秒）
constexpr uint32_t RX_PERIOD_MS = 10; // 受信周期（ミリ秒）

void send_frame();
void receive_frame();

// ================= TASK =================

// 送受信タスク
void serialTask(void *) {
    TickType_t last_tx = xTaskGetTickCount();

    while (1) {
        // RXは毎ループ呼び出す
        receive_frame();

        // TXのみ周期管理
        if (xTaskGetTickCount() - last_tx >= pdMS_TO_TICKS(TX_PERIOD_MS)) {
            send_frame();
            last_tx = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// TX,RXでタスクを分けると不具合が発生、シリアルに２つのタスクからアクセスしているのが原因？
// 以下２つのタスクは削除予定

// 送信タスク（削除予定）
void txTask(void *) {
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        send_frame();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TX_PERIOD_MS));
    }
}
// 受信タスク（削除予定）
void rxTask(void *) {
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        receive_frame();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(RX_PERIOD_MS));
    }
}

// ================= TX =================

void send_frame() {
    Tx_8Data[0] = START_BYTE;
    Tx_8Data[1] = DEVICE_ID;
    Tx_8Data[2] = Tx16NUM * 2;

    uint8_t checksum = 0;
    checksum ^= Tx_8Data[1];
    checksum ^= Tx_8Data[2];

    for (int i = 0; i < Tx16NUM; i++) {
        Tx_8Data[3 + i * 2] = (uint8_t)(Tx_16Data[i] >> 8);
        Tx_8Data[3 + i * 2 + 1] = (uint8_t)(Tx_16Data[i] & 0xFF);
        checksum ^= Tx_8Data[3 + i * 2];
        checksum ^= Tx_8Data[3 + i * 2 + 1];
    }

    Tx_8Data[3 + Tx16NUM * 2] = checksum;

    // #if !ENABLE_LOOPBACK
    //     // ===== LOOPBACK =====
    //     Serial.write(Tx_8Data, sizeof(Tx_8Data));
    // #endif
    Serial.write(Tx_8Data, sizeof(Tx_8Data));
}

// ================= RX =================

void receive_frame() {
    while (Serial.available()) {
        uint8_t b = Serial.read();

        switch (rx_state) {

        case WAIT_START:
            if (b == START_BYTE) {
                rx_state = WAIT_ID;
            }
            break;

        case WAIT_ID:
            rx_id = b;
            rx_checksum = b;
            rx_state = WAIT_LEN;
            break;

        case WAIT_LEN:
            rx_len = b;
            rx_checksum ^= b;

            if (rx_len > Rx16NUM * 2) {
                rx_state = WAIT_START; // 不正LEN
            } else {
                rx_index = 0;
                rx_state = WAIT_DATA;
            }
            break;

        case WAIT_DATA:
            rx_buf[rx_index++] = b;
            rx_checksum ^= b;

            if (rx_index >= rx_len) {
                rx_state = WAIT_CHECKSUM;
            }
            break;

        case WAIT_CHECKSUM:
            if (rx_checksum == b && rx_id == DEVICE_ID) {

                // digitalWrite(F446RE_BUILTIN_LED, !digitalRead(F446RE_BUILTIN_LED));
                //    digitalWrite(F446RE_BUILTIN_LED, HIGH); // これはテスト用、消して！

                // ===== RAW フレーム保存 =====
                Rx_raw_frame[0] = START_BYTE;
                Rx_raw_frame[1] = rx_id;
                Rx_raw_frame[2] = rx_len;

                for (int i = 0; i < rx_len; i++) {
                    Rx_raw_frame[3 + i] = rx_buf[i];
                }

                Rx_raw_frame[3 + rx_len] = b;
                Rx_raw_len = 1 + 1 + 1 + rx_len + 1;

                // ===== int16 デコード =====
                for (int i = 0; i < rx_len / 2; i++) {
                    Rx_16Data[i] =
                        (int16_t)((rx_buf[i * 2] << 8) |
                                  rx_buf[i * 2 + 1]);
                }

#if ENABLE_LOOPBACK
                // ===== LOOPBACK =====
                Serial.write(Rx_raw_frame, Rx_raw_len);
#endif
            }

            rx_state = WAIT_START;
            break;
        }
    }
}
