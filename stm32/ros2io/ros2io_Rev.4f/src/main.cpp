/*====================================================================
ros2io Rev.4f alpha
Target board: NUCLEO-F446RE

Description:
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

#define Tx16NUM 8
#define START_BYTE 0xAA
#define DEVICE_ID 0x02

#define ENABLE_LOOPBACK 1 // 受信したデータをそのまま送り返す

// ================= TX =================

int16_t Tx_16Data[Tx16NUM] = {0};
uint8_t Tx_8Data[1 + 1 + 1 + Tx16NUM * 2 + 1];
// START + ID + LEN + DATA + CHECKSUM

// ================= RX =================

int16_t Rx_16Data[Tx16NUM] = {0};

// RAW フレーム保持（loopback 用）
uint8_t Rx_raw_frame[1 + 1 + 1 + Tx16NUM * 2 + 1];
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
uint8_t rx_buf[Tx16NUM * 2];
uint8_t rx_index = 0;
uint8_t rx_checksum = 0;

// ================= SETUP =================

void setup() {
    Serial.begin(115200);
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

            if (rx_len > Tx16NUM * 2) {
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

// ================= LOOP =================

void loop() {
    send_frame();    // マイコン → PC
    receive_frame(); // PC → マイコン

    delay(10);
}
