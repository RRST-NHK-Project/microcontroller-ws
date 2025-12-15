#include <Arduino.h>

#define RxNUM 16
#define Tx16NUM 8 // 送信データサイズ（16ビット時の要素数）

#define CMD_ACK 0x20
#define CMD_ESTOP 0xFF

const uint8_t DEVICE_ID = 0x01;
int16_t Rx_Data[RxNUM] = {0};

int16_t Tx_16Data[Tx16NUM] = {0, 1, 2, 3, 4, 5, 6, 7};

// 1 byte(ID) + 16bit × Tx16NUM
uint8_t Tx_8Data[1 + Tx16NUM * 2] = {0};

bool connected = false;

void setup() {
    Tx_8Data[0] = DEVICE_ID;
    Serial.begin(115200);
    // Serial.write(DEVICE_ID);
}

void loop() {

    // ===== ID送信フェーズ =====
    while (!connected) {
        Serial.write(DEVICE_ID); // IDのみ送信
        delay(100);

        // PCからの応答確認
        if (Serial.available() > 0) {
            uint8_t rx = Serial.read();
            if (rx == CMD_ACK) {
                connected = true; // 接続確定
                delay(500);
            }
        }
    }

    Tx_8Data[0] = DEVICE_ID;
    for (int i = 0; i < Tx16NUM; i++) {
        Tx_8Data[1 + i * 2] = (uint8_t)(Tx_16Data[i] >> 8);       // 上位バイト
        Tx_8Data[1 + i * 2 + 1] = (uint8_t)(Tx_16Data[i] & 0xFF); // 下位バイト
    }
    Serial.write(Tx_8Data, 1 + Tx16NUM * 2);
    delay(100);
}
