#include <Arduino.h>

#define Tx16NUM 8
#define START_BYTE 0xAA

#define DEVICE_ID 0x02

int16_t Tx_16Data[Tx16NUM] = {0, 1, 2, 3, 4, 5, 6, 7};
uint8_t Tx_8Data[1 + 1 + 1 + Tx16NUM * 2 + 1];
// START + ID + LENGTH + DATA + CHECKSUM

void setup() {
    Serial.begin(115200);
}

void loop() {
    // フレーム作成
    Tx_8Data[0] = START_BYTE;
    Tx_8Data[1] = DEVICE_ID;
    Tx_8Data[2] = Tx16NUM * 2; // データ長

    uint8_t checksum = 0;
    checksum ^= Tx_8Data[1]; // ID
    checksum ^= Tx_8Data[2]; // LENGTH

    for (int i = 0; i < Tx16NUM; i++) {
        Tx_8Data[3 + i * 2] = (uint8_t)(Tx_16Data[i] >> 8);       // 上位
        Tx_8Data[3 + i * 2 + 1] = (uint8_t)(Tx_16Data[i] & 0xFF); // 下位
        checksum ^= Tx_8Data[3 + i * 2];
        checksum ^= Tx_8Data[3 + i * 2 + 1];
    }

    Tx_8Data[3 + Tx16NUM * 2] = checksum;

    Serial.write(Tx_8Data, 1 + 1 + 1 + Tx16NUM * 2 + 1);

    delay(100);
}
