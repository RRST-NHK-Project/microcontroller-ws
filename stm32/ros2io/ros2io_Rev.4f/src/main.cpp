#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    delay(500);         // PC側がポートを開く時間
    Serial.write(0x01); // 固定ID送信
}

void loop() {
    ;
}
