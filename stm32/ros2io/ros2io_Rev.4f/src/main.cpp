#include <Arduino.h>

const uint8_t DEVICE_ID = 0x01;

void setup() {
    Serial.begin(115200);
}

void loop() {
    Serial.write(DEVICE_ID);
    delay(100);
}
