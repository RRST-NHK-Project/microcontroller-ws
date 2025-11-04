#include <Wire.h>
#include <stdint.h>

#define AS5600_ADDR 0x36
#define REG_RAW_ANGLE_H 0x0C
#define REG_AGC 0x1A
#define REG_MAGNITUDE_H 0x1B // MAG is 2 bytes (MSB, LSB)

void setup() {
    Serial.begin(115200);
    Wire.begin(); // ESP32なら Wire.begin(SDA_pin, SCL_pin);
    Wire.setClock(100000);
    delay(100);
}

uint16_t readRawAngle() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_RAW_ANGLE_H);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() < 2)
        return 0xFFFF;
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    return ((uint16_t)(hi & 0x0F) << 8) | lo;
}

uint8_t readAGC() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_AGC);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 1);
    if (Wire.available() < 1)
        return 0xFF;
    return Wire.read();
}

uint16_t readMagnitude() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_MAGNITUDE_H);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() < 2)
        return 0xFFFF;
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    return ((uint16_t)hi << 8) | lo; // magnitudeは12bit扱いの場合あり
}

void loop() {
    uint16_t raw = readRawAngle();
    uint8_t agc = readAGC();
    uint16_t mag = readMagnitude();

    if (raw != 0xFFFF) {
        float deg = (raw / 4096.0) * 360.0;
        Serial.print("Raw: ");
        Serial.print(raw);
        Serial.print("  Deg: ");
        Serial.print(deg, 2);
    } else {
        Serial.print("Raw: ERR ");
    }
    Serial.print("  AGC: ");
    Serial.print(agc);
    Serial.print("  MAG: ");
    Serial.println(mag);
    delay(200);
}
