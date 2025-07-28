// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
//2025/07/28

#include "Arduino.h"
#include <CAN.h>
#include <Ticker.h>
const int LED_PIN = 2;

Ticker tick;
void setup() {
    pinMode(LED_PIN, OUTPUT);
    tick.attach_ms(1000, []() {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    });
    Serial.begin(250000);
    while (!Serial)
        ;

    Serial.println("CAN Receiver");

    // start the CAN bus at 500 kbps
    CAN.setPins(26, 25);
    if (!CAN.begin(500E3)) {
        Serial.println("Starting CAN failed!");
        while (1)
            ;
    }
}

void loop() {
    // try to parse packet
    int packetSize = CAN.parsePacket();

    if (packetSize) {
        // received a packet
        Serial.print("Received ");

        if (CAN.packetExtended()) {
            Serial.print("extended ");
        }

        if (CAN.packetRtr()) {
            // Remote transmission request, packet contains no data
            Serial.print("RTR ");
        }

        Serial.print("packet with id 0x");
        Serial.print(CAN.packetId(), HEX);

        if (CAN.packetRtr()) {
            Serial.print(" and requested length ");
            Serial.println(CAN.packetDlc());
        } else {
            Serial.print(" and length ");
            Serial.println(packetSize);

            // only print packet data for non-RTR packets
            while (CAN.available()) {
                Serial.print((char)CAN.read());
            }
            Serial.println();
        }

        Serial.println();
    }
}