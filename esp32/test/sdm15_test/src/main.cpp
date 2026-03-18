#include "SDM15.h"

// Serial2を使用
HardwareSerial SerialSDM(2);
SDM15 sdm15(SerialSDM);

void setup() {
    Serial.begin(115200);

    // UART2: RX=21, TX=19
    SerialSDM.begin(460800, SERIAL_8N1, 19, 21);

    // 高速通信対策（重要）
    SerialSDM.setRxBufferSize(2048);

    delay(1000);

    Serial.println("===== SDM15 INIT =====");

    // ===== バージョン取得 =====
    VersionInfo info = sdm15.ObtainVersionInfo();

    if (info.checksum_error) {
        Serial.println("VersionInfo checksum error");
    } else {
        Serial.println("Version Info:");
        Serial.printf("model: %d\n", info.model);
        Serial.printf("hardware_version: %d\n", info.hardware_version);
        Serial.printf("firmware_version: %d.%d\n",
                      info.firmware_version_major,
                      info.firmware_version_minor);
        Serial.printf("serial_number: %d\n", info.serial_number);
    }

    // ===== セルフチェック =====
    TestResult test = sdm15.SelfCheckTest();

    if (test.checksum_error) {
        Serial.println("SelfCheck checksum error");
    } else if (test.self_check_result) {
        Serial.println("Self check SUCCESS");
    } else {
        Serial.println("Self check FAILED");
        Serial.printf("error code: %d\n", test.self_check_error_code);
    }

    delay(500);

    // ===== スキャン開始（1回だけ）=====
    if (sdm15.StartScan()) {
        Serial.println("Scan started");
    } else {
        Serial.println("StartScan checksum error");
    }
}

void loop() {
    // データが来てるときだけ読む
    if (SerialSDM.available()) {

        ScanData data = sdm15.GetScanData();

        if (!data.checksum_error) {
            // 軽量ログ（重要）
            Serial.printf("%d,%d,%d\n",
                          data.distance,
                          data.intensity,
                          data.disturb);
        } else {
            Serial.println("checksum error");
        }
    }
}