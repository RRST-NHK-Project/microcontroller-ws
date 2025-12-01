// 外部シリアル変換モジュール
#define DEBUG_SERIAL_TxD 14
#define DEBUG_SERIAL_RxD 18

void setup() {
    // UART0（デフォルト）: USB経由でPCと通信
    Serial.begin(115200);

    // UART1を独自ピンで使用（例: TX=GPIO4, RX=GPIO5）
    Serial1.begin(9600, SERIAL_8N1, DEBUG_SERIAL_RxD, DEBUG_SERIAL_TxD);
}

void loop() {
    uint8_t data = 128;
    Serial.println("Send.");
    Serial1.write(data);
    delay(1000);
}