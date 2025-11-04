void setup() {
  // UART0（デフォルト）: USB経由でPCと通信
  Serial.begin(115200);

  // UART1を独自ピンで使用（例: TX=GPIO4, RX=GPIO5）
  Serial1.begin(9600, SERIAL_8N1, 5, 4);
}

void loop() {
  Serial.println("0");
  Serial1.println("1");

}