#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
byte txBuf[8] = {0};  // 送信用バッファ
unsigned long Pre_millis = 0;

// CSピンはUNOのデフォルトSPI CSピンの10に設定
MCP_CAN CAN0(10);

int16_t fmap(double x, double in_min, double in_max, int16_t out_min, int16_t out_max);

void setup()
{
  Serial.begin(115200);

  // MCP2515初期化。MCP_ANYは受信ID指定なし、速度は1000kbps、クロック8MHz
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
  {
    Serial.println("CAN0: Init OK!");
    CAN0.setMode(MCP_NORMAL);
  }
  else
  {
    Serial.println("CAN0: Init Fail!");
  }

  Pre_millis = millis();
}

void loop()
{
  double motor_ouput_current_A = 1.0;

  // 0~20Aを0~16384の2バイト値に変換
  int16_t motor_ouput_current_Byte = fmap(motor_ouput_current_A, 0, 20, 0, 16384);

  txBuf[0] = (motor_ouput_current_Byte >> 8) & 0xFF;  // 上位バイト
  txBuf[1] = motor_ouput_current_Byte & 0xFF;         // 下位バイト
  // 残り6バイトは0のまま

  // 20ms周期で送信
  if (millis() - Pre_millis > 20)
  {
    byte sendResult = CAN0.sendMsgBuf(0x200, 0, 8, txBuf);
    if (sendResult == CAN_OK)
      Serial.println("CAN send success");
    else
      Serial.println("CAN send fail");

    Pre_millis = millis();
  }

  // 受信処理
  if (CAN0.checkReceive() == CAN_MSGAVAIL)
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    Serial.print("Receive ID: 0x");
    Serial.print(rxId, HEX);
    Serial.print(" Data: ");

    if (len >= 7)
    {
      int16_t angle = (rxBuf[0] << 8) | rxBuf[1];
      int16_t rpm = (rxBuf[2] << 8) | rxBuf[3];
      int16_t amp = (rxBuf[4] << 8) | rxBuf[5];
      int8_t temp = rxBuf[6];

      Serial.print(angle);
      Serial.print(", ");
      Serial.print(rpm);
      Serial.print(", ");
      Serial.print(amp);
      Serial.print(", ");
      Serial.println(temp);
    }
    else
    {
      Serial.println("Invalid data length");
    }
  }
}

int16_t fmap(double x, double in_min, double in_max, int16_t out_min, int16_t out_max)
{
  return (int16_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
