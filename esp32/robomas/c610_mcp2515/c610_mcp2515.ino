#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
byte txBuf[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
long Pre_millis;
MCP_CAN CAN0(4); //megaのときは53にする

int16_t fmap(double x, double in_min, double in_max, int16_t out_min, int16_t out_max);

void setup()
{
  Serial.begin(115200);
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
  double motor_ouput_current_A = 0.0;
  int16_t motor_ouput_current_Byte = fmap(motor_ouput_current_A, 0, 20, 0, 16384);//2バイトに変換
  txBuf[0] = (motor_ouput_current_Byte >> 8) & 0xFF;//上位バイト
  txBuf[1] = motor_ouput_current_Byte & 0xFF;//下位バイト

  //Send
  if (millis() - Pre_millis > 20) { // Period: 20ms
    CAN0.sendMsgBuf(0x200, 0, 8, txBuf);
    Pre_millis = millis();
  }

  //Receive
  if (CAN0.checkReceive() == CAN_MSGAVAIL)
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    Serial.print("Recive ID: ");
    Serial.print(rxId, HEX);
    Serial.print(" Data: ");

    int16_t angle = rxBuf[0] << 8 | rxBuf[1];
    int16_t rpm = rxBuf[2] << 8 | rxBuf[3];
    int16_t amp = rxBuf[4] << 8 | rxBuf[5];
    int8_t  temp = rxBuf[6];

    Serial.print(angle);
    Serial.print(",");
    Serial.print(rpm);
    Serial.print(",");
    Serial.print(amp);
    Serial.print(",");
    Serial.println(temp);
  }
}

int16_t fmap(double x, double in_min, double in_max, int16_t out_min, int16_t out_max) {
  return (x - in_min) * ((double)(out_max - out_min)) / (in_max - in_min) + out_min;
}
