#include <Arduino.h>
#include <output_task.h>
#include <defs.h>
#include <esp32-hal-ledc.h>

// 受信データ格納用
extern int32_t received_data[MAX_ARRAY_SIZE]; // 受信データ

void MD_Output_Task(void *pvParameters) {
    while (1) {

        // MD出力の制限
        received_data[1] = constrain(received_data[1], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[2] = constrain(received_data[2], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[3] = constrain(received_data[3], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[4] = constrain(received_data[4], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[5] = constrain(received_data[5], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[6] = constrain(received_data[6], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[7] = constrain(received_data[7], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[8] = constrain(received_data[8], -MD_PWM_MAX, MD_PWM_MAX);

        // ピンの操作
        digitalWrite(MD1D, received_data[1] > 0 ? HIGH : LOW);
        digitalWrite(MD2D, received_data[2] > 0 ? HIGH : LOW);
        digitalWrite(MD3D, received_data[3] > 0 ? HIGH : LOW);
        digitalWrite(MD4D, received_data[4] > 0 ? HIGH : LOW);
        digitalWrite(MD5D, received_data[5] > 0 ? HIGH : LOW);
        digitalWrite(MD6D, received_data[6] > 0 ? HIGH : LOW);
        digitalWrite(MD7D, received_data[7] > 0 ? HIGH : LOW);
        digitalWrite(MD8D, received_data[8] > 0 ? HIGH : LOW);

        ledcWrite(MD1P, abs(received_data[1]));
        ledcWrite(MD2P, abs(received_data[2]));
        ledcWrite(MD3P, abs(received_data[3]));
        ledcWrite(MD4P, abs(received_data[4]));
        ledcWrite(MD5P, abs(received_data[5]));
        ledcWrite(MD6P, abs(received_data[6]));
        ledcWrite(MD7P, abs(received_data[7]));
        ledcWrite(MD8P, abs(received_data[8]));

        vTaskDelay(1); // WDTのリセット(必須)
    }
}

void Servo_Output_Task(void *pvParameters) {
    // for文で書きかえたいところ、、、
    while (1) {
        // サーボ1
        int angle1 = received_data[9];
        if (angle1 < SERVO1_MIN_DEG)
            angle1 = SERVO1_MIN_DEG;
        if (angle1 > SERVO1_MAX_DEG)
            angle1 = SERVO1_MAX_DEG;
        int us1 = map(angle1, SERVO1_MIN_DEG, SERVO1_MAX_DEG, SERVO1_MIN_US, SERVO1_MAX_US);
        int duty1 = (int)(us1 * SERVO_PWM_SCALE);
        ledcWrite(SERVO1, duty1);

        // サーボ2
        int angle2 = received_data[10];
        if (angle2 < SERVO2_MIN_DEG)
            angle2 = SERVO2_MIN_DEG;
        if (angle2 > SERVO2_MAX_DEG)
            angle2 = SERVO2_MAX_DEG;
        int us2 = map(angle2, SERVO2_MIN_DEG, SERVO2_MAX_DEG, SERVO2_MIN_US, SERVO2_MAX_US);
        int duty2 = (int)(us2 * SERVO_PWM_SCALE);
        ledcWrite(SERVO2, duty2);

        // サーボ3
        int angle3 = received_data[11];
        if (angle3 < SERVO3_MIN_DEG)
            angle3 = SERVO3_MIN_DEG;
        if (angle3 > SERVO3_MAX_DEG)
            angle3 = SERVO3_MAX_DEG;
        int us3 = map(angle3, SERVO3_MIN_DEG, SERVO3_MAX_DEG, SERVO3_MIN_US, SERVO3_MAX_US);
        int duty3 = (int)(us3 * SERVO_PWM_SCALE);
        ledcWrite(SERVO3, duty3);

        // サーボ4
        int angle4 = received_data[12];
        if (angle4 < SERVO4_MIN_DEG)
            angle4 = SERVO4_MIN_DEG;
        if (angle4 > SERVO4_MAX_DEG)
            angle4 = SERVO4_MAX_DEG;
        int us4 = map(angle4, SERVO4_MIN_DEG, SERVO4_MAX_DEG, SERVO4_MIN_US, SERVO4_MAX_US);
        int duty4 = (int)(us4 * SERVO_PWM_SCALE);
        ledcWrite(SERVO4, duty4);

        // サーボ5
        int angle5 = received_data[13];
        if (angle5 < SERVO5_MIN_DEG)
            angle5 = SERVO5_MIN_DEG;
        if (angle5 > SERVO5_MAX_DEG)
            angle5 = SERVO5_MAX_DEG;
        int us5 = map(angle5, SERVO5_MIN_DEG, SERVO5_MAX_DEG, SERVO5_MIN_US, SERVO5_MAX_US);
        int duty5 = (int)(us5 * SERVO_PWM_SCALE);
        ledcWrite(SERVO5, duty5);

        // サーボ6
        int angle6 = received_data[14];
        if (angle6 < SERVO6_MIN_DEG)
            angle6 = SERVO6_MIN_DEG;
        if (angle6 > SERVO6_MAX_DEG)
            angle6 = SERVO6_MAX_DEG;
        int us6 = map(angle6, SERVO6_MIN_DEG, SERVO6_MAX_DEG, SERVO6_MIN_US, SERVO6_MAX_US);
        int duty6 = (int)(us6 * SERVO_PWM_SCALE);
        ledcWrite(SERVO6, duty6);

        // サーボ7
        int angle7 = received_data[15];
        if (angle7 < SERVO7_MIN_DEG)
            angle7 = SERVO7_MIN_DEG;
        if (angle7 > SERVO7_MAX_DEG)
            angle7 = SERVO7_MAX_DEG;
        int us7 = map(angle7, SERVO7_MIN_DEG, SERVO7_MAX_DEG, SERVO7_MIN_US, SERVO7_MAX_US);
        int duty7 = (int)(us7 * SERVO_PWM_SCALE);
        ledcWrite(SERVO7, duty7);

        // サーボ8
        int angle8 = received_data[16];
        if (angle8 < SERVO8_MIN_DEG)
            angle8 = SERVO8_MIN_DEG;
        if (angle8 > SERVO8_MAX_DEG)
            angle8 = SERVO8_MAX_DEG;
        int us8 = map(angle8, SERVO8_MIN_DEG, SERVO8_MAX_DEG, SERVO8_MIN_US, SERVO8_MAX_US);
        int duty8 = (int)(us8 * SERVO_PWM_SCALE);
        ledcWrite(SERVO8, duty8);

        vTaskDelay(1); // WDTのリセット(必須)
    }
}

void SV_Task(void *pvParameters) {
    while (1) {

        digitalWrite(SV1, received_data[17] ? HIGH : LOW);
        digitalWrite(SV2, received_data[18] ? HIGH : LOW);
        digitalWrite(SV3, received_data[19] ? HIGH : LOW);
        digitalWrite(SV4, received_data[20] ? HIGH : LOW);
        digitalWrite(SV5, received_data[21] ? HIGH : LOW);
        digitalWrite(SV6, received_data[22] ? HIGH : LOW);
        digitalWrite(SV7, received_data[23] ? HIGH : LOW);

        vTaskDelay(1); // WDTのリセット(必須)
    }
}

void LED_Blink100_Task(void *pvParameters) {
    while (1) {
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        delay(100);
    }
}

// ledcWrite,vTaskDelayに書き換え予定
void LED_PWM_Task(void *pvParameters) {
    while (1) {
        for (int val = 0; val <= 255; val++) {
            analogWrite(LED, val);
            delay(5);
        }
        for (int val = 255; val >= 0; val--) {
            analogWrite(LED, val);
            delay(5);
        }
    }
}