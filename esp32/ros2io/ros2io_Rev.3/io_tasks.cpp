#include <io_tasks.h>
#include <pin_defs.h>

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

        vTaskDelay(1); // ウォッチドッグタイマのリセット(必須)
    }
}
