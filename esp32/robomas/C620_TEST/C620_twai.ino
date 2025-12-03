#include <Arduino.h>
#include "driver/twai.h"
#include "driver/gpio.h"

// =============================
// ピン設定
// =============================
const gpio_num_t CAN_TX = GPIO_NUM_4; // 元コードの TX=4
const gpio_num_t CAN_RX = GPIO_NUM_2; // 元コードの RX=2

// =============================
// 定数
// =============================
#define NUM_MOTOR 4
#define AMP_MAX 20.0f
const int ENCODER_MAX = 8192;
const int HALF_ENCODER = ENCODER_MAX / 2;
constexpr float gear_m3508 = 19.2f;

int encoder_count[NUM_MOTOR] = {0};
int rpm[NUM_MOTOR] = {0};
int current[NUM_MOTOR] = {0};
int encoder_offset[NUM_MOTOR] = {0};
int rotation_count[NUM_MOTOR] = {0};
int last_encoder[NUM_MOTOR] = {0};
long total_encoder[NUM_MOTOR] = {0};
float angle_m3508[NUM_MOTOR] = {0};
float vel_m3508[NUM_MOTOR] = {0};
float c[NUM_MOTOR] = {0};

// PID関連
float target_rpm[NUM_MOTOR] = {0};
float vel_error_prev[NUM_MOTOR] = {0};
float vel_prop_prev[NUM_MOTOR] = {0};
float vel_output[NUM_MOTOR] = {0};
float vel_out[NUM_MOTOR] = {0};
float motor_output_current[NUM_MOTOR] = {0};

unsigned long lastPidTime = 0;

// ------------PIDゲイン-----------
float kp_vel = 0.5;
float ki_vel = 0.0;
float kd_vel = 0.1;

float pid_vel(float setpoint, float input, float &error_prev, float &prop_prev, float &output,
              float kp, float ki, float kd, float dt)
{
    float error = setpoint - input;
    float prop = error - error_prev;
    float deriv = prop - prop_prev;
    float du = kp * prop + ki * error * dt + kd * deriv;
    output += du;

    prop_prev = prop;
    error_prev = error;

    return output;
}

float constrain_double(float val, float min_val, float max_val)
{
    if (val < min_val)
        return min_val;
    if (val > max_val)
        return max_val;
    return val;
}

// =============================
// TWAI送信: M2006/M3508用 4モータ電流送信
// =============================
void send_cur_all(float cur_array[NUM_MOTOR])
{
    twai_message_t tx;       // 送信用メッセージ
    tx.identifier = 0x200;   // CAN ID
    tx.extd = 0;             // 標準フレーム
    tx.rtr = 0;              // データフレーム
    tx.data_length_code = 8; // 8バイト

    // C620 の仕様: -16384 ～ +16384
    for (int i = 0; i < NUM_MOTOR; i++)
    {
        float amp = constrain_double(cur_array[i], -20, 20);
        int16_t val = amp * (16384.0f / 200.0f);

        tx.data[i * 2] = (val >> 8) & 0xFF;
        tx.data[i * 2 + 1] = val & 0xFF;
    }

    if (twai_transmit(&tx, pdMS_TO_TICKS(20)) != ESP_OK)
    {
        Serial.println("[ERR] twai_transmit failed");
    }
}

// =============================
// TWAI受信処理（C610/C620フィードバック）
// =============================
void twai_receive_feedback()
{
    twai_message_t rx_msg;

    while (twai_receive(&rx_msg, 0) == ESP_OK)
    {
        if (rx_msg.data_length_code != 8)
            continue;
        if (rx_msg.identifier < 0x201 || rx_msg.identifier > 0x204)
            continue;

        int m = rx_msg.identifier - 0x201;

        encoder_count[m] = (int16_t)(rx_msg.data[0] << 8 | rx_msg.data[1]);
        rpm[m] = (int16_t)(rx_msg.data[2] << 8 | rx_msg.data[3]);
        current[m] = (int16_t)(rx_msg.data[4] << 8 | rx_msg.data[5]);

        // エンコーダ回転数計算
        int diff = encoder_count[m] - last_encoder[m];
        if (diff > HALF_ENCODER)
            rotation_count[m]--;
        else if (diff < -HALF_ENCODER)
            rotation_count[m]++;

        last_encoder[m] = encoder_count[m];

        total_encoder[m] =
            rotation_count[m] * ENCODER_MAX + encoder_count[m];

        angle_m3508[m] = total_encoder[m] * (360.0f / (ENCODER_MAX * gear_m3508));
        vel_m3508[m] = rpm[m] / gear_m3508;
        c[m] = current[m] * 20.0f / 16384.0f;
    }
}

// =============================
// TWAI初期化
// =============================
void setup()
{
    Serial.begin(115200);
    delay(1000);

    // TWAI 設定
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1000KBITS(); // 元コードは 1Mbps
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    {
        Serial.println("TWAI install failed");
        while (1)
            ;
    }
    if (twai_start() != ESP_OK)
    {
        Serial.println("TWAI start failed");
        while (1)
            ;
    }

    Serial.println("TWAI started");
}

void loop()
{
    unsigned long now = millis();
    float dt = (now - lastPidTime) / 1000.0f;
    if (dt <= 0)
        dt = 0.000001f;
    if (dt > 0.02f)
        dt = 0.02f;
    lastPidTime = now;

    // TWAI受信処理
    twai_receive_feedback();

    // 5秒後にターゲットRPM増加
    for (int i = 0; i < NUM_MOTOR; i++)
    {
        static float rpm_step[NUM_MOTOR] = {100, 100, 100, 100};

        if (now < 5000)
        {
            target_rpm[i] = 0;
        }
        else
        {
            if (rpm_step[i] < 300)
                rpm_step[i] += 0.1;
            target_rpm[i] = rpm_step[i];
        }

        vel_out[i] = pid_vel(target_rpm[i], vel_m3508[i],
                             vel_error_prev[i], vel_prop_prev[i], vel_output[i],
                             kp_vel, ki_vel, kd_vel, dt);

        motor_output_current[i] = constrain_double(vel_out[i] * 10, -20, 20);
    }

    // 送信
    send_cur_all(motor_output_current);

    // debug
    Serial.print(vel_m3508[0]);
    Serial.print("\t");
    Serial.println(current[0]);

    delay(1);
}
