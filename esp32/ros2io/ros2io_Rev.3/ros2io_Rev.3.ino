
/*<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
ros2io Rev.3
Copyright © 2025 RRST-NHK-Project. All rights reserved.
<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>*/

/*
For NHK-Robocon-2026
ESP32用microROSプログラム。ROSメッセージからマイコンのIOを操作する。
ライブラリはros2io/librariesに格納。修正済みのライブラリを使用しているため同梱のZipを使用すること。
ボードマネージャーはesp32 by Espressif Systemsを選択
*/

/*
TODO:ROSからのモード選択機能の実装（保留中）
TODO:定数定義の統一
FIXME:Pub、Subの同時使用時の遅延問題
*/

#include <Arduino.h>
#include <CAN.h>
#include <esp32-hal-ledc.h>

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
// **複数のESPを使用する場合はIDを変更** //
#define ID 0
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
// **使用する基板に合わせてモードを変更** //
#define MODE 6
/*
0:基板テスト用（ROSと接続せずに基板のテストのみを行う）※実機で「絶対」に実行しないこと　※テストモードについては下記参照
1:MD専用
2:センサ類（エンコーダー・スイッチ・C610からのフィードバック）
3:その他アクチュエータ（サーボ・ソレノイドバルブ）
4:サーボ・ソレノイドバルブ・スイッチ＋エンコーダー（Pub,Subを同時にしたときの遅延問題が解決できていないため未実装）
5:ロボマス制御
*/
// **テスト内容変更** //
#define TEST_MODE 0
/*
0:何もせず待機
1:MDテスト
2:ENC/SWテスト
3:SERVO/SVテスト
4:未実装
5:CANテスト
*/
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
//**テストモード(MODE 0)について** //
/*
テストモードはROSと接続せずに基板のテストやデバッグを行うためのモードです。
MODEを0に変更することで有効化され、TEST_MODEを変更することでテスト内容を変更します。
書き込み後シリアルモニターでEnterキーを押すとテストモードの動作が開始します。
テスト内容によっては機体の暴走や破損につながるので機体搭載時には「絶対」に実行しないでください。
*/
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
// ROSとの接続にはエージェントが必要です。下記コマンドでエージェント立ち上げを行ってください（Dockerのインストールが必要）。USBポートは適宜変更すること。
// sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -v6
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

// microROS関連
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32_multi_array.h>

//  パルスカウンタ関連
#include "driver/pcnt.h"

// 　自作ライブラリ（関数・定数をまとめてる）
#include "Output_Task.h"
#include "pin_defs.h"

// ********* CAN関連 ********* //

// -------- 定数 -------- //
const int ENCODER_MAX = 8192;             // エンコーダの最大値
const int HALF_ENCODER = ENCODER_MAX / 2; // エンコーダ半回転値
constexpr float gear_ratio = 36.0f;       // 減速比
const int NUM_MOTORS = 4;                 // モータ数
float current_limit_A = 10.0f;

// -------- 状態量 / CAN受信関連 -------- //
int encoders[NUM_MOTORS] = {0};       // エンコーダ値
int rpms[NUM_MOTORS] = {0};           // 回転速度
int currents[NUM_MOTORS] = {0};       // 電流値
bool offset_ok[NUM_MOTORS] = {false}; // オフセット完了フラグ
int encoder_offset[NUM_MOTORS] = {0}; // エンコーダオフセット
int last_encoder[NUM_MOTORS];         // 前回エンコーダ値
int rotation_count[NUM_MOTORS] = {0}; // 回転数
long total_encoder[NUM_MOTORS] = {0}; // 累積エンコーダ値
float angles[NUM_MOTORS] = {0};       // 角度
float vels[NUM_MOTORS] = {0};         // 速度

// -------- PID関連変数 -------- //
float target_angle[NUM_MOTORS] = {0};         // 目標角度
float pos_error_prev[NUM_MOTORS] = {0};       // 前回角度誤差
float pos_integral[NUM_MOTORS] = {0};         // 角度積分項
float pos_output[NUM_MOTORS] = {0};           // PID出力
float motor_output_current[NUM_MOTORS] = {0}; // 出力電流

unsigned long lastPidTime = 0; // PID制御用タイマー

// -------- PIDゲイン -------- //
float kp_pos = 0.8f;  // 角度比例ゲイン
float ki_pos = 0.01f; // 角度積分ゲイン
float kd_pos = 0.02f; // 角度微分ゲイン

// 複数モータ対応CAN送信関数
void send_cur_all(float cur_array[NUM_MOTORS]) {
    constexpr float MAX_CUR = 10;
    constexpr int MAX_CUR_VAL = 10000;
    uint8_t send_data[8] = {};

    for (int i = 0; i < NUM_MOTORS; i++) {
        float val = cur_array[i] * (MAX_CUR_VAL / MAX_CUR);
        if (val < -MAX_CUR_VAL)
            val = -MAX_CUR_VAL;
        if (val > MAX_CUR_VAL)
            val = MAX_CUR_VAL;

        int16_t transmit_val = val;
        send_data[i * 2] = (transmit_val >> 8) & 0xFF;
        send_data[i * 2 + 1] = transmit_val & 0xFF;
    }

    CAN.beginPacket(0x200);
    CAN.write(send_data, 8);
    CAN.endPacket();
}

// PID計算関数（単独モータ用だが複数モータでループ使用可能）
float pid(float setpoint, float input, float &error_prev, float &integral,
          float kp, float ki, float kd, float dt) {
    float error = setpoint - input;
    integral += ((error + error_prev) * dt / 2.0f); // 台形積分
    float derivative = (error - error_prev) / dt;
    error_prev = error;
    return kp * error + ki * integral + kd * derivative;
}

// 値制限関数
float constrain_double(float val, float min_val, float max_val) {
    if (val < min_val)
        return min_val;
    if (val > max_val)
        return max_val;
    return val;
}

// ********* CAN関連ここまで ********* //

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
// rcl_timer_t timer;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ノード名とトピック名の定義（ID付き）
String node_name = "esp32node_" + String(ID, DEC);
String publisher_topic_name = "from_esp32_" + String(ID, DEC);
String subscriber_topic_name = "to_esp32_" + String(ID, DEC);

// 受信データ格納用のバッファ
int32_t buffer[MAX_ARRAY_SIZE];

// 受信データ格納用
int32_t received_data[MAX_ARRAY_SIZE]; // 受信データ //2025/09/29: volatileを削除
size_t received_size = 0;              // 受信データのサイズ //2025/09/29: volatileを削除

// エンコーダのカウント格納用
int16_t count[4] = {0};

// スイッチの状態格納用
bool sw_state[4] = {false};

// 詳細デバッグ
#define RCCHECK(fn)                                                                           \
    do {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                               \
        if ((temp_rc) != RCL_RET_OK) {                                                        \
            /* デバッグ出力 */                                                                \
            /* SerialBT.printf("RCL error at %s:%d -> %d\n", __FILE__, __LINE__, temp_rc); */ \
            error_loop();                                                                     \
        }                                                                                     \
    } while (0)

// エラー発生時のループ
void error_loop() {
    while (1) {
        // このエラーが表示される場合は、シリアルモニターを閉じているか確認
        delay(1000);
    }
}

// コールバック内でグローバル変数にコピー
void subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;
    size_t len = msg->data.size;
    if (len > MAX_ARRAY_SIZE)
        len = MAX_ARRAY_SIZE;

    for (size_t i = 0; i < len; i++) {
        received_data[i] = msg->data.data[i];
    }
    received_size = len;
}

// ===== タスクハンドルのグローバル変数 =====
TaskHandle_t led_blink100_handle = NULL;
TaskHandle_t led_pwm_handle = NULL;

void setup() {

    pinMode(LED, OUTPUT);

    // MODEに応じた初期化
    switch (MODE) {
    case 0:
        Serial.begin(115200);
        mode0_init();
        break;
    case 1:
        ros_init();
        mode1_init();
        break;
    case 2:
        ros_init();
        mode2_init();
        break;
    case 3:
        ros_init();
        mode3_init();
        break;
    case 4:
        ros_init();
        mode4_init();
        break;
    case 5:
        ros_init();
        mode5_init();
        break;
    default:;
        ;
    }
}

void loop() {
    if (MODE != 0) {
        RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
        vTaskDelay(1);
    }
}

// micro-ROSの初期化
void ros_init() {
    delay(2000);

    set_microros_transports();
    allocator = rcl_get_default_allocator();

    xTaskCreateUniversal(
        LED_Blink100_Task,
        "LED_Blink100_Task",
        2048,
        NULL,
        1, // 優先度、最大25？
        &led_blink100_handle,
        APP_CPU_NUM);

    // Agentと接続できるまでリトライ
    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        delay(1000); // 1秒待つ
    }

    // Nodeの初期化
    RCCHECK(rclc_node_init_default(&node, node_name.c_str(), "", &support));

    // Subscriberの初期化
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        subscriber_topic_name.c_str()));

    // Publisherの初期化
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        publisher_topic_name.c_str()));

    std_msgs__msg__Int32MultiArray__init(&msg);
    msg.data.data = buffer;
    msg.data.size = 0;
    msg.data.capacity = MAX_ARRAY_SIZE;

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // 以下のサービスの数でexecutorのサイズを変える。

    // Executorにサービスを追加
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    vTaskDelete(led_blink100_handle);
    led_blink100_handle = NULL;
}

void ENC_SW_Read_Publish_Task(void *pvParameters) {
    while (1) {

        // パルスカウンタの値を取得
        pcnt_get_counter_value(PCNT_UNIT_0, &count[0]);
        pcnt_get_counter_value(PCNT_UNIT_1, &count[1]);
        pcnt_get_counter_value(PCNT_UNIT_2, &count[2]);
        pcnt_get_counter_value(PCNT_UNIT_3, &count[3]);

        // スイッチの状態を取得
        sw_state[0] = (digitalRead(SW1) == HIGH);
        sw_state[1] = (digitalRead(SW2) == HIGH);
        sw_state[2] = (digitalRead(SW3) == HIGH);
        sw_state[3] = (digitalRead(SW4) == HIGH);

        msg.data.data[0] = count[0];
        msg.data.data[1] = count[1];
        msg.data.data[2] = count[2];
        msg.data.data[3] = count[3];
        msg.data.data[4] = sw_state[0];
        msg.data.data[5] = sw_state[1];
        msg.data.data[6] = sw_state[2];
        msg.data.data[7] = sw_state[3];

        // Publish
        if (MODE != 0) {
            RCCHECK(rcl_publish(&publisher, &msg, NULL));
        }

        vTaskDelay(1); // ウォッチドッグタイマのリセット(必須)
    }
}

void ROBOMAS_ENC_SW_Read_Publish_Task(void *pvParameters) {
    // 初期化
    for (int i = 0; i < NUM_MOTORS; i++) {
        last_encoder[i] = -1;
    }
    while (1) {

        // パルスカウンタの値を取得
        pcnt_get_counter_value(PCNT_UNIT_0, &count[0]);
        pcnt_get_counter_value(PCNT_UNIT_1, &count[1]);
        pcnt_get_counter_value(PCNT_UNIT_2, &count[2]);
        pcnt_get_counter_value(PCNT_UNIT_3, &count[3]);

        // スイッチの状態を取得
        sw_state[0] = (digitalRead(SW1) == HIGH);
        sw_state[1] = (digitalRead(SW2) == HIGH);
        sw_state[2] = (digitalRead(SW3) == HIGH);
        sw_state[3] = (digitalRead(SW4) == HIGH);

        // ----一旦応急処置---- //
        // 2025/09/18 gptのコードを、gptを使って読みやすく書き換えました。そのうち書き直す、、、、はず

        // CAN受信
        int packetSize = CAN.parsePacket();
        while (packetSize) {
            int id = CAN.packetId();

            if (id >= 0x201 && id < 0x201 + NUM_MOTORS) {
                int idx = id - 0x201;
                uint8_t rx[8];
                for (int i = 0; i < 8; i++)
                    rx[i] = CAN.read();

                encoders[idx] = (rx[0] << 8) | rx[1];
                rpms[idx] = (rx[2] << 8) | rx[3];
                currents[idx] = (rx[4] << 8) | rx[5];

                // 初回オフセット設定
                if (!offset_ok[idx]) {
                    encoder_offset[idx] = encoders[idx];
                    last_encoder[idx] = -1;
                    rotation_count[idx] = 0;
                    total_encoder[idx] = 0;
                    offset_ok[idx] = true;
                }

                int enc_rel = encoders[idx] - encoder_offset[idx];
                if (enc_rel < 0)
                    enc_rel += ENCODER_MAX;

                if (last_encoder[idx] != -1) {
                    int diff = encoders[idx] - last_encoder[idx];
                    if (diff > HALF_ENCODER)
                        rotation_count[idx]--;
                    else if (diff < -HALF_ENCODER)
                        rotation_count[idx]++;
                }

                last_encoder[idx] = encoders[idx];
                total_encoder[idx] = rotation_count[idx] * ENCODER_MAX + encoders[idx];
                angles[idx] = total_encoder[idx] * (360.0 / (8192.0 * gear_ratio));
                vels[idx] = rpms[idx] / gear_ratio;
            }

            packetSize = CAN.parsePacket();
        }

        msg.data.data[0] = count[0];
        msg.data.data[1] = count[1];
        msg.data.data[2] = count[2];
        msg.data.data[3] = count[3];
        msg.data.data[4] = sw_state[0];
        msg.data.data[5] = sw_state[1];
        msg.data.data[6] = sw_state[2];
        msg.data.data[7] = sw_state[3];
        msg.data.data[8] = angles[0];
        msg.data.data[9] = angles[1];
        msg.data.data[10] = angles[2];
        msg.data.data[11] = angles[3];
        msg.data.data[12] = rpms[0];
        msg.data.data[13] = rpms[1];
        msg.data.data[14] = rpms[2];
        msg.data.data[15] = rpms[3];
        msg.data.data[16] = currents[0];
        msg.data.data[17] = currents[1];
        msg.data.data[18] = currents[2];
        msg.data.data[19] = currents[3];
        // ----応急処置ここまで---- //

        // msg.data.data[0] = count[0];
        // msg.data.data[1] = count[1];
        // msg.data.data[2] = count[2];
        // msg.data.data[3] = count[3];
        // msg.data.data[4] = sw_state[0];
        // msg.data.data[5] = sw_state[1];
        // msg.data.data[6] = sw_state[2];
        // msg.data.data[7] = sw_state[3];

        // Publish
        if (MODE != 0) {
            RCCHECK(rcl_publish(&publisher, &msg, NULL));
        }

        vTaskDelay(1); // ウォッチドッグタイマのリセット(必須)
    }
}

void C610_Task(void *pvParameters) {
    while (1) {
        unsigned long now = millis();
        float dt = (now - lastPidTime) / 1000.0f;
        if (dt <= 0)
            dt = 0.000001f; // dtが0にならないよう補正
        lastPidTime = now;

        // -------- 目標角度の更新 -------- //
        // received_data[1]～[4] にモータ1～4の目標角度が入っている前提
        for (int i = 0; i < NUM_MOTORS; i++) {
            target_angle[i] = received_data[i + 1];
        }

        // -------- CAN受信処理 -------- //
        int packetSize = CAN.parsePacket();
        while (packetSize) {
            int id = CAN.packetId();
            if (id >= 0x201 && id < 0x201 + NUM_MOTORS) {
                int motor_index = id - 0x201;
                uint8_t rx[8];
                for (int i = 0; i < 8; i++)
                    rx[i] = CAN.read();

                // エンコーダ・速度・電流取得
                encoders[motor_index] = (rx[0] << 8) | rx[1];
                rpms[motor_index] = (rx[2] << 8) | rx[3];
                currents[motor_index] = (rx[4] << 8) | rx[5];

                // 初回オフセット設定
                if (!offset_ok[motor_index]) {
                    encoder_offset[motor_index] = encoders[motor_index];
                    last_encoder[motor_index] = -1;
                    rotation_count[motor_index] = 0;
                    total_encoder[motor_index] = 0;
                    pos_integral[motor_index] = 0;
                    pos_error_prev[motor_index] = 0;
                    offset_ok[motor_index] = true;
                }

                // エンコーダ差分とラップ補正
                int enc_relative = encoders[motor_index] - encoder_offset[motor_index];
                if (enc_relative < 0)
                    enc_relative += ENCODER_MAX;

                if (last_encoder[motor_index] != -1) {
                    int diff = encoders[motor_index] - last_encoder[motor_index];
                    if (diff > HALF_ENCODER)
                        rotation_count[motor_index]--;
                    else if (diff < -HALF_ENCODER)
                        rotation_count[motor_index]++;
                }

                last_encoder[motor_index] = encoders[motor_index];
                total_encoder[motor_index] = rotation_count[motor_index] * ENCODER_MAX + encoders[motor_index];
                angles[motor_index] = total_encoder[motor_index] * (360.0f / (ENCODER_MAX * gear_ratio));
                vels[motor_index] = (rpms[motor_index] / gear_ratio) * 360.0f / 60.0f;
            }

            packetSize = CAN.parsePacket(); // 次の受信も処理
        }

        // -------- PID制御（全モータ） -------- //
        for (int i = 0; i < NUM_MOTORS; i++) {
            pos_output[i] = pid(target_angle[i], angles[i], pos_error_prev[i], pos_integral[i],
                                kp_pos, ki_pos, kd_pos, dt);
            motor_output_current[i] = constrain_double(pos_output[i], -current_limit_A, current_limit_A);
        }

        // -------- CAN送信（全モータ） -------- //
        send_cur_all(motor_output_current);

        delay(1);
    }
}

void C610_FB_Task(void *pvParameters) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        last_encoder[i] = -1;
    }
    while (1) {
        unsigned long now = millis();
        float dt = (now - lastPidTime) / 1000.0f;
        if (dt <= 0)
            dt = 0.000001f; // dtが0にならないよう補正
        lastPidTime = now;

        // -------- 目標角度の更新 -------- //
        // received_data[1]～[4] にモータ1～4の目標角度が入っている前提
        for (int i = 0; i < NUM_MOTORS; i++) {
            target_angle[i] = received_data[i + 1];
        }

        // -------- CAN受信処理 -------- //
        int packetSize = CAN.parsePacket();
        while (packetSize) {
            int id = CAN.packetId();
            if (id >= 0x201 && id < 0x201 + NUM_MOTORS) {
                int motor_index = id - 0x201;
                uint8_t rx[8];
                for (int i = 0; i < 8; i++)
                    rx[i] = CAN.read();

                // エンコーダ・速度・電流取得
                encoders[motor_index] = (rx[0] << 8) | rx[1];
                rpms[motor_index] = (rx[2] << 8) | rx[3];
                currents[motor_index] = (rx[4] << 8) | rx[5];

                // 初回オフセット設定
                if (!offset_ok[motor_index]) {
                    encoder_offset[motor_index] = encoders[motor_index];
                    last_encoder[motor_index] = -1;
                    rotation_count[motor_index] = 0;
                    total_encoder[motor_index] = 0;
                    pos_integral[motor_index] = 0;
                    pos_error_prev[motor_index] = 0;
                    offset_ok[motor_index] = true;
                }

                // エンコーダ差分とラップ補正
                int enc_relative = encoders[motor_index] - encoder_offset[motor_index];
                if (enc_relative < 0)
                    enc_relative += ENCODER_MAX;

                if (last_encoder[motor_index] != -1) {
                    int diff = encoders[motor_index] - last_encoder[motor_index];
                    if (diff > HALF_ENCODER)
                        rotation_count[motor_index]--;
                    else if (diff < -HALF_ENCODER)
                        rotation_count[motor_index]++;
                }

                last_encoder[motor_index] = encoders[motor_index];
                total_encoder[motor_index] = rotation_count[motor_index] * ENCODER_MAX + encoders[motor_index];
                angles[motor_index] = total_encoder[motor_index] * (360.0f / (ENCODER_MAX * gear_ratio));
                vels[motor_index] = (rpms[motor_index] / gear_ratio) * 360.0f / 60.0f;
            }

            packetSize = CAN.parsePacket(); // 次の受信も処理
        }

        // -------- PID制御（全モータ） -------- //
        for (int i = 0; i < NUM_MOTORS; i++) {
            pos_output[i] = pid(target_angle[i], angles[i], pos_error_prev[i], pos_integral[i],
                                kp_pos, ki_pos, kd_pos, dt);
            motor_output_current[i] = constrain_double(pos_output[i], -current_limit_A, current_limit_A);
        }

        // -------- CAN送信（全モータ） -------- //
        send_cur_all(motor_output_current);

        msg.data.data[0] = count[0];
        msg.data.data[1] = count[1];
        msg.data.data[2] = count[2];
        msg.data.data[3] = count[3];
        msg.data.data[4] = sw_state[0];
        msg.data.data[5] = sw_state[1];
        msg.data.data[6] = sw_state[2];
        msg.data.data[7] = sw_state[3];
        msg.data.data[8] = angles[0];
        msg.data.data[9] = angles[1];
        msg.data.data[10] = angles[2];
        msg.data.data[11] = angles[3];
        msg.data.data[12] = rpms[0];
        msg.data.data[13] = rpms[1];
        msg.data.data[14] = rpms[2];
        msg.data.data[15] = rpms[3];
        msg.data.data[16] = currents[0];
        msg.data.data[17] = currents[1];
        msg.data.data[18] = currents[2];
        msg.data.data[19] = currents[3];

        delay(1);
    }
}

void enc_init() {
    // プルアップを有効化
    gpio_set_pull_mode((gpio_num_t)ENC1_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC1_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC2_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC2_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC3_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC3_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC4_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)ENC4_B, GPIO_PULLUP_ONLY);

    // パルスカウンタの設定
    pcnt_config_t pcnt_config1 = {};
    pcnt_config1.pulse_gpio_num = ENC1_A;
    pcnt_config1.ctrl_gpio_num = ENC1_B;
    pcnt_config1.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config1.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config1.pos_mode = PCNT_COUNT_INC;
    pcnt_config1.neg_mode = PCNT_COUNT_DEC;
    pcnt_config1.counter_h_lim = COUNTER_H_LIM;
    pcnt_config1.counter_l_lim = COUNTER_L_LIM;
    pcnt_config1.unit = PCNT_UNIT_0;
    pcnt_config1.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config2 = {};
    pcnt_config2.pulse_gpio_num = ENC1_B;
    pcnt_config2.ctrl_gpio_num = ENC1_A;
    pcnt_config2.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config2.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config2.pos_mode = PCNT_COUNT_INC;
    pcnt_config2.neg_mode = PCNT_COUNT_DEC;
    pcnt_config2.counter_h_lim = COUNTER_H_LIM;
    pcnt_config2.counter_l_lim = COUNTER_L_LIM;
    pcnt_config2.unit = PCNT_UNIT_0;
    pcnt_config2.channel = PCNT_CHANNEL_1;

    pcnt_config_t pcnt_config3 = {};
    pcnt_config3.pulse_gpio_num = ENC2_A;
    pcnt_config3.ctrl_gpio_num = ENC2_B;
    pcnt_config3.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config3.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config3.pos_mode = PCNT_COUNT_INC;
    pcnt_config3.neg_mode = PCNT_COUNT_DEC;
    pcnt_config3.counter_h_lim = COUNTER_H_LIM;
    pcnt_config3.counter_l_lim = COUNTER_L_LIM;
    pcnt_config3.unit = PCNT_UNIT_1;
    pcnt_config3.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config4 = {};
    pcnt_config4.pulse_gpio_num = ENC2_B;
    pcnt_config4.ctrl_gpio_num = ENC2_A;
    pcnt_config4.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config4.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config4.pos_mode = PCNT_COUNT_INC;
    pcnt_config4.neg_mode = PCNT_COUNT_DEC;
    pcnt_config4.counter_h_lim = COUNTER_H_LIM;
    pcnt_config4.counter_l_lim = COUNTER_L_LIM;
    pcnt_config4.unit = PCNT_UNIT_1;
    pcnt_config4.channel = PCNT_CHANNEL_1;

    pcnt_config_t pcnt_config5 = {};
    pcnt_config5.pulse_gpio_num = ENC3_A;
    pcnt_config5.ctrl_gpio_num = ENC3_B;
    pcnt_config5.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config5.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config5.pos_mode = PCNT_COUNT_INC;
    pcnt_config5.neg_mode = PCNT_COUNT_DEC;
    pcnt_config5.counter_h_lim = COUNTER_H_LIM;
    pcnt_config5.counter_l_lim = COUNTER_L_LIM;
    pcnt_config5.unit = PCNT_UNIT_2;
    pcnt_config5.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config6 = {};
    pcnt_config6.pulse_gpio_num = ENC3_B;
    pcnt_config6.ctrl_gpio_num = ENC3_A;
    pcnt_config6.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config6.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config6.pos_mode = PCNT_COUNT_INC;
    pcnt_config6.neg_mode = PCNT_COUNT_DEC;
    pcnt_config6.counter_h_lim = COUNTER_H_LIM;
    pcnt_config6.counter_l_lim = COUNTER_L_LIM;
    pcnt_config6.unit = PCNT_UNIT_2;
    pcnt_config6.channel = PCNT_CHANNEL_1;

    pcnt_config_t pcnt_config7 = {};
    pcnt_config7.pulse_gpio_num = ENC4_A;
    pcnt_config7.ctrl_gpio_num = ENC4_B;
    pcnt_config7.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config7.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config7.pos_mode = PCNT_COUNT_INC;
    pcnt_config7.neg_mode = PCNT_COUNT_DEC;
    pcnt_config7.counter_h_lim = COUNTER_H_LIM;
    pcnt_config7.counter_l_lim = COUNTER_L_LIM;
    pcnt_config7.unit = PCNT_UNIT_3;
    pcnt_config7.channel = PCNT_CHANNEL_0;

    pcnt_config_t pcnt_config8 = {};
    pcnt_config8.pulse_gpio_num = ENC4_B;
    pcnt_config8.ctrl_gpio_num = ENC4_A;
    pcnt_config8.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config8.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config8.pos_mode = PCNT_COUNT_INC;
    pcnt_config8.neg_mode = PCNT_COUNT_DEC;
    pcnt_config8.counter_h_lim = COUNTER_H_LIM;
    pcnt_config8.counter_l_lim = COUNTER_L_LIM;
    pcnt_config8.unit = PCNT_UNIT_3;
    pcnt_config8.channel = PCNT_CHANNEL_1;

    // パルスカウンタの初期化
    pcnt_unit_config(&pcnt_config1);
    pcnt_unit_config(&pcnt_config2);
    pcnt_unit_config(&pcnt_config3);
    pcnt_unit_config(&pcnt_config4);
    pcnt_unit_config(&pcnt_config5);
    pcnt_unit_config(&pcnt_config6);
    pcnt_unit_config(&pcnt_config7);
    pcnt_unit_config(&pcnt_config8);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_pause(PCNT_UNIT_2);
    pcnt_counter_pause(PCNT_UNIT_3);

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_2);
    pcnt_counter_clear(PCNT_UNIT_3);

    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_2);
    pcnt_counter_resume(PCNT_UNIT_3);

    // チャタリング防止のフィルターを有効化
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_filter_enable(PCNT_UNIT_1);
    pcnt_filter_enable(PCNT_UNIT_2);
    pcnt_filter_enable(PCNT_UNIT_3);

    // フィルター値を設定
    pcnt_set_filter_value(PCNT_UNIT_0, PCNT_FILTER_VALUE);
    pcnt_set_filter_value(PCNT_UNIT_1, PCNT_FILTER_VALUE);
    pcnt_set_filter_value(PCNT_UNIT_2, PCNT_FILTER_VALUE);
    pcnt_set_filter_value(PCNT_UNIT_3, PCNT_FILTER_VALUE);
}

// 各モードの初期化関数
void mode1_init() {
    // モード1用の初期化

    // PWMの初期化
    ledcAttach(MD1P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD2P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD3P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD4P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD5P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD6P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD7P, MD_PWM_FREQ, MD_PWM_RESOLUTION);
    ledcAttach(MD8P, MD_PWM_FREQ, MD_PWM_RESOLUTION);

    // 受信＆ピン操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        MD_Output_Task,
        "MD_Output_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        PRO_CPU_NUM);
}

void mode2_init() {
    // モード2用の初期化
    // SerialBT.println("Mode 2 Initialized");

    // エンコーダの初期化
    enc_init();

    // スイッチのピンを入力に設定し内蔵プルアップ抵抗を有効化
    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW3, INPUT_PULLUP);
    pinMode(SW4, INPUT_PULLUP);

    CAN.setPins(CAN_RX, CAN_TX); // rx.tx
    if (!CAN.begin(1000E3)) {
        while (1)
            ;
    }

    msg.data.data = (int32_t *)malloc(sizeof(int32_t) * 20);
    msg.data.size = 20;
    msg.data.capacity = 20;

    xTaskCreateUniversal(
        ROBOMAS_ENC_SW_Read_Publish_Task,
        "ROBOMAS_ENC_SW_Read_Publish_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    xTaskCreateUniversal(
        LED_PWM_Task,
        "LED_PWM_Task",
        2048,
        NULL,
        1, // 優先度、最大25？
        &led_pwm_handle,
        APP_CPU_NUM);
}

void mode3_init() {
    // モード3用の初期化
    // SerialBT.println("Mode 3 Initialized");

    // サーボのPWMの初期化
    ledcAttach(SERVO1, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO2, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO3, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO4, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO5, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO6, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO7, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO8, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);

    pinMode(SV1, OUTPUT);
    pinMode(SV2, OUTPUT);
    pinMode(SV3, OUTPUT);
    pinMode(SV4, OUTPUT);
    pinMode(SV5, OUTPUT);
    pinMode(SV6, OUTPUT);
    pinMode(SV7, OUTPUT);

    // サーボ操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        Servo_Output_Task,
        "Servo_Output_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    // ソレノイド操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        SV_Task,
        "SV_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);
}

void mode4_init() {
    // モード4用の初期化
    // SerialBT.println("Mode 4 Initialized");
    // Pub,Subを同時にしたときの遅延問題が解決できていないため未使用
    // とりあえず書いておく

    // エンコーダの初期化
    CAN.setPins(CAN_RX, CAN_TX); // rx.tx
    if (!CAN.begin(1000E3)) {
        while (1)
            ;
    }

    msg.data.data = (int32_t *)malloc(sizeof(int32_t) * 20);
    msg.data.size = 20;
    msg.data.capacity = 20;

    xTaskCreateUniversal(
        C610_FB_Task,
        "C610_FB_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    xTaskCreateUniversal(
        LED_PWM_Task,
        "LED_PWM_Task",
        2048,
        NULL,
        1, // 優先度、最大25？
        &led_pwm_handle,
        APP_CPU_NUM);
}

void mode5_init() {

    CAN.setPins(CAN_RX, CAN_TX); // rx.tx
    if (!CAN.begin(1000E3)) {
        while (1)
            ;
    }

    xTaskCreateUniversal(
        C610_Task,
        "C610_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    xTaskCreateUniversal(
        LED_PWM_Task,
        "LED_PWM_Task",
        2048,
        NULL,
        1, // 優先度、最大25？
        &led_pwm_handle,
        APP_CPU_NUM);
}

// テストモード　※実機で「絶対」に実行するな！
// シリアルモニターからEnterが押されるまで待機する
void mode0_init() {
    // デバッグ・テスト用の初期化
    Serial.println("Debug/Test Mode Initialized.");
    Serial.println("Press Enter to continue...");

    xTaskCreateUniversal(
        LED_Blink100_Task,
        "LED_Blink100_Task",
        2048,
        NULL,
        1, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    // テストモードの安全装置
    while (1) {
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') { // Enterが押されたら抜ける
                break;
            }
        }
    }

    switch (TEST_MODE) {
    case 0:
        // なにもしない
        Serial.println("MODE_DUMMY");
        while (1) {
            ;
        }
        break;
    case 1:
        // MDのテスト
        Serial.println("MD_TEST");
        mode1_init();
        while (1) {
            Serial.println("MD:20%");
            for (int i = 1; i <= 8; i++) {
                received_data[i] = 20;
            }
            delay(1000);

            Serial.println("MD:0%");
            for (int i = 1; i <= 8; i++) {
                received_data[i] = 0;
            }
            delay(1000);

            Serial.println("MD:-20%");
            for (int i = 1; i <= 8; i++) {
                received_data[i] = -20;
            }
            delay(1000);
        }
        break;
    case 2:
        // エンコーダ、スイッチのテスト
        Serial.println("ENC/SW_TEST");
        enc_init();
        mode2_init();
        while (1) {
            for (int i = 0; i < 4; i++) {
                Serial.print("count[");
                Serial.print(i);
                Serial.print("] = ");
                Serial.print(count[i]);
                Serial.print("\t sw_state[");
                Serial.print(i);
                Serial.print("] = ");
                Serial.println(sw_state[i]);
            }
            Serial.println("------");
        }
        break;
    case 3:
        // サーボ、ソレノイドのテスト
        Serial.println("SERVO/SV_TEST");
        mode3_init();
        while (1) {
            for (int i = 9; i <= 16; i++) {
                Serial.print("SERVO");
                Serial.print(i - 8);
                Serial.println(" Sweep");
                for (int angle = 0; angle <= 180; angle += 10) {
                    received_data[i] = angle;
                    delay(500);
                }
                for (int angle = 180; angle >= 0; angle -= 10) {
                    received_data[i] = angle;
                    delay(500);
                }
            }
            for (int i = 17; i <= 23; i++) {
                Serial.print("SV");
                Serial.print(i - 16);
                Serial.println(" ON");
                received_data[i] = 1;
                delay(1000);
                Serial.print("SV");
                Serial.print(i - 16);
                Serial.println(" OFF");
                received_data[i] = 0;
                delay(1000);
            }
        }
        break;

    case 5:
        // CANのテスト
        Serial.println("CAN_TEST");
        mode5_init();
        while (1) {
            // unsigned long now = millis();
            // float dt = (now - lastPidTime) / 1000.0;
            // if (dt <= 0)
            //     dt = 0.000001f; // dtが0にならないようにする
            // lastPidTime = now;

            // // 1. CAN受信
            // int packetSize = CAN.parsePacket();
            // while (packetSize) {               // 複数パケットも処理
            //     if (CAN.packetId() == 0x201) { // モータID=1
            //         uint8_t rx[8];
            //         for (int i = 0; i < 8; i++)
            //             rx[i] = CAN.read();
            //         encoder_count = (rx[0] << 8) | rx[1];
            //         rpm = (rx[2] << 8) | rx[3];

            //         // --- 初回オフセット設定 --- //
            //         if (!offset_ok) {
            //             encoder_offset = encoder_count;
            //             last_encoder_count = -1;
            //             rotation_count = 0;
            //             total_encoder_count = 0;
            //             pos_integral = 0;
            //             pos_error_prev = 0;
            //             offset_ok = true;
            //             // Serial.println("Offset set!");
            //         }

            //         int enc_relative = encoder_count - encoder_offset;
            //         if (enc_relative < 0)
            //             enc_relative += ENCODER_MAX; // wrap-around補正

            //         if (last_encoder_count != -1) {
            //             int diff = encoder_count - last_encoder_count;
            //             if (diff > HALF_ENCODER)
            //                 rotation_count--;
            //             else if (diff < -HALF_ENCODER)
            //                 rotation_count++;
            //         }

            //         last_encoder_count = encoder_count;
            //         total_encoder_count = rotation_count * ENCODER_MAX + encoder_count;
            //         angle = total_encoder_count * (360.0 / (8192.0 * gear_ratio));
            //         vel_input = (rpm / gear_ratio) * 360.0 / 60.0;
            //     }
            //     packetSize = CAN.parsePacket(); // 次の受信も処理
            // }
            // float pos_output = pid(target_angle, angle, pos_error_prev, pos_integral, kp_pos, ki_pos, kd_pos, dt);
            // // float vel_output = pid(pos_output, vel_input, vel_error_prev, vel_integral, kp_vel, ki_vel, kd_vel, dt);
            // motor_output_current_A = constrain_double(pos_output, -current_limit_A, current_limit_A);
            // // motor_output_current_A = 0.3;
            // // 2. コマンド送信
            // send_cur(motor_output_current_A);

            // // 3. デバッグ出力
            // // Serial.print("pos:\t"); Serial.println(angle);
            // Serial.println(target_angle - angle);

            // delay(1);
        }
        break;

    default:
        Serial.println("Invalid MODE for Test Mode.");
        while (1) {
            ;
        }
        break;
    }
}