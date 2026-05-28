// /*====================================================================
// Project: SimpleFOC_SERIAL_WIP
// Target board: B-G431B-ESC1 (disco_b_g431b_esc1)

// Description:
//   serial_bridge (ROS 2) と互換のフレーム(0xAA + DEVICE_ID + int16配列)で
//   目標値(速度/位置)を受信し、FOCで制御してテレメトリ(角度/速度など)を返す。

//   - ROS 側は serial_bridge/src/bridge_node.cpp の実装を前提
//   - DATA は int16 big-endian

// Copyright (c) 2026.
// ====================================================================*/

// #include "config.hpp"
// #include "frame_data.hpp"
// #include "serial_task.hpp"

// #include <Arduino.h>
// #include <SimpleFOC.h>
// #include <math.h>

// #include "STM32HWEncoder.h"

// #ifndef PI
// #define PI 3.14159265358979323846f
// #endif

// // ================= SimpleFOC objects =================

// // モータ極数
// BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

// // ドライバ設定
// BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL,
//                                        A_PHASE_VH, A_PHASE_VL,
//                                        A_PHASE_WH, A_PHASE_WL);

// // 電流センサ設定
// LowsideCurrentSense currentSense = LowsideCurrentSense(
//     0.003f, -64.0f / 7.0f,
//     A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// // // エンコーダ設定 (A,B相,PPR,Index)
// // #if ENCODER_ENABLE_INDEX
// // Encoder encoder = Encoder(A_HALL1, A_HALL2, ENCODER_PPR, A_HALL3);
// // #else
// // Encoder encoder = Encoder(A_HALL1, A_HALL2, ENCODER_PPR, 0);
// // #endif

// #if ENCODER_ENABLE_INDEX
// STM32HWEncoder encoder = STM32HWEncoder(ENCODER_PPR, A_HALL1, A_HALL2, A_HALL3);
// #else
// STM32HWEncoder encoder = STM32HWEncoder(ENCODER_PPR, A_HALL1, A_HALL2);
// #endif

// // void doA() { encoder.handleA(); }
// // void doB() { encoder.handleB(); }
// // void doIndex() { encoder.handleIndex(); }

// // ================= Control state =================

// enum ControlMode {
//     MODE_VELOCITY = 0,
//     MODE_ANGLE = 1,
// };

// static enum ControlMode control_mode = MODE_VELOCITY;
// static float target_velocity = 0.0f;
// static float target_angle = 0.0f;
// static enum ControlMode requested_control_mode = MODE_VELOCITY;
// static float requested_velocity = 0.0f;
// static float requested_angle = 0.0f;

// static float voltage_limit = DEFAULT_VOLTAGE_LIMIT;
// static int16_t debug_status = 0;

// static float last_angle = 0.0f;
// static uint32_t last_angle_sample_us = 0;
// static float estimated_velocity = 0.0f;
// static uint32_t last_target_update_us = 0;

// static constexpr float kDegToRad = PI / 180.0f;
// static constexpr float kRadToDeg = 180.0f / PI;
// static constexpr float kTwoPi = 2.0f * PI;
// static constexpr float kMicrosToSeconds = 1.0e-6f;

// // ================= Helpers =================

// static inline int16_t clamp_i16(long v) {
//     if (v > 32767)
//         return 32767;
//     if (v < -32768)
//         return -32768;
//     return (int16_t)v;
// }

// static float clamp_abs(float value, float limit) {
//     if (limit <= 0.0f)
//         return value;
//     if (value > limit)
//         return limit;
//     if (value < -limit)
//         return -limit;
//     return value;
// }

// static float ramp_towards(float current, float target, float max_delta) {
//     if (max_delta <= 0.0f)
//         return target;
//     if (target > current + max_delta)
//         return current + max_delta;
//     if (target < current - max_delta)
//         return current - max_delta;
//     return target;
// }

// static void apply_velocity_target(float v_rad_per_s) {
//     control_mode = MODE_VELOCITY;
//     target_velocity = v_rad_per_s;
//     motor.controller = MotionControlType::velocity;
//     motor.target = target_velocity;
// }

// static void apply_angle_target(float a_rad) {
//     control_mode = MODE_ANGLE;
//     target_angle = a_rad;
//     motor.controller = MotionControlType::angle;
//     motor.target = target_angle;
// }

// static void request_velocity_target(float v_rad_per_s) {
//     requested_control_mode = MODE_VELOCITY;
//     requested_velocity = v_rad_per_s;
// }

// static void request_angle_target(float a_rad) {
//     requested_control_mode = MODE_ANGLE;
//     requested_angle = a_rad;
// }

// static void update_velocity_estimate() {
//     estimated_velocity = motor.shaft_velocity;
// }

// static float wrap_0_to_2pi(float a_rad) {
//     float w = fmodf(a_rad, kTwoPi);
//     if (w < 0.0f)
//         w += kTwoPi;
//     return w;
// }

// static void update_ramped_target() {
// #if !ENABLE_TARGET_RAMP
//     if (requested_control_mode == MODE_ANGLE) {
//         apply_angle_target(requested_angle);
//     } else {
//         apply_velocity_target(requested_velocity);
//     }
//     return;
// #else
//     const uint32_t now = micros();
//     if (last_target_update_us == 0) {
//         last_target_update_us = now;
//     }

//     float dt = (float)(now - last_target_update_us) * kMicrosToSeconds;
//     if (dt < 0.0f)
//         dt = 0.0f;
//     last_target_update_us = now;

//     if (requested_control_mode != control_mode) {
//         if (requested_control_mode == MODE_ANGLE) {
//             // 位置モード切り替え時は現在角から追従開始して段差を避ける。
//             apply_angle_target(encoder.getAngle());
//         } else {
//             // 速度モード切り替え時は現在速度近傍から追従開始する。
//             apply_velocity_target(estimated_velocity);
//         }
//     }

//     if (control_mode == MODE_VELOCITY) {
//         const float max_delta = TARGET_VELOCITY_SLEW_RATE * dt;
//         apply_velocity_target(ramp_towards(target_velocity, requested_velocity, max_delta));
//     } else {
//         const float max_delta = TARGET_ANGLE_SLEW_RATE_DEG * kDegToRad * dt;
//         apply_angle_target(ramp_towards(target_angle, requested_angle, max_delta));
//     }
// #endif
// }

// static void apply_commands_from_rx() {
//     const uint32_t now = millis();
//     const uint32_t last_rx = serial_last_rx_ms();
//     const bool timed_out = (last_rx == 0) || (now - last_rx > CMD_TIMEOUT_MS);

//     const int16_t enable = Rx_16Data[RX_ENABLE];
//     const int16_t mode_raw = Rx_16Data[RX_MODE];

//     const bool has_nonzero_target = (Rx_16Data[RX_TARGET_VELOCITY] != 0) || (Rx_16Data[RX_TARGET_ANGLE] != 0);
//     const bool run = (!timed_out) && ((enable != 0) || (ALLOW_IMPLICIT_ENABLE && has_nonzero_target));

//     debug_status = 0;
//     if (timed_out)
//         debug_status |= 1;
//     if (enable != 0)
//         debug_status |= 2;
//     if (run)
//         debug_status |= 4;
//     if (mode_raw == 1)
//         debug_status |= 8;

//     // voltage limit
//     const float vlim = (float)Rx_16Data[RX_VOLTAGE_LIMIT] * VOLTAGE_LIMIT_SCALE;
//     if (!timed_out && vlim > 0.1f && vlim < 100.0f) {
//         voltage_limit = vlim;
//     }
//     motor.voltage_limit = voltage_limit;

//     if (!run) {
//         // フェイルセーフ: stop
//         request_velocity_target(0.0f);
//         return;
//     }

//     const enum ControlMode mode =
// #if FORCE_VELOCITY_MODE
//         MODE_VELOCITY;
// #else
//         (mode_raw == 1) ? MODE_ANGLE : MODE_VELOCITY;
// #endif

//     if (mode == MODE_VELOCITY) {
//         const float vel = clamp_abs(
//             (float)Rx_16Data[RX_TARGET_VELOCITY] * TARGET_VELOCITY_SCALE,
//             DEFAULT_VELOCITY_LIMIT);
//         request_velocity_target(vel);
//     } else {
//         const float ang_deg = (float)Rx_16Data[RX_TARGET_ANGLE] * TARGET_ANGLE_SCALE;
//         const float ang = ang_deg * kDegToRad;
//         request_angle_target(ang);
//     }
// }

// static void update_tx_telemetry() {
//     const float angle = encoder.getAngle();
//     const float angle_wrapped = wrap_0_to_2pi(angle);
//     const float vel = estimated_velocity;
//     const float target = (control_mode == MODE_VELOCITY) ? target_velocity : target_angle;

//     // measured mechanical RPM (signed)
//     const float rpm = vel * (60.0f / (2.0f * PI));

//     Tx_16Data[TX_DEBUG] = debug_status;

//     // angle (wrapped) [rad] -> 0.1 deg
//     Tx_16Data[TX_ANGLE] = clamp_i16(lroundf((angle_wrapped * kRadToDeg) * (1.0f / TARGET_ANGLE_SCALE)));

//     // velocity [rad/s] -> 0.1 rad/s
//     Tx_16Data[TX_VELOCITY] = clamp_i16(lroundf(vel * (1.0f / TARGET_VELOCITY_SCALE)));

//     // rpm [rpm] -> 1 rpm
//     Tx_16Data[TX_RPM] = clamp_i16(lroundf(rpm));

//     // target: unit depends on mode
//     if (control_mode == MODE_VELOCITY) {
//         Tx_16Data[TX_TARGET] = clamp_i16(lroundf(target * (1.0f / TARGET_VELOCITY_SCALE)));
//     } else {
//         const float target_wrapped = wrap_0_to_2pi(target);
//         Tx_16Data[TX_TARGET] = clamp_i16(lroundf((target_wrapped * kRadToDeg) * (1.0f / TARGET_ANGLE_SCALE)));
//     }

//     Tx_16Data[TX_MODE] = (int16_t)((control_mode == MODE_ANGLE) ? 1 : 0);
//     Tx_16Data[TX_VOLTAGE_LIMIT] = clamp_i16(lroundf(voltage_limit * (1.0f / VOLTAGE_LIMIT_SCALE)));
// }

// // ================= SETUP =================

// void setup() {

//     Serial.begin(115200);
//     delay(200);

//     // port_scanner が ID 判定のために短時間で1フレーム読むので、
//     // FOC初期化が長くても検出できるように先に送信する。
//     serial_task_init();
//     // open_serial_port() 側が USB CDC 安定待ちで 500ms 待つため、
//     // その間も含めて複数回送って検出成功率を上げる。
//     for (int i = 0; i < 30; i++) {
//         serial_task_send_now();
//         delay(50);
//     }

//     encoder.init();
// #if ENCODER_ENABLE_INDEX
//     encoder.enableInterrupts(doA, doB, doIndex);
// #else
//     // encoder.enableInterrupts(doA, doB);
// #endif

//     motor.linkSensor(&encoder);

//     driver.voltage_power_supply = DEFAULT_VOLTAGE_SUPPLY;
//     driver.init();
//     motor.linkDriver(&driver);

//     currentSense.linkDriver(&driver);
//     currentSense.init();
//     currentSense.skip_align = true;
//     motor.linkCurrentSense(&currentSense);

//     motor.voltage_sensor_align = 3;
//     motor.velocity_index_search = 3;

//     motor.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
//     motor.velocity_limit = DEFAULT_VELOCITY_LIMIT;
//     motor.current_limit = DEFAULT_CURRENT_LIMIT;

//     // 高速域での電圧利用率を上げて、飽和/ギクシャクを減らす
//     motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

//     // トルク制御方式
//     motor.torque_controller = TorqueControlType::foc_current;

//     // 電流制御 PID
//     motor.PID_current_q.P = motor.PID_current_d.P = CURRENT_PID_P;
//     motor.PID_current_q.I = motor.PID_current_d.I = CURRENT_PID_I;
//     motor.PID_current_q.D = motor.PID_current_d.D = CURRENT_PID_D;

//     // 速度制御 PID
//     motor.PID_velocity.P = VELOCITY_PID_P;
//     motor.PID_velocity.I = VELOCITY_PID_I;
//     motor.PID_velocity.D = VELOCITY_PID_D;
//     motor.PID_velocity.output_ramp = VELOCITY_PID_OUTPUT_RAMP;

//     // LPF
//     motor.LPF_velocity.Tf = VELOCITY_LPF_TF;

//     // 位置制御 (Pのみ)
//     motor.P_angle.P = ANGLE_P_GAIN;

//     apply_velocity_target(0.0f);
//     request_velocity_target(0.0f);

//     motor.init();
//     motor.initFOC();

//     last_angle = encoder.getAngle();
//     last_angle_sample_us = micros();
//     last_target_update_us = micros();
// }

// // ================= LOOP =================

// void loop() {
//     // FOC
//     motor.loopFOC();

//     // RX/TX
//     serial_task_update();

//     // Command apply
//     apply_commands_from_rx();

//     // Internal command ramp
//     update_ramped_target();

//     // Control
//     motor.move();

//     // Telemetry
//     update_velocity_estimate();
//     update_tx_telemetry();
// }

////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <SimpleFOC.h>

// #include "STM32HWEncoder.h"

// // ================= Encoder setting =================

// // AMT102-V
// // 48PPR設定なら 48
// #define ENCODER_PPR 48

// // B-G431B-ESC1
// // HALLコネクタ使用
// #define ENC_A A_HALL1
// #define ENC_B A_HALL2

// // ================= HW Encoder =================

// STM32HWEncoder encoder(ENCODER_PPR, PB6, PB7);

// // ================= Main =================

// void setup() {
//     Serial.begin(115200);
//     delay(2000);

//     Serial.println("encoder start");

//     // HW timer encoder mode init
//     encoder.init();
// }

// void loop() {
//     static uint32_t last_print = 0;

//     // 1msごとに更新
//     encoder.update();

//     // 50ms周期で表示
//     if (millis() - last_print > 50) {

//         last_print = millis();

//         float angle = encoder.getAngle();
//         float velocity = encoder.getVelocity();

//         float rpm = velocity * 60.0f / (2.0f * PI);

//         Serial.print("angle[rad]: ");
//         Serial.print(angle, 4);

//         Serial.print("  vel[rad/s]: ");
//         Serial.print(velocity, 4);

//         Serial.print("  rpm: ");
//         Serial.println(rpm, 2);
//     }
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// #include <Arduino.h>

// HardwareTimer *htim = new HardwareTimer(TIM2);

// void setup() {

//     Serial.begin(115200);

//     pinMode(PA0, INPUT_PULLUP);
//     pinMode(PA1, INPUT_PULLUP);

//     htim->setMode(1, TIMER_ENCODER);
//     htim->setMode(2, TIMER_ENCODER);

//     htim->resume();
// }

// void loop() {

//     Serial.println((int16_t)TIM2->CNT);

//     delay(50);
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// #include "STM32HWEncoder.h"
// #include <Arduino.h>

// #define ENCODER_PPR 48

// STM32HWEncoder encoder(
//     ENCODER_PPR,
//     PB6,
//     PB7);

// void setup() {

//     Serial.begin(115200);

//     delay(2000);

//     Serial.println("encoder start");

//     encoder.init();
// }

// void loop() {

//     static uint32_t last_print = 0;

//     if (millis() - last_print > 50) {

//         last_print = millis();

//         float angle =
//             encoder.getAngle();

//         float vel =
//             encoder.getVelocity();

//         float rpm =
//             vel * 60.0f / (2.0f * PI);

//         int32_t cnt =
//             encoder.getCount();

//         Serial.print("cnt: ");
//         Serial.print(cnt);

//         Serial.print(" angle: ");
//         Serial.print(angle, 4);

//         Serial.print(" vel: ");
//         Serial.print(vel, 4);

//         Serial.print(" rpm: ");
//         Serial.println(rpm, 2);
//     }
// }

// #include "HALEncoderSensor.h"
// #include <Arduino.h>

// #define ENCODER_PPR 48

// HALEncoderSensor encoder(ENCODER_PPR);

// void setup() {

//     Serial.begin(115200);

//     delay(2000);

//     Serial.println("HAL encoder start");

//     encoder.init();
// }

// void loop() {

//     encoder.update();

//     static uint32_t last_print = 0;

//     if (millis() - last_print > 50) {

//         last_print = millis();

//         float angle =
//             encoder.getAngle();

//         float vel =
//             encoder.getVelocity();

//         float rpm =
//             vel * 60.0f / (2.0f * PI);

//         Serial.print("angle: ");
//         Serial.print(angle, 4);

//         Serial.print(" vel: ");
//         Serial.print(vel, 4);

//         Serial.print(" rpm: ");
//         Serial.println(rpm, 2);
//     }
// }

// #include "stm32g4xx_hal.h"
// #include <Arduino.h>

// TIM_HandleTypeDef htim4;

// void setup() {

//     Serial.begin(115200);

//     delay(2000);

//     __HAL_RCC_GPIOB_CLK_ENABLE();
//     __HAL_RCC_TIM4_CLK_ENABLE();

//     GPIO_InitTypeDef GPIO_InitStruct = {};

//     GPIO_InitStruct.Pin =
//         GPIO_PIN_6 | GPIO_PIN_7;

//     GPIO_InitStruct.Mode =
//         GPIO_MODE_AF_PP;

//     GPIO_InitStruct.Pull =
//         GPIO_PULLUP;

//     GPIO_InitStruct.Speed =
//         GPIO_SPEED_FREQ_VERY_HIGH;

//     GPIO_InitStruct.Alternate =
//         GPIO_AF2_TIM4;

//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//     htim4.Instance = TIM4;

//     htim4.Init.Prescaler = 0;
//     htim4.Init.CounterMode =
//         TIM_COUNTERMODE_UP;

//     htim4.Init.Period = 65535;

//     htim4.Init.ClockDivision =
//         TIM_CLOCKDIVISION_DIV1;

//     TIM_Encoder_InitTypeDef sConfig = {};

//     sConfig.EncoderMode =
//         TIM_ENCODERMODE_TI12;

//     sConfig.IC1Polarity =
//         TIM_ICPOLARITY_RISING;

//     sConfig.IC1Selection =
//         TIM_ICSELECTION_DIRECTTI;

//     sConfig.IC1Prescaler =
//         TIM_ICPSC_DIV1;

//     sConfig.IC1Filter = 0;

//     sConfig.IC2Polarity =
//         TIM_ICPOLARITY_RISING;

//     sConfig.IC2Selection =
//         TIM_ICSELECTION_DIRECTTI;

//     sConfig.IC2Prescaler =
//         TIM_ICPSC_DIV1;

//     sConfig.IC2Filter = 0;

//     HAL_TIM_Encoder_Init(
//         &htim4,
//         &sConfig);

//     HAL_TIM_Encoder_Start(
//         &htim4,
//         TIM_CHANNEL_ALL);

//     Serial.println("start");
// }

// void loop() {

//     int16_t cnt =
//         __HAL_TIM_GET_COUNTER(&htim4);

//     Serial.println(cnt);

//     delay(50);
// }

// #include <STM32HWEncoder.h>
// #include <SimpleFOC.h>

// STM32HWEncoder encoder(48, PB6, PB7);

// void setup() {
//     Serial.begin(115200);
//     encoder.init();
// }

// void loop() {
//     encoder.update();
//     Serial.println(encoder.getCount());
//}

// #include "stm32g4xx_hal.h"
// #include <Arduino.h>

// // ======================
// // TIM4 encoder handle
// // ======================
// TIM_HandleTypeDef htim4;

// // 前回値
// static int32_t prev_cnt = 0;
// static uint32_t prev_time = 0;

// // ======================
// // TIM4 init (PB6/PB7)
// // ======================
// void MX_TIM4_Init(void) {
//     __HAL_RCC_TIM4_CLK_ENABLE();
//     __HAL_RCC_GPIOB_CLK_ENABLE();

//     GPIO_InitTypeDef GPIO_InitStruct = {0};

//     // PB6 / PB7 -> TIM4 AF2
//     GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_PULLUP;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//     htim4.Instance = TIM4;
//     htim4.Init.Prescaler = 0;
//     htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//     htim4.Init.Period = 0xFFFF;
//     htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//     htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

//     TIM_Encoder_InitTypeDef encoderConfig = {0};

//     encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;

//     // CH1 (A)
//     encoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
//     encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
//     encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
//     encoderConfig.IC1Filter = 3; // ⭐重要

//     // CH2 (B)
//     encoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
//     encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
//     encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
//     encoderConfig.IC2Filter = 3; // ⭐重要

//     HAL_TIM_Encoder_Init(&htim4, &encoderConfig);
//     HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
// }

// // ======================
// // setup
// // ======================
// void setup() {
//     Serial.begin(115200);

//     MX_TIM4_Init();

//     prev_cnt = __HAL_TIM_GET_COUNTER(&htim4);
//     prev_time = micros();
// }

// // ======================
// // loop
// // ======================
// void loop() {
//     uint32_t now = micros();
//     float dt = (now - prev_time) * 1e-6f;
//     prev_time = now;

//     int32_t cnt = __HAL_TIM_GET_COUNTER(&htim4);

//     // ★オーバーフロー対策（最重要）
//     int32_t diff = (int16_t)(cnt - prev_cnt);
//     prev_cnt = cnt;

//     // ======================
//     // 速度計算
//     // ======================
//     const float PPR = 48.0f;
//     const float CPR = PPR * 4.0f; // quadrature

//     float rps = diff / (CPR * dt);
//     float rpm = rps * 60.0f;

//     // ======================
//     // 出力
//     // ======================
//     Serial.print("CNT: ");
//     Serial.print(cnt);

//     Serial.print("  diff: ");
//     Serial.print(diff);

//     Serial.print("  RPM: ");
//     Serial.println(rpm);

//     delay(1);
// }

// #include "stm32g4xx_hal.h"
// #include <Arduino.h>

// // =====================
// // TIM4 handle
// // =====================
// TIM_HandleTypeDef htim4;

// // =====================
// // 状態ログ用
// // =====================
// uint32_t last_log = 0;
// int32_t prev_cnt = 0;

// // =====================
// // TIM4 init
// // =====================
// void MX_TIM4_Init(void) {
//     Serial.println("[INIT] TIM4 init start");

//     __HAL_RCC_TIM4_CLK_ENABLE();
//     __HAL_RCC_GPIOB_CLK_ENABLE();

//     GPIO_InitTypeDef GPIO_InitStruct = {0};

//     // PB6 / PB7 -> TIM4 AF2
//     GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//     Serial.println("[GPIO] PB6/PB7 configured");

//     htim4.Instance = TIM4;
//     htim4.Init.Prescaler = 0;
//     htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//     htim4.Init.Period = 0xFFFF;
//     htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//     htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

//     TIM_Encoder_InitTypeDef encoderConfig = {0};

//     encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;

//     encoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
//     encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
//     encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
//     encoderConfig.IC1Filter = 0;

//     encoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
//     encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
//     encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
//     encoderConfig.IC2Filter = 0;

//     if (HAL_TIM_Encoder_Init(&htim4, &encoderConfig) != HAL_OK) {
//         Serial.println("[ERROR] TIM4 init failed");
//     } else {
//         Serial.println("[OK] TIM4 encoder init done");
//     }

//     HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

//     Serial.println("[RUN] TIM4 encoder started");
// }

// // =====================
// // GPIO生存チェック
// // =====================
// void check_gpio() {
//     int a = digitalRead(PB6);
//     int b = digitalRead(PB7);

//     Serial.print("[GPIO] A=");
//     Serial.print(a);
//     Serial.print(" B=");
//     Serial.println(b);
// }

// // =====================
// // encoderチェック
// // =====================
// void check_encoder() {
//     int32_t cnt = __HAL_TIM_GET_COUNTER(&htim4);

//     int32_t diff = (int16_t)(cnt - prev_cnt);
//     prev_cnt = cnt;

//     Serial.print("[ENC] CNT=");
//     Serial.print(cnt);
//     Serial.print(" DIFF=");
//     Serial.println(diff);
// }

// // =====================
// // setup
// // =====================
// void setup() {
//     Serial.begin(115200);
//     delay(1000);

//     Serial.println("===== START DEBUG =====");

//     MX_TIM4_Init();

//     prev_cnt = __HAL_TIM_GET_COUNTER(&htim4);
//     last_log = millis();
// }

// // =====================
// // loop
// // =====================
// void loop() {
//     uint32_t now = millis();

//     // 1. GPIOチェック（2秒ごと）
//     if (now - last_log > 2000) {
//         Serial.println("----- STATUS -----");

//         check_gpio();
//         check_encoder();

//         Serial.println("------------------");

//         last_log = now;
//     }

//     // 2. 高頻度カウントチェック（毎回）
//     static int32_t last_fast = 0;
//     int32_t cnt = __HAL_TIM_GET_COUNTER(&htim4);

//     if (cnt != last_fast) {
//         Serial.print("[FAST] moving CNT=");
//         Serial.println(cnt);
//         last_fast = cnt;
//     }

//     delay(10);
// // }

// #include "stm32g4xx_hal.h"
// #include <Arduino.h>

// // ======================
// // TIM4 handle
// // ======================
// TIM_HandleTypeDef htim4;

// // ======================
// // 状態管理
// // ======================
// int32_t prev_cnt = 0;
// uint32_t last_log = 0;

// // ======================
// // TIM4 init (PB6/PB7 encoder)
// // ======================
// void MX_TIM4_Init(void) {
//     Serial.println("\n===== TIM4 INIT START =====");

//     __HAL_RCC_TIM4_CLK_ENABLE();
//     __HAL_RCC_GPIOB_CLK_ENABLE();

//     GPIO_InitTypeDef GPIO_InitStruct = {0};

//     // ===== PB6 / PB7 AF2 (TIM4) =====
//     GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

//     // ★重要：まずNOPULLで確認
//     GPIO_InitStruct.Pull = GPIO_NOPULL;

//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//     Serial.println("[GPIO] PB6/PB7 -> AF2 configured");

//     // ===== TIM4 basic =====
//     htim4.Instance = TIM4;
//     htim4.Init.Prescaler = 0;
//     htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//     htim4.Init.Period = 0xFFFF;
//     htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//     htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

//     TIM_Encoder_InitTypeDef enc = {0};

//     enc.EncoderMode = TIM_ENCODERMODE_TI12;

//     // ===== CH1 =====
//     enc.IC1Polarity = TIM_ICPOLARITY_RISING;
//     enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
//     enc.IC1Prescaler = TIM_ICPSC_DIV1;
//     enc.IC1Filter = 0; // ★超重要：まず0で動作確認

//     // ===== CH2 =====
//     enc.IC2Polarity = TIM_ICPOLARITY_RISING;
//     enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
//     enc.IC2Prescaler = TIM_ICPSC_DIV1;
//     enc.IC2Filter = 0; // ★同じく0

//     if (HAL_TIM_Encoder_Init(&htim4, &enc) != HAL_OK) {
//         Serial.println("[ERROR] TIM4 INIT FAILED");
//     } else {
//         Serial.println("[OK] TIM4 INIT OK");
//     }

//     HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

//     Serial.println("[RUN] TIM4 STARTED");
//     Serial.println("==========================\n");
// }

// // ======================
// // GPIO raw check
// // ======================
// void check_gpio_raw() {
//     int a = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
//     int b = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);

//     Serial.print("[GPIO RAW] A=");
//     Serial.print(a);
//     Serial.print(" B=");
//     Serial.println(b);
// }

// // ======================
// // encoder check
// // ======================
// void check_encoder() {
//     int32_t cnt = __HAL_TIM_GET_COUNTER(&htim4);
//     int32_t diff = (int16_t)(cnt - prev_cnt);
//     prev_cnt = cnt;

//     Serial.print("[ENC] CNT=");
//     Serial.print(cnt);
//     Serial.print(" DIFF=");
//     Serial.println(diff);
// }

// // ======================
// // setup
// // ======================
// void setup() {
//     Serial.begin(115200);
//     delay(1000);

//     Serial.println("\n========================");
//     Serial.println("  TIM4 FULL DEBUG START");
//     Serial.println("========================\n");

//     MX_TIM4_Init();

//     prev_cnt = __HAL_TIM_GET_COUNTER(&htim4);
// }

// // ======================
// // loop
// // ======================
// void loop() {
//     uint32_t now = millis();

//     // ===== 2秒ごと詳細ログ =====
//     if (now - last_log > 2000) {
//         Serial.println("\n----- STATUS -----");
//         check_gpio_raw();
//         check_encoder();
//         Serial.println("------------------\n");

//         last_log = now;
//     }

//     // ===== 高速変化検出 =====
//     static int32_t last_fast = 0;
//     int32_t cnt = __HAL_TIM_GET_COUNTER(&htim4);

//     if (cnt != last_fast) {
//         Serial.print("[FAST] CNT=");
//         Serial.println(cnt);
//         last_fast = cnt;
//     }

//     delay(5);
// }

#include "stm32g4xx_hal.h"
#include <Arduino.h>

// ======================
// TIM4 encoder handle
// ======================
TIM_HandleTypeDef htim4;

// ======================
// 状態
// ======================
static int32_t prev_cnt = 0;
static uint32_t last_log = 0;

// ======================
// TIM4 init
// ======================
void MX_TIM4_Init(void) {
    Serial.println("\n===== TIM4 INIT (STABLE MODE) =====");

    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PB6 / PB7 -> TIM4 AF2
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

    // ★安定優先：プルアップ（ノイズ耐性）
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    Serial.println("[GPIO] PB6/PB7 AF2 + PULLUP");

    // ======================
    // TIM4 base config
    // ======================
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xFFFF;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef enc = {0};
    enc.EncoderMode = TIM_ENCODERMODE_TI12;

    // ======================
    // CH1
    // ======================
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;

    // ★安定化の核心
    enc.IC1Filter = 3;

    // ======================
    // CH2
    // ======================
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;

    // ★同じくフィルタ
    enc.IC2Filter = 3;

    if (HAL_TIM_Encoder_Init(&htim4, &enc) != HAL_OK) {
        Serial.println("[ERROR] TIM4 INIT FAILED");
    } else {
        Serial.println("[OK] TIM4 INIT SUCCESS");
    }

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    Serial.println("[RUN] TIM4 STARTED");
    Serial.println("==================================\n");
}

// ======================
// encoder read
// ======================
float get_speed_rpm(int32_t diff, float dt) {
    const float PPR = 48.0f;
    const float CPR = PPR * 4.0f;
    return (diff / (CPR * dt)) * 60.0f;
}

// ======================
// setup
// ======================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== ENCODER STABLE MODE START ===");

    MX_TIM4_Init();

    prev_cnt = __HAL_TIM_GET_COUNTER(&htim4);
}

// ======================
// loop
// ======================
void loop() {
    uint32_t now = micros();
    static uint32_t prev_time = now;

    float dt = (now - prev_time) * 1e-6f;
    prev_time = now;

    int32_t cnt = __HAL_TIM_GET_COUNTER(&htim4);

    // ======================
    // 差分（ノイズ耐性）
    // ======================
    int32_t diff = (int16_t)(cnt - prev_cnt);
    prev_cnt = cnt;

    // ======================
    // RPM
    // ======================
    float rpm = get_speed_rpm(diff, dt);

    // ======================
    // デバッグログ（間引き）
    // ======================
    if (millis() - last_log > 50) {
        last_log = millis();

        Serial.print("[CNT] ");
        Serial.print(cnt);

        Serial.print("  [DIFF] ");
        Serial.print(diff);

        Serial.print("  [RPM] ");
        Serial.println(rpm);
    }

    delay(1);
}