/*====================================================================
Project: SimpleFOC_SERIAL_WIP
Target board: B-G431B-ESC1 (disco_b_g431b_esc1)

Description:
  serial_bridge (ROS 2) と互換のフレーム(0xAA + DEVICE_ID + int16配列)で
  目標値(速度/位置)を受信し、FOCで制御してテレメトリ(角度/速度など)を返す。

  - ROS 側は serial_bridge/src/bridge_node.cpp の実装を前提
  - DATA は int16 big-endian

Copyright (c) 2026.
====================================================================*/

#include "config.hpp"
#include "frame_data.hpp"
#include "serial_task.hpp"

#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ================= SimpleFOC objects =================

// モータ極数
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

// ドライバ設定
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL,
                                       A_PHASE_VH, A_PHASE_VL,
                                       A_PHASE_WH, A_PHASE_WL);

// 電流センサ設定
LowsideCurrentSense currentSense = LowsideCurrentSense(
    0.003f, -64.0f / 7.0f,
    A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// エンコーダ設定 (A,B相,PPR,Index)
Encoder encoder = Encoder(A_HALL1, A_HALL2, 2048, A_HALL3);

void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
void doIndex() { encoder.handleIndex(); }

// ================= Control state =================

enum ControlMode {
    MODE_VELOCITY = 0,
    MODE_ANGLE = 1,
};

static enum ControlMode control_mode = MODE_VELOCITY;
static float target_velocity = 0.0f;
static float target_angle = 0.0f;
static enum ControlMode requested_control_mode = MODE_VELOCITY;
static float requested_velocity = 0.0f;
static float requested_angle = 0.0f;

static float voltage_limit = DEFAULT_VOLTAGE_LIMIT;

static float last_angle = 0.0f;
static uint32_t last_angle_sample_us = 0;
static float estimated_velocity = 0.0f;
static uint32_t last_target_update_us = 0;

static constexpr float kDegToRad = PI / 180.0f;
static constexpr float kRadToDeg = 180.0f / PI;
static constexpr float kMicrosToSeconds = 1.0e-6f;

// ================= Helpers =================

static inline int16_t clamp_i16(long v) {
    if (v > 32767)
        return 32767;
    if (v < -32768)
        return -32768;
    return (int16_t)v;
}

static float clamp_abs(float value, float limit) {
    if (limit <= 0.0f)
        return value;
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}

static float ramp_towards(float current, float target, float max_delta) {
    if (max_delta <= 0.0f)
        return target;
    if (target > current + max_delta)
        return current + max_delta;
    if (target < current - max_delta)
        return current - max_delta;
    return target;
}

static void apply_velocity_target(float v_rad_per_s) {
    control_mode = MODE_VELOCITY;
    target_velocity = v_rad_per_s;
    motor.controller = MotionControlType::velocity;
    motor.target = target_velocity;
}

static void apply_angle_target(float a_rad) {
    control_mode = MODE_ANGLE;
    target_angle = a_rad;
    motor.controller = MotionControlType::angle;
    motor.target = target_angle;
}

static void request_velocity_target(float v_rad_per_s) {
    requested_control_mode = MODE_VELOCITY;
    requested_velocity = v_rad_per_s;
}

static void request_angle_target(float a_rad) {
    requested_control_mode = MODE_ANGLE;
    requested_angle = a_rad;
}

static void update_velocity_estimate() {
    const uint32_t now = micros();
    const float angle = encoder.getAngle();

    if (last_angle_sample_us != 0) {
        const float dt = (float)(now - last_angle_sample_us) * kMicrosToSeconds;
        if (dt > 0.0f) {
            estimated_velocity = (angle - last_angle) / dt;
        }
    }

    last_angle = angle;
    last_angle_sample_us = now;
}

static void update_ramped_target() {
#if !ENABLE_TARGET_RAMP
    if (requested_control_mode == MODE_ANGLE) {
        apply_angle_target(requested_angle);
    } else {
        apply_velocity_target(requested_velocity);
    }
    return;
#else
    const uint32_t now = micros();
    if (last_target_update_us == 0) {
        last_target_update_us = now;
    }

    float dt = (float)(now - last_target_update_us) * kMicrosToSeconds;
    if (dt < 0.0f)
        dt = 0.0f;
    last_target_update_us = now;

    if (requested_control_mode != control_mode) {
        if (requested_control_mode == MODE_ANGLE) {
            // 位置モード切り替え時は現在角から追従開始して段差を避ける。
            apply_angle_target(encoder.getAngle());
        } else {
            // 速度モード切り替え時は現在速度近傍から追従開始する。
            apply_velocity_target(estimated_velocity);
        }
    }

    if (control_mode == MODE_VELOCITY) {
        const float max_delta = TARGET_VELOCITY_SLEW_RATE * dt;
        apply_velocity_target(ramp_towards(target_velocity, requested_velocity, max_delta));
    } else {
        const float max_delta = TARGET_ANGLE_SLEW_RATE_DEG * kDegToRad * dt;
        apply_angle_target(ramp_towards(target_angle, requested_angle, max_delta));
    }
#endif
}

static void apply_commands_from_rx() {
    const uint32_t now = millis();
    const uint32_t last_rx = serial_last_rx_ms();
    const bool timed_out = (last_rx == 0) || (now - last_rx > CMD_TIMEOUT_MS);

    const int16_t enable = Rx_16Data[RX_ENABLE];
    const int16_t mode_raw = Rx_16Data[RX_MODE];

    const bool run = (!timed_out) && (enable != 0);

    // voltage limit
    const float vlim = (float)Rx_16Data[RX_VOLTAGE_LIMIT] * VOLTAGE_LIMIT_SCALE;
    if (!timed_out && vlim > 0.1f && vlim < 100.0f) {
        voltage_limit = vlim;
    }
    motor.voltage_limit = voltage_limit;

    if (!run) {
        // フェイルセーフ: stop
        request_velocity_target(0.0f);
        return;
    }

    const enum ControlMode mode =
#if FORCE_VELOCITY_MODE
        MODE_VELOCITY;
#else
        (mode_raw == 1) ? MODE_ANGLE : MODE_VELOCITY;
#endif

    if (mode == MODE_VELOCITY) {
        const float vel = clamp_abs(
            (float)Rx_16Data[RX_TARGET_VELOCITY] * TARGET_VELOCITY_SCALE,
            DEFAULT_VELOCITY_LIMIT);
        request_velocity_target(vel);
    } else {
        const float ang_deg = (float)Rx_16Data[RX_TARGET_ANGLE] * TARGET_ANGLE_SCALE;
        const float ang = ang_deg * kDegToRad;
        request_angle_target(ang);
    }
}

static void update_tx_telemetry() {
    const float angle = encoder.getAngle();
    const float vel = estimated_velocity;
    const float target = (control_mode == MODE_VELOCITY) ? target_velocity : target_angle;

    Tx_16Data[TX_DEBUG] = 0;

    // angle [rad] -> 0.1 deg
    Tx_16Data[TX_ANGLE] = clamp_i16(lroundf((angle * kRadToDeg) * (1.0f / TARGET_ANGLE_SCALE)));

    // velocity [rad/s] -> 0.1 rad/s
    Tx_16Data[TX_VELOCITY] = clamp_i16(lroundf(vel * (1.0f / TARGET_VELOCITY_SCALE)));

    // target: unit depends on mode
    if (control_mode == MODE_VELOCITY) {
        Tx_16Data[TX_TARGET] = clamp_i16(lroundf(target * (1.0f / TARGET_VELOCITY_SCALE)));
    } else {
        Tx_16Data[TX_TARGET] = clamp_i16(lroundf((target * kRadToDeg) * (1.0f / TARGET_ANGLE_SCALE)));
    }

    Tx_16Data[TX_MODE] = (int16_t)((control_mode == MODE_ANGLE) ? 1 : 0);
    Tx_16Data[TX_VOLTAGE_LIMIT] = clamp_i16(lroundf(voltage_limit * (1.0f / VOLTAGE_LIMIT_SCALE)));
}

// ================= SETUP =================

void setup() {

    Serial.begin(115200);
    delay(200);

    // port_scanner が ID 判定のために短時間で1フレーム読むので、
    // FOC初期化が長くても検出できるように先に送信する。
    serial_task_init();
    // open_serial_port() 側が USB CDC 安定待ちで 500ms 待つため、
    // その間も含めて複数回送って検出成功率を上げる。
    for (int i = 0; i < 30; i++) {
        serial_task_send_now();
        delay(50);
    }

    encoder.init();
    encoder.enableInterrupts(doA, doB, doIndex);

    motor.linkSensor(&encoder);

    driver.voltage_power_supply = DEFAULT_VOLTAGE_SUPPLY;
    driver.init();
    motor.linkDriver(&driver);

    currentSense.linkDriver(&driver);
    currentSense.init();
    currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);

    motor.voltage_sensor_align = 1;
    motor.velocity_index_search = 3;

    motor.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
    motor.velocity_limit = DEFAULT_VELOCITY_LIMIT;

    // トルク制御方式
    motor.torque_controller = TorqueControlType::foc_current;

    // 電流制御 PID
    motor.PID_current_q.P = motor.PID_current_d.P = CURRENT_PID_P;
    motor.PID_current_q.I = motor.PID_current_d.I = CURRENT_PID_I;
    motor.PID_current_q.D = motor.PID_current_d.D = CURRENT_PID_D;

    // 速度制御 PID
    motor.PID_velocity.P = VELOCITY_PID_P;
    motor.PID_velocity.I = VELOCITY_PID_I;
    motor.PID_velocity.D = VELOCITY_PID_D;
    motor.PID_velocity.output_ramp = VELOCITY_PID_OUTPUT_RAMP;

    // LPF
    motor.LPF_velocity.Tf = VELOCITY_LPF_TF;

    // 位置制御 (Pのみ)
    motor.P_angle.P = ANGLE_P_GAIN;

    apply_velocity_target(0.0f);
    request_velocity_target(0.0f);

    motor.init();
    motor.initFOC();

    last_angle = encoder.getAngle();
    last_angle_sample_us = micros();
    last_target_update_us = micros();
}

// ================= LOOP =================

void loop() {
    // RX/TX
    serial_task_update();

    // FOC
    motor.loopFOC();

    // Command apply
    apply_commands_from_rx();

    // Internal command ramp
    update_ramped_target();

    // Control
    motor.move();

    // Telemetry
    update_velocity_estimate();
    update_tx_telemetry();
}
