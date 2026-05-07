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

// ================= SimpleFOC objects =================

// モータ極数
BLDCMotor motor = BLDCMotor(7);

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

static float voltage_limit = DEFAULT_VOLTAGE_LIMIT;

static float last_angle = 0.0f;
static uint32_t last_angle_sample_ms = 0;
static float estimated_velocity = 0.0f;

// ================= Helpers =================

static inline int16_t clamp_i16(long v) {
    if (v > 32767)
        return 32767;
    if (v < -32768)
        return -32768;
    return (int16_t)v;
}

static void set_velocity_target(float v_rad_per_s) {
    control_mode = MODE_VELOCITY;
    target_velocity = v_rad_per_s;
    motor.controller = MotionControlType::velocity;
    motor.target = target_velocity;
}

static void set_angle_target(float a_rad) {
    control_mode = MODE_ANGLE;
    target_angle = a_rad;
    motor.controller = MotionControlType::angle;
    motor.target = target_angle;
}

static void update_velocity_estimate() {
    const uint32_t now = millis();
    const float angle = encoder.getAngle();

    if (last_angle_sample_ms != 0) {
        const float dt = (float)(now - last_angle_sample_ms) * 0.001f;
        if (dt > 0.0f) {
            estimated_velocity = (angle - last_angle) / dt;
        }
    }

    last_angle = angle;
    last_angle_sample_ms = now;
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
        set_velocity_target(0.0f);
        return;
    }

    const enum ControlMode mode = (mode_raw == 1) ? MODE_ANGLE : MODE_VELOCITY;

    if (mode == MODE_VELOCITY) {
        const float vel = (float)Rx_16Data[RX_TARGET_VELOCITY] * TARGET_VELOCITY_SCALE;
        set_velocity_target(vel);
    } else {
        const float ang = (float)Rx_16Data[RX_TARGET_ANGLE] * TARGET_ANGLE_SCALE;
        set_angle_target(ang);
    }
}

static void update_tx_telemetry() {
    const float angle = encoder.getAngle();
    const float vel = estimated_velocity;
    const float target = (control_mode == MODE_VELOCITY) ? target_velocity : target_angle;

    Tx_16Data[TX_DEBUG] = 0;

    // angle [rad] -> mrad
    Tx_16Data[TX_ANGLE] = clamp_i16(lroundf(angle * (1.0f / TARGET_ANGLE_SCALE)));

    // velocity [rad/s] -> 0.1 rad/s
    Tx_16Data[TX_VELOCITY] = clamp_i16(lroundf(vel * (1.0f / TARGET_VELOCITY_SCALE)));

    // target: unit depends on mode
    if (control_mode == MODE_VELOCITY) {
        Tx_16Data[TX_TARGET] = clamp_i16(lroundf(target * (1.0f / TARGET_VELOCITY_SCALE)));
    } else {
        Tx_16Data[TX_TARGET] = clamp_i16(lroundf(target * (1.0f / TARGET_ANGLE_SCALE)));
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
    motor.PID_current_q.P = motor.PID_current_d.P = 0.1f;
    motor.PID_current_q.I = motor.PID_current_d.I = 10.0f;
    motor.PID_current_q.D = motor.PID_current_d.D = 0.0f;

    // 速度制御 PID
    motor.PID_velocity.P = 0.5f;
    motor.PID_velocity.I = 1.0f;
    motor.PID_velocity.D = 0.0f;
    motor.PID_velocity.output_ramp = 1000.0f;

    // LPF
    motor.LPF_velocity.Tf = 0.01f;

    // 位置制御 (Pのみ)
    motor.P_angle.P = 9.0f;

    set_velocity_target(0.0f);

    motor.init();
    motor.initFOC();

    last_angle = encoder.getAngle();
    last_angle_sample_ms = millis();
}

// ================= LOOP =================

void loop() {

    // RX/TX
    serial_task_update();

    // FOC
    motor.loopFOC();

    // Command apply
    apply_commands_from_rx();

    // Control
    motor.move();

    // Telemetry
    update_velocity_estimate();
    update_tx_telemetry();
}
