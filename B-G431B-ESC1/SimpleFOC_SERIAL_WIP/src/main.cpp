#include "config.hpp"
#include "frame_data.hpp"
#include "serial_task.hpp"

#include <algorithm>
#include <cmath>

#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>

#include "stm32g4xx_hal.h"

// ======================================================
// TIM4 HARD ENCODER
// ======================================================
TIM_HandleTypeDef htim4;

// ======================================================
// SimpleFOC Sensor (TIM4 wrapper)
// ======================================================
class TIM4Sensor : public Sensor {
public:
    int32_t cpr;

    TIM4Sensor(int32_t cpr) {
        this->cpr = cpr;
    }

    void init() override {
        prev = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        full_rotations = 0;
    }

    float getSensorAngle() override {
        int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

        int16_t diff = now - prev;
        prev = now;

        // 16bitオーバーフロー補正
        if (diff > 32767)
            diff -= 65536;
        if (diff < -32768)
            diff += 65536;

        acc += diff;

        // ★ここが重要：連続角（ラップ禁止）
        return (float)acc / (float)cpr * _2PI;
    }

private:
    int16_t prev = 0;
    int32_t acc = 0;
};

// AMT102 PPR
TIM4Sensor encoder(2048);

// ======================================================
// SimpleFOC Objects
// ======================================================
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

BLDCDriver6PWM driver = BLDCDriver6PWM(
    A_PHASE_UH, A_PHASE_UL,
    A_PHASE_VH, A_PHASE_VL,
    A_PHASE_WH, A_PHASE_WL);

LowsideCurrentSense currentSense =
    LowsideCurrentSense(0.003f, -64.0f / 7.0f,
                        A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// ======================================================
// Control state (ROS bridge)
// ======================================================
enum ControlMode {
    MODE_VELOCITY = 0,
    MODE_ANGLE = 1,
};

static ControlMode control_mode = MODE_VELOCITY;
static float target_velocity = 0;
static float target_angle = 0;

static float voltage_limit = DEFAULT_VOLTAGE_LIMIT;
static int16_t debug_status = 0;

static float estimated_velocity = 0;

// last applied mask (for edge-detection if desired)
static int16_t last_param_apply_mask = 0;

// ======================================================
// TIM4 init (FINAL STABLE)
// ======================================================
void tim4_init() {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP; // 安定優先
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xFFFF;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef enc = {0};
    enc.EncoderMode = TIM_ENCODERMODE_TI12;

    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter = 3;

    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter = 3;

    HAL_TIM_Encoder_Init(&htim4, &enc);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

// ======================================================
// helpers
// ======================================================
static void apply_velocity(float v) {
    control_mode = MODE_VELOCITY;
    target_velocity = v;
    motor.controller = MotionControlType::velocity;
    motor.target = v;
}

static void apply_angle(float a) {
    control_mode = MODE_ANGLE;
    target_angle = a;
    motor.controller = MotionControlType::angle;
    motor.target = a * (PI / 180.0f);
}

static float clamp_non_negative(float value) {
    return (value < 0.0f) ? 0.0f : value;
}

static void apply_params_from_rx() {
    const int16_t mask = Rx_16Data[RX_PARAM_APPLY_MASK];
    if (mask == 0) {
        last_param_apply_mask = 0;
        return;
    }

    // Apply on any non-zero mask (edge-detection can be added later).
    (void)last_param_apply_mask;
    last_param_apply_mask = mask;

    if (mask & RX_PARAM_APPLY_VOLTAGE_LIMIT) {
        const float v = clamp_non_negative(Rx_16Data[RX_VOLTAGE_LIMIT] * VOLTAGE_LIMIT_SCALE);
        // keep within supply if known
        motor.voltage_limit = std::min(v, (float)DEFAULT_VOLTAGE_SUPPLY);
        voltage_limit = motor.voltage_limit;
    }

    if (mask & RX_PARAM_APPLY_VELOCITY_LIMIT) {
        const float vlim = clamp_non_negative(Rx_16Data[RX_VELOCITY_LIMIT] * VELOCITY_LIMIT_SCALE);
        motor.velocity_limit = vlim;
    }

    if (mask & RX_PARAM_APPLY_CURRENT_LIMIT) {
        const float clim = clamp_non_negative(Rx_16Data[RX_CURRENT_LIMIT] * CURRENT_LIMIT_SCALE);
        motor.current_limit = clim;
    }

    if (mask & RX_PARAM_APPLY_VELOCITY_PID) {
        motor.PID_velocity.P = Rx_16Data[RX_VELOCITY_PID_P] * VELOCITY_PID_GAIN_SCALE;
        motor.PID_velocity.I = Rx_16Data[RX_VELOCITY_PID_I] * VELOCITY_PID_GAIN_SCALE;
        motor.PID_velocity.D = Rx_16Data[RX_VELOCITY_PID_D] * VELOCITY_PID_GAIN_SCALE;
    }

    if (mask & RX_PARAM_APPLY_VELOCITY_OUTPUT_RAMP) {
        motor.PID_velocity.output_ramp = clamp_non_negative((float)Rx_16Data[RX_VELOCITY_PID_OUTPUT_RAMP]);
    }

    if (mask & RX_PARAM_APPLY_VELOCITY_LPF_TF) {
        const float tf_ms = clamp_non_negative((float)Rx_16Data[RX_VELOCITY_LPF_TF_MS]);
        motor.LPF_velocity.Tf = tf_ms / 1000.0f;
    }

    if (mask & RX_PARAM_APPLY_ANGLE_P_GAIN) {
        motor.P_angle.P = Rx_16Data[RX_ANGLE_P_GAIN] * ANGLE_P_GAIN_SCALE;
    }
}

// ======================================================
// RX command handling
// ======================================================
static void apply_rx() {
    const int16_t enable = Rx_16Data[RX_ENABLE];
    const int16_t mode = Rx_16Data[RX_MODE];

    const bool run = (enable != 0);

    if (!run) {
        apply_velocity(0);
        return;
    }

    if (mode == 1) {
        float ang = Rx_16Data[RX_TARGET_ANGLE] * TARGET_ANGLE_SCALE;
        apply_angle(ang);
    } else {
        float vel = Rx_16Data[RX_TARGET_VELOCITY] * TARGET_VELOCITY_SCALE;
        apply_velocity(vel);
    }
}

// ======================================================
// telemetry
// ======================================================
static void update_tx() {
    float angle = encoder.getAngle();
    float vel = motor.shaft_velocity;

    Tx_16Data[TX_ANGLE] = (int16_t)std::lround((angle * 180.0f / PI) / TARGET_ANGLE_SCALE);
    Tx_16Data[TX_VELOCITY] = (int16_t)std::lround(vel / TARGET_VELOCITY_SCALE);
    Tx_16Data[TX_DEBUG] = debug_status;

    Tx_16Data[TX_MODE] = (int16_t)control_mode;
    if (control_mode == MODE_ANGLE) {
        Tx_16Data[TX_TARGET] = (int16_t)(target_angle / TARGET_ANGLE_SCALE);
    } else {
        Tx_16Data[TX_TARGET] = (int16_t)(target_velocity / TARGET_VELOCITY_SCALE);
    }

    Tx_16Data[TX_VOLTAGE_LIMIT] = (int16_t)(motor.voltage_limit / VOLTAGE_LIMIT_SCALE);

    const float rpm = vel * 60.0f / _2PI;
    Tx_16Data[TX_RPM] = (int16_t)rpm;

    // Echo runtime parameters (best-effort)
    Tx_16Data[TX_VELOCITY_LIMIT] = (int16_t)(motor.velocity_limit / VELOCITY_LIMIT_SCALE);
    Tx_16Data[TX_CURRENT_LIMIT] = (int16_t)(motor.current_limit / CURRENT_LIMIT_SCALE);

    Tx_16Data[TX_VELOCITY_PID_P] = (int16_t)(motor.PID_velocity.P / VELOCITY_PID_GAIN_SCALE);
    Tx_16Data[TX_VELOCITY_PID_I] = (int16_t)(motor.PID_velocity.I / VELOCITY_PID_GAIN_SCALE);
    Tx_16Data[TX_VELOCITY_PID_D] = (int16_t)(motor.PID_velocity.D / VELOCITY_PID_GAIN_SCALE);
    Tx_16Data[TX_VELOCITY_PID_OUTPUT_RAMP] = (int16_t)(motor.PID_velocity.output_ramp);
    Tx_16Data[TX_VELOCITY_LPF_TF_MS] = (int16_t)std::lround(motor.LPF_velocity.Tf * 1000.0f);
    Tx_16Data[TX_ANGLE_P_GAIN] = (int16_t)(motor.P_angle.P / ANGLE_P_GAIN_SCALE);
}

// ======================================================
// SETUP
// ======================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    serial_task_init();

    // 初期ブロードキャスト
    for (int i = 0; i < 20; i++) {
        serial_task_send_now();
        delay(50);
    }

    // ===== TIM4 =====
    tim4_init();
    encoder.init();

    // ===== driver =====
    driver.voltage_power_supply = DEFAULT_VOLTAGE_SUPPLY;
    driver.init();

    motor.linkDriver(&driver);

    // ===== sensor =====
    motor.linkSensor(&encoder);

    // ===== current sense =====
    currentSense.linkDriver(&driver);
    currentSense.init();
    currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);

    // ===== config =====
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller = TorqueControlType::foc_current;

    motor.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
    motor.velocity_limit = DEFAULT_VELOCITY_LIMIT;
    motor.current_limit = DEFAULT_CURRENT_LIMIT;

    motor.PID_velocity.P = VELOCITY_PID_P;
    motor.PID_velocity.I = VELOCITY_PID_I;
    motor.PID_velocity.D = VELOCITY_PID_D;

    motor.P_angle.P = ANGLE_P_GAIN;

    motor.LPF_velocity.Tf = 0.05f;
    motor.PID_velocity.output_ramp = 100;

    motor.init();
    motor.initFOC();

    apply_velocity(0);
}

// ======================================================
// LOOP
// ======================================================
void loop() {
    motor.loopFOC();

    serial_task_update();

    apply_params_from_rx();

    apply_rx();

    motor.move();

    update_tx();
}