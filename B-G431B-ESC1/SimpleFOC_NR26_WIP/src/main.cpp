#include <Arduino.h>
#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);

BLDCDriver6PWM driver = BLDCDriver6PWM(
    A_PHASE_UH, A_PHASE_UL,
    A_PHASE_VH, A_PHASE_VL,
    A_PHASE_WH, A_PHASE_WL);

Encoder encoder = Encoder(A_HALL1, A_HALL2, 2048, A_HALL3);

void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
void doIndex() { encoder.handleIndex(); }

Commander command = Commander(Serial);

int speed_limit = 100;

// ---- 速度制御（rad/s）----
void doVelocity(char *cmd) {
    float v = atof(cmd);
    v = constrain(v, -speed_limit, speed_limit); // 安全制限
    motor.controller = MotionControlType::velocity;
    motor.target = v;
}

// ---- 位置制御（deg）----
void doPosition(char *cmd) {
    float deg = atof(cmd);
    float rad = deg * _PI / 180.0;
    rad = constrain(rad, -_PI, _PI); // ±180°
    motor.controller = MotionControlType::angle;
    motor.target = rad;
}

void setup() {
    Serial.begin(115200);

    encoder.init();
    encoder.enableInterrupts(doA, doB, doIndex);
    motor.linkSensor(&encoder);

    driver.voltage_power_supply = 18;
    driver.init();
    motor.linkDriver(&driver);

    motor.torque_controller = TorqueControlType::voltage;

    motor.PID_velocity.P = 1.0;
    motor.PID_velocity.I = 0.0;
    motor.PID_velocity.D = 0;
    motor.PID_velocity.output_ramp = 10;
    motor.LPF_velocity.Tf = 0.01;
    motor.velocity_limit = 500; // rad/s

    motor.P_angle.P = 5.0;

    motor.voltage_limit = 12;

    motor.useMonitoring(Serial);
    motor.init();
    motor.initFOC();

    command.add('V', doVelocity, "velocity rad/s");
    command.add('P', doPosition, "position deg");

    Serial.println("Motor ready.");
    Serial.println("Vx  : velocity [rad/s]");
    Serial.println("Px  : position [deg]");
}

void loop() {
    motor.loopFOC();
    motor.move();
    command.run();
}
