#include <Arduino.h>
#include <SimpleFOC.h>

// モータ極数
BLDCMotor motor = BLDCMotor(7);

// ドライバ設定
BLDCDriver6PWM driver = BLDCDriver6PWM(
    A_PHASE_UH, A_PHASE_UL,
    A_PHASE_VH, A_PHASE_VL,
    A_PHASE_WH, A_PHASE_WL);

// エンコーダ設定　(A,B相,PPR,Index)
Encoder encoder = Encoder(A_HALL1, A_HALL2, 2048, A_HALL3);

void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
void doIndex() { encoder.handleIndex(); }

// シリアルでコマンドを受け付ける
Commander command = Commander(Serial);

// 速度制御（rad/s）
void doVelocity(char *cmd) {
    float v = atof(cmd);
    // 入力値の制限(rad/s)
    int speed_limit = 100;
    v = constrain(v, -speed_limit, speed_limit);
    motor.controller = MotionControlType::velocity;
    motor.target = v;
}

// 位置制御（deg）
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

    // ！！注意！！
    //  ここの設定を間違えるとドライバが燃えます
    driver.voltage_power_supply = 24;
    motor.voltage_limit = 12;

    driver.init();
    motor.linkDriver(&driver);

    motor.torque_controller = TorqueControlType::voltage;

    motor.PID_velocity.P = 1.0;
    motor.PID_velocity.I = 0.0;
    motor.PID_velocity.D = 0;
    motor.PID_velocity.output_ramp = 10;
    motor.LPF_velocity.Tf = 0.01;
    motor.velocity_limit = 40; // rad/s

    motor.P_angle.P = 5.0;

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
