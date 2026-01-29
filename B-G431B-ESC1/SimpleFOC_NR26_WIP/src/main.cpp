#include <Arduino.h>
#include <SimpleFOC.h>

#define SPEED_LIMIT 100 // 速度制限(rad/s)
#define GEAR_RATIO 1.0  // ギア比

// ！！注意！！
//  ここの設定を間違えるとドライバが燃えます
#define VOLTAGE_SUPPLY 12 // 電源電圧
#define VOLTAGE_LIMIT 6   // モータ印加電圧制限（もう少し攻めてもいいかも）

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
    v = constrain(v, -SPEED_LIMIT, SPEED_LIMIT); // 速度制限，削除予定
    motor.controller = MotionControlType::velocity;
    motor.target = v;
}

// 位置制御（deg）
void doPosition(char *cmd) {
    float deg = atof(cmd);
    float rad = deg * _PI / 180.0; // deg to rad
    motor.controller = MotionControlType::angle;
    motor.target = rad * GEAR_RATIO;
}

void setup() {
    Serial.begin(115200);

    encoder.init();
    encoder.enableInterrupts(doA, doB, doIndex);
    motor.linkSensor(&encoder);

    driver.voltage_power_supply = VOLTAGE_SUPPLY;
    motor.voltage_limit = VOLTAGE_LIMIT;

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
