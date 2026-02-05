#include <Arduino.h>
#include <SimpleFOC.h>

// モータ極数
BLDCMotor motor = BLDCMotor(7);

// ドライバ設定
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// 電流センサ設定
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// エンコーダ設定　(A,B相,PPR,Index)
Encoder encoder = Encoder(A_HALL1, A_HALL2, 2048, A_HALL3);

void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
void doIndex() { encoder.handleIndex(); }

//Commander command = Commander(Serial);
//void doTarget(char *cmd) { command.motion(&motor, cmd); }

void setup() {
    encoder.init();
    encoder.enableInterrupts(doA, doB, doIndex);

    motor.linkSensor(&encoder);

    // 電源電圧設定
    driver.voltage_power_supply = 12;

    driver.init();
    motor.linkDriver(&driver);

    currentSense.linkDriver(&driver);
    currentSense.init();
    currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);

    motor.voltage_sensor_align = 1;
    motor.velocity_index_search = 3;

    // モータの電圧の制限
    motor.voltage_limit = 6;
    // モータの速度の制限？
    motor.velocity_limit = 1000;
    // モータの電流の制限
    // motor.current_limit = 40;

    // 以下コントローラ設定、コメントアウトで速度、位置制御の切り替え
    // 速度制御
    motor.controller = MotionControlType::velocity;

    // 位置制御
    //  motor.controller = MotionControlType::angle;

    // トルク制御方式の設定
    motor.torque_controller = TorqueControlType::foc_current;

    // q軸,d軸のPIDゲイン設定（q軸,d軸ってなんですか？）
    motor.PID_current_q.P = motor.PID_current_d.P = 0.1;
    motor.PID_current_q.I = motor.PID_current_d.I = 10;
    motor.PID_current_q.D = motor.PID_current_d.D = 0;

    // 速度制御のPIDゲイン設定
    motor.PID_velocity.P = 0.5;
    motor.PID_velocity.I = 1;
    motor.PID_velocity.D = 0;

    // 速度制御の出力変化速度制限
    motor.PID_velocity.output_ramp = 1000;

    // LPFの設定
    motor.LPF_velocity.Tf = 0.01;

    // 位置制御のPゲイン設定、SimpleFOCの位置制御はP制御のみらしい
    motor.P_angle.P = 9;

    Serial.begin(115200);
    //motor.useMonitoring(Serial);

    motor.init();
    motor.initFOC();
    //command.add('T', doTarget, "target angle");

    //Serial.println(F("Motor ready."));
    //Serial.println(F("Set the target angle using serial terminal:"));
    _delay(1000);
}

void loop() {
    motor.loopFOC();
    motor.move(50);
    // command.run();
}
