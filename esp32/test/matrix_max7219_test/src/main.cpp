#include <LedControl.h>

LedControl lc = LedControl(2, 4, 3, 1); // DIN, CLK, CS, モジュール数

enum RobotState {
    STATE_IDLE,
    STATE_RUN,
    STATE_ERROR,
};

const byte ICON_IDLE[8] = {
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
};

const byte ICON_RUN[8] = {
    B00000000,
    B00011000,
    B00011100,
    B11111110,
    B11111110,
    B00011100,
    B00011000,
    B00000000,
};

const byte ICON_ERROR[8] = {
    B10000001,
    B01000010,
    B00100100,
    B00011000,
    B00011000,
    B00100100,
    B01000010,
    B10000001,
};

RobotState currentState = STATE_IDLE;
unsigned long lastDrawMs = 0;
bool errorBlinkOn = true;

void drawPattern(const byte pattern[8]) {
    for (int row = 0; row < 8; row++) {
        lc.setRow(0, row, pattern[row]);
    }
}

void setRobotState(RobotState state) {
    currentState = state;
}

void updateDisplay(unsigned long nowMs) {
    if (nowMs - lastDrawMs < 100) {
        return;
    }
    lastDrawMs = nowMs;

    if (currentState == STATE_ERROR) {
        errorBlinkOn = !errorBlinkOn;
        if (errorBlinkOn) {
            drawPattern(ICON_ERROR);
        } else {
            lc.clearDisplay(0);
        }
        return;
    }

    if (currentState == STATE_IDLE) {
        drawPattern(ICON_IDLE);
    } else {
        drawPattern(ICON_RUN);
    }
}

void updateDemoState(unsigned long nowMs) {
    // デモ用: 3秒ごとに IDLE -> RUN -> ERROR を循環
    unsigned long phase = (nowMs / 3000) % 3;
    if (phase == 0) {
        setRobotState(STATE_IDLE);
    } else if (phase == 1) {
        setRobotState(STATE_RUN);
    } else {
        setRobotState(STATE_ERROR);
    }
}

void setup() {
    for (int i = 0; i < lc.getDeviceCount(); i++) {
        lc.shutdown(i, false); // 表示する(お約束)
        lc.setIntensity(i, 8); // 明るさ(0..15)
        lc.clearDisplay(i);    // いったん全消去
    }

    drawPattern(ICON_IDLE);
}

void loop() {
    unsigned long now = millis();
    updateDemoState(now);
    updateDisplay(now);
}