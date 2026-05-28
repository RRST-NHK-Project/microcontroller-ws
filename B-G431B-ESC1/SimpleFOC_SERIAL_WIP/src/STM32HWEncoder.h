#pragma once

#if defined(_STM32_DEF_) || defined(ARDUINO_ARCH_STM32)

#include "stm32g4xx_hal.h"
#include <SimpleFOC.h>

class STM32HWEncoder : public Sensor {

public:
    STM32HWEncoder(unsigned int _ppr,
                   int pinA,
                   int pinB,
                   int pinI = -1);

    void init() override;

    void update() override {}

    float getSensorAngle() override;

    float getVelocity();

    int32_t getCount();

    int needsSearch() override;

protected:
    int hasIndex();

    PinName _pinA;
    PinName _pinB;
    PinName _pinI;

    TIM_HandleTypeDef encoder_handle;

    int cpr;

    bool index_found;
};

#endif