#pragma once

#include <Arduino.h>

namespace I2SOutput
{
    void createStereoBuffer(
        const int16_t* chirp
    );

    bool configure();

    void transmit();

    size_t bufferSize();
}