#pragma once

#include <Arduino.h>

namespace EchoSimulation
{
    void simulate(
        const int16_t* chirp
    );

    void showInformation();

    const float* data();
}