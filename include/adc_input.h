#pragma once

#include <Arduino.h>

namespace ADCInput
{
    bool begin();

    bool capture();

    void showInformation();

    const float* data();
}