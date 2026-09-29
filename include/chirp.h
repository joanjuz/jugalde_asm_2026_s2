#pragma once

#include <Arduino.h>

namespace Chirp
{
    void generate();

    void showInformation();

    const int16_t* data();
}