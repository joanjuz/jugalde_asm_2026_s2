#pragma once

#include <Arduino.h>

namespace FFTCorrelation
{
    struct Detection
    {
        size_t lag;
        float correlationValue;
        float distance;
    };


    void calculate(
        const int16_t* chirp,
        const float* receivedSignal
    );


    void detectEchoes();

    void showDetectedEchoes();


    unsigned long elapsedMicros();

    size_t detectionCount();

    const Detection* detections();
}