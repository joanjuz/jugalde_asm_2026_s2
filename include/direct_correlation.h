#pragma once

#include <Arduino.h>

namespace DirectCorrelation
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


    size_t detectionCount();


    const Detection* detections();
}