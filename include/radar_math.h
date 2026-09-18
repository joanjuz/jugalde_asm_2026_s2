#pragma once

#include <Arduino.h>
#include <math.h>

#include "config.h"

namespace RadarMath
{
    inline size_t distanceToSamples(
        float distance
    )
    {
        const float travelTime =
            (2.0f * distance) /
            Config::SOUND_SPEED;

        return static_cast<size_t>(
            roundf(
                travelTime *
                Config::SAMPLE_RATE
            )
        );
    }


    inline float samplesToDistance(
        size_t samples
    )
    {
        const float travelTime =
            static_cast<float>(samples) /
            Config::SAMPLE_RATE;

        return
            (
                travelTime *
                Config::SOUND_SPEED
            )
            / 2.0f;
    }
}