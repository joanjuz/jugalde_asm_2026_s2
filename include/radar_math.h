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


    inline float samplesToTimeSeconds(
        size_t samples
    )
    {
        return
            static_cast<float>(samples) /
            Config::SAMPLE_RATE;
    }


    inline float samplesToTimeMilliseconds(
        size_t samples
    )
    {
        return
            samplesToTimeSeconds(samples) *
            1000.0f;
    }


    inline float samplesToDistance(
        size_t samples
    )
    {
        const float travelTime =
            samplesToTimeSeconds(samples);

        return
            (
                travelTime *
                Config::SOUND_SPEED
            )
            / 2.0f;
    }
}