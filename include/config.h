#pragma once

#include <Arduino.h>

namespace Config
{
    // ==================================================
    // Chirp
    // ==================================================

    constexpr uint32_t SAMPLE_RATE = 48000;

    constexpr float DURATION = 0.020f;

    constexpr float F_START = 3000.0f;
    constexpr float F_END = 8000.0f;

    constexpr float SIGNAL_AMPLITUDE = 30000.0f;

    constexpr size_t NUM_SAMPLES =
        static_cast<size_t>(
            SAMPLE_RATE * DURATION
        );


    // ==================================================
    // Parámetros físicos
    // ==================================================

    constexpr float SOUND_SPEED = 343.0f;


    // ==================================================
    // Ruido
    // ==================================================

    constexpr float NOISE_STD = 0.15f;

    constexpr uint32_t RANDOM_SEED = 12345;


    // ==================================================
    // Ecos simulados
    // ==================================================

    struct Echo
    {
        float distance;
        float amplitude;
    };

    constexpr Echo ECHOES[] =
    {
        {1.0f, 0.7f},
        {1.5f, 0.5f},
        {2.2f, 0.3f}
    };

    constexpr size_t NUM_ECHOES =
        sizeof(ECHOES) /
        sizeof(ECHOES[0]);


    // ==================================================
    // Buffers
    // ==================================================

    constexpr size_t MAX_DELAY = 700;

    constexpr size_t RECEIVED_SAMPLES =
        NUM_SAMPLES + MAX_DELAY;

    constexpr size_t MAX_LAG =
        RECEIVED_SAMPLES -
        NUM_SAMPLES;

    constexpr size_t CORRELATION_SIZE =
        MAX_LAG + 1;


    // ==================================================
    // Detección
    // ==================================================

    constexpr float MIN_DETECTION_DISTANCE =
        0.20f;

    constexpr float PEAK_THRESHOLD_RATIO =
        0.30f;

    constexpr size_t MIN_PEAK_SEPARATION =
        50;

    constexpr size_t MAX_DETECTIONS =
        10;


    // ==================================================
    // Pines I2S
    // ==================================================

    constexpr int I2S_BCLK = 4;
    constexpr int I2S_LRCLK = 5;
    constexpr int I2S_DATA = 6;
}