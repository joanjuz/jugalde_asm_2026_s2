#pragma once

#include <Arduino.h>

namespace Config
{
    // ==================================================
    // Chirp
    // ==================================================

    // ==================================================
    // Salida de diagnostico del radar
    //
    // false = modo demostracion
    // true  = diagnostico detallado
    // ==================================================

    constexpr bool DEBUG_RADAR =
        false;

    constexpr uint32_t SAMPLE_RATE = 48000;

    constexpr float DURATION = 0.002f;

    constexpr float F_START = 3000.0f;
    constexpr float F_END = 8000.0f;

    constexpr float SIGNAL_AMPLITUDE = 18000.0f;

    constexpr size_t NUM_SAMPLES =
        static_cast<size_t>(
            SAMPLE_RATE * DURATION
        );
    // ==================================================
    // Calibración de fondo
    // ==================================================

    constexpr size_t BACKGROUND_CALIBRATION_FRAMES =
        30;


    // Rango en el que buscaremos el objeto real.
    // Por ahora trabajaremos aproximadamente de
    // 30 cm a 130 cm.
    constexpr float TARGET_MIN_DISTANCE =
        0.48f;

    constexpr float TARGET_MAX_DISTANCE =
        0.80f;


    // Cambio mínimo respecto al fondo.
    // 0.04 = 4 % de la correlación normalizada.
    // Después lo ajustaremos con pruebas reales.
    constexpr float BACKGROUND_CHANGE_THRESHOLD =
        0.04f;

    // ==================================================
    // Parámetros físicos
    // ==================================================

    constexpr float SOUND_SPEED = 343.0f;


    // ==================================================
    // Geometria fisica TX / RX
    // ==================================================
    //
    // Separacion centro-a-centro fija entre:
    //
    // - parlante
    // - microfono
    //
    // Medida fisica aproximada:
    //
    //     9.5 cm = 0.095 m
    //
    // Por ahora se usa SOLO para diagnostico.
    // ==================================================

    constexpr float TX_RX_BASELINE =
        0.095f;


    // ==================================================
    // Ruido
    // ==================================================

    constexpr float NOISE_STD = 0.15f;

    constexpr uint32_t RANDOM_SEED = 12345;
    // ==================================================
    // ADC / micrófono
    // ==================================================

    constexpr int ADC_MIC_PIN = 1;


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
    // FFT
    // ==================================================

    // Para correlación lineal se necesita al menos:
    //
    // NUM_SAMPLES + RECEIVED_SAMPLES - 1
    //
    // 960 + 1660 - 1 = 2619
    //
    // La siguiente potencia de 2 es 4096.
    constexpr size_t FFT_SIZE = 4096;

    static_assert(
        FFT_SIZE >=
            NUM_SAMPLES +
            RECEIVED_SAMPLES -
            1,
        "FFT_SIZE es demasiado pequeno"
    );


    // ==================================================
    // Detección
    // ==================================================

    constexpr float MIN_DETECTION_DISTANCE =
        0.0f;

    constexpr float PEAK_THRESHOLD_RATIO =
        0.20f;

    constexpr size_t MIN_PEAK_SEPARATION =
        20;

    constexpr size_t MAX_DETECTIONS =
        10;


    // ==================================================
    // Pines I2S
    // ==================================================

    constexpr int I2S_BCLK = 4;
    constexpr int I2S_LRCLK = 5;
    constexpr int I2S_DATA = 6;
}