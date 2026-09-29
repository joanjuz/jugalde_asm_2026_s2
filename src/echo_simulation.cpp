#include "echo_simulation.h"

#include <math.h>

#include "config.h"
#include "radar_math.h"

namespace EchoSimulation
{
    static float receivedSignal[
        Config::RECEIVED_SAMPLES
    ];


    static uint32_t randomState =
        Config::RANDOM_SEED;

    static bool hasSpareGaussian = false;

    static float spareGaussian = 0.0f;


    // ==================================================
    // Generador pseudoaleatorio
    // ==================================================

    static uint32_t nextRandom()
    {
        randomState =
            1664525UL *
            randomState
            +
            1013904223UL;

        return randomState;
    }


    static float randomUniform()
    {
        return
            (
                static_cast<float>(
                    nextRandom()
                )
                +
                1.0f
            )
            /
            4294967297.0f;
    }


    // ==================================================
    // Ruido gaussiano mediante Box-Muller
    // ==================================================

    static float gaussianRandom()
    {
        if (hasSpareGaussian)
        {
            hasSpareGaussian = false;

            return spareGaussian;
        }


        const float u1 =
            randomUniform();

        const float u2 =
            randomUniform();


        const float magnitude =
            sqrtf(
                -2.0f *
                logf(u1)
            );


        const float angle =
            2.0f *
            PI *
            u2;


        const float value1 =
            magnitude *
            cosf(angle);

        const float value2 =
            magnitude *
            sinf(angle);


        spareGaussian =
            value2;

        hasSpareGaussian =
            true;


        return value1;
    }


    static void resetNoiseGenerator()
    {
        randomState =
            Config::RANDOM_SEED;

        hasSpareGaussian =
            false;

        spareGaussian =
            0.0f;
    }


    // ==================================================
    // Simulación
    // ==================================================

    void simulate(
        const int16_t* chirp
    )
    {
        resetNoiseGenerator();


        const float noiseAmplitude =
            Config::NOISE_STD *
            Config::SIGNAL_AMPLITUDE;


        // Primero llenar el buffer con ruido
        for (size_t i = 0;
             i < Config::RECEIVED_SAMPLES;
             i++)
        {
            receivedSignal[i] =
                gaussianRandom() *
                noiseAmplitude;
        }


        // Agregar los ecos
        for (size_t echoIndex = 0;
             echoIndex < Config::NUM_ECHOES;
             echoIndex++)
        {
            const size_t delay =
                RadarMath::distanceToSamples(
                    Config::ECHOES[
                        echoIndex
                    ].distance
                );


            for (size_t n = 0;
                 n < Config::NUM_SAMPLES;
                 n++)
            {
                const size_t destination =
                    n + delay;


                if (destination <
                    Config::RECEIVED_SAMPLES)
                {
                    receivedSignal[
                        destination
                    ]
                    +=
                        static_cast<float>(
                            chirp[n]
                        )
                        *
                        Config::ECHOES[
                            echoIndex
                        ].amplitude;
                }
            }
        }
    }


    // ==================================================
    // Información
    // ==================================================

    void showInformation()
    {
        Serial.println();
        Serial.println(
            "=== ECOS SIMULADOS ==="
        );


        Serial.printf(
            "Ruido gaussiano: sigma = %.2f\n",
            Config::NOISE_STD
        );


        Serial.printf(
            "Desviacion PCM: %.0f\n",
            Config::NOISE_STD *
            Config::SIGNAL_AMPLITUDE
        );


        Serial.printf(
            "Semilla: %lu\n",
            Config::RANDOM_SEED
        );


        Serial.println();


        for (size_t i = 0;
             i < Config::NUM_ECHOES;
             i++)
        {
            const float distance =
                Config::ECHOES[i].distance;


            const float travelTime =
                (
                    2.0f *
                    distance
                )
                /
                Config::SOUND_SPEED;


            const size_t delay =
                RadarMath::distanceToSamples(
                    distance
                );


            Serial.printf(
                "Eco %u: %.2f m | "
                "%.3f ms | "
                "%u muestras | "
                "amplitud %.2f\n",

                static_cast<unsigned>(
                    i + 1
                ),

                distance,

                travelTime *
                1000.0f,

                static_cast<unsigned>(
                    delay
                ),

                Config::ECHOES[i].amplitude
            );
        }


        Serial.printf(
            "Buffer recibido: %u muestras\n",
            static_cast<unsigned>(
                Config::RECEIVED_SAMPLES
            )
        );


        Serial.printf(
            "Memoria de señal recibida: "
            "%u bytes\n",
            static_cast<unsigned>(
                sizeof(receivedSignal)
            )
        );
    }


    const float* data()
    {
        return receivedSignal;
    }
}