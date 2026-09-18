#include "direct_correlation.h"

#include "config.h"
#include "radar_math.h"

namespace DirectCorrelation
{
    static float correlation[
        Config::CORRELATION_SIZE
    ];


    static Detection detected[
        Config::MAX_DETECTIONS
    ];


    static size_t detectedCount = 0;


    // ==================================================
    // Correlación directa
    // ==================================================

    void calculate(
        const int16_t* chirp,
        const float* receivedSignal
    )
    {
        Serial.println();
        Serial.println(
            "=== CORRELACION DIRECTA ==="
        );


        const unsigned long startTime =
            micros();


        for (size_t lag = 0;
             lag <= Config::MAX_LAG;
             lag++)
        {
            float sum = 0.0f;


            for (size_t n = 0;
                 n < Config::NUM_SAMPLES;
                 n++)
            {
                sum +=
                    static_cast<float>(
                        chirp[n]
                    )
                    *
                    receivedSignal[
                        n + lag
                    ];
            }


            correlation[lag] =
                sum;
        }


        const unsigned long elapsed =
            micros() -
            startTime;


        Serial.println(
            "Correlacion calculada."
        );


        Serial.printf(
            "Retardos evaluados: %u\n",
            static_cast<unsigned>(
                Config::CORRELATION_SIZE
            )
        );


        Serial.printf(
            "Tiempo de calculo: %lu us\n",
            elapsed
        );


        Serial.printf(
            "Tiempo de calculo: %.3f ms\n",
            elapsed /
            1000.0f
        );
    }


    // ==================================================
    // Máximo de correlación
    // ==================================================

    static float findMaximumCorrelation()
    {
        float maximum = 0.0f;


        const size_t minimumLag =
            RadarMath::distanceToSamples(
                Config::MIN_DETECTION_DISTANCE
            );


        for (size_t lag = minimumLag;
             lag <= Config::MAX_LAG;
             lag++)
        {
            if (correlation[lag] >
                maximum)
            {
                maximum =
                    correlation[lag];
            }
        }


        return maximum;
    }


    // ==================================================
    // Detección de picos
    // ==================================================

    void detectEchoes()
    {
        detectedCount = 0;


        const size_t minimumLag =
            RadarMath::distanceToSamples(
                Config::MIN_DETECTION_DISTANCE
            );


        const float maximumCorrelation =
            findMaximumCorrelation();


        const float threshold =
            maximumCorrelation *
            Config::PEAK_THRESHOLD_RATIO;


        Serial.println();
        Serial.println(
            "=== DETECCION DE PICOS ==="
        );


        Serial.printf(
            "Retardo minimo: %u muestras\n",
            static_cast<unsigned>(
                minimumLag
            )
        );


        Serial.printf(
            "Umbral relativo: %.2f\n",
            Config::PEAK_THRESHOLD_RATIO
        );


        for (size_t lag = minimumLag + 1;
             lag < Config::MAX_LAG;
             lag++)
        {
            const float current =
                correlation[lag];


            const bool aboveThreshold =
                current >= threshold;


            const bool localMaximum =
                current >
                    correlation[lag - 1]
                &&
                current >=
                    correlation[lag + 1];


            if (!aboveThreshold ||
                !localMaximum)
            {
                continue;
            }


            if (detectedCount == 0)
            {
                detected[0].lag =
                    lag;

                detected[0].correlationValue =
                    current;

                detected[0].distance =
                    RadarMath::samplesToDistance(
                        lag
                    );

                detectedCount = 1;

                continue;
            }


            Detection &lastDetection =
                detected[
                    detectedCount - 1
                ];


            const size_t separation =
                lag -
                lastDetection.lag;


            if (separation <
                Config::MIN_PEAK_SEPARATION)
            {
                if (current >
                    lastDetection.correlationValue)
                {
                    lastDetection.lag =
                        lag;

                    lastDetection.correlationValue =
                        current;

                    lastDetection.distance =
                        RadarMath::samplesToDistance(
                            lag
                        );
                }

                continue;
            }


            if (detectedCount <
                Config::MAX_DETECTIONS)
            {
                detected[
                    detectedCount
                ].lag =
                    lag;


                detected[
                    detectedCount
                ].correlationValue =
                    current;


                detected[
                    detectedCount
                ].distance =
                    RadarMath::samplesToDistance(
                        lag
                    );


                detectedCount++;
            }
        }
    }


    // ==================================================
    // Mostrar resultados
    // ==================================================

    void showDetectedEchoes()
    {
        Serial.println();
        Serial.println(
            "=== ECOS DETECTADOS ==="
        );


        if (detectedCount == 0)
        {
            Serial.println(
                "No se detectaron ecos."
            );

            return;
        }


        for (size_t i = 0;
             i < detectedCount;
             i++)
        {
            Serial.printf(
                "Eco %u\n",
                static_cast<unsigned>(
                    i + 1
                )
            );


            Serial.printf(
                "Retardo: %u muestras\n",
                static_cast<unsigned>(
                    detected[i].lag
                )
            );


            Serial.printf(
                "Distancia: %.3f m\n",
                detected[i].distance
            );


            Serial.println();
        }
    }


    size_t detectionCount()
    {
        return detectedCount;
    }


    const Detection* detections()
    {
        return detected;
    }
}