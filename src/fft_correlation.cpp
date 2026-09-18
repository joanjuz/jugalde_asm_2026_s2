#include "fft_correlation.h"

#include <math.h>

#include "config.h"
#include "radar_math.h"


namespace FFTCorrelation
{
    // ==================================================
    // Buffers FFT
    // ==================================================

    // FFT del chirp
    static float realChirp[
        Config::FFT_SIZE
    ];

    static float imagChirp[
        Config::FFT_SIZE
    ];


    // FFT de la señal recibida
    static float realReceived[
        Config::FFT_SIZE
    ];

    static float imagReceived[
        Config::FFT_SIZE
    ];


    // Solo necesitamos conservar los retardos útiles
    static float correlation[
        Config::CORRELATION_SIZE
    ];


    // ==================================================
    // Resultados
    // ==================================================

    static Detection detected[
        Config::MAX_DETECTIONS
    ];

    static size_t detectedCount = 0;

    static unsigned long lastElapsed = 0;


    // ==================================================
    // FFT radix-2 in-place
    // ==================================================

    static void fft(
        float* real,
        float* imag,
        size_t size,
        bool inverse
    )
    {
        // ----------------------------------------------
        // Reordenamiento bit-reversal
        // ----------------------------------------------

        for (size_t i = 1, j = 0;
             i < size;
             i++)
        {
            size_t bit =
                size >> 1;


            while (j & bit)
            {
                j ^= bit;

                bit >>= 1;
            }


            j ^= bit;


            if (i < j)
            {
                float temp =
                    real[i];

                real[i] =
                    real[j];

                real[j] =
                    temp;


                temp =
                    imag[i];

                imag[i] =
                    imag[j];

                imag[j] =
                    temp;
            }
        }


        // ----------------------------------------------
        // Etapas Cooley-Tukey
        // ----------------------------------------------

        for (size_t length = 2;
             length <= size;
             length <<= 1)
        {
            const float angle =
                (
                    inverse
                    ? 2.0f
                    : -2.0f
                )
                *
                PI
                /
                static_cast<float>(
                    length
                );


            const float stepReal =
                cosf(angle);

            const float stepImag =
                sinf(angle);


            const size_t halfLength =
                length >> 1;


            for (size_t block = 0;
                 block < size;
                 block += length)
            {
                float wReal = 1.0f;
                float wImag = 0.0f;


                for (size_t j = 0;
                     j < halfLength;
                     j++)
                {
                    const size_t evenIndex =
                        block + j;

                    const size_t oddIndex =
                        evenIndex +
                        halfLength;


                    const float oddReal =
                        real[oddIndex] *
                            wReal
                        -
                        imag[oddIndex] *
                            wImag;


                    const float oddImag =
                        real[oddIndex] *
                            wImag
                        +
                        imag[oddIndex] *
                            wReal;


                    const float evenReal =
                        real[evenIndex];

                    const float evenImag =
                        imag[evenIndex];


                    real[evenIndex] =
                        evenReal +
                        oddReal;

                    imag[evenIndex] =
                        evenImag +
                        oddImag;


                    real[oddIndex] =
                        evenReal -
                        oddReal;

                    imag[oddIndex] =
                        evenImag -
                        oddImag;


                    // Actualizar factor de giro
                    const float nextWReal =
                        wReal *
                            stepReal
                        -
                        wImag *
                            stepImag;


                    const float nextWImag =
                        wReal *
                            stepImag
                        +
                        wImag *
                            stepReal;


                    wReal =
                        nextWReal;

                    wImag =
                        nextWImag;
                }
            }
        }


        // ----------------------------------------------
        // Normalización de la IFFT
        // ----------------------------------------------

        if (inverse)
        {
            const float scale =
                1.0f /
                static_cast<float>(
                    size
                );


            for (size_t i = 0;
                 i < size;
                 i++)
            {
                real[i] *= scale;

                imag[i] *= scale;
            }
        }
    }


    // ==================================================
    // Correlación mediante FFT
    // ==================================================

    void calculate(
        const int16_t* chirp,
        const float* receivedSignal
    )
    {
        Serial.println();
        Serial.println(
            "=== CORRELACION FFT ==="
        );


        const unsigned long startTime =
            micros();


        // ----------------------------------------------
        // 1. Zero padding
        // ----------------------------------------------

        for (size_t i = 0;
             i < Config::FFT_SIZE;
             i++)
        {
            realChirp[i] = 0.0f;
            imagChirp[i] = 0.0f;

            realReceived[i] = 0.0f;
            imagReceived[i] = 0.0f;
        }


        // ----------------------------------------------
        // 2. Copiar chirp
        // ----------------------------------------------

        for (size_t i = 0;
             i < Config::NUM_SAMPLES;
             i++)
        {
            realChirp[i] =
                static_cast<float>(
                    chirp[i]
                );
        }


        // ----------------------------------------------
        // 3. Copiar señal recibida
        // ----------------------------------------------

        for (size_t i = 0;
             i < Config::RECEIVED_SAMPLES;
             i++)
        {
            realReceived[i] =
                receivedSignal[i];
        }


        // ----------------------------------------------
        // 4. FFT de ambas señales
        // ----------------------------------------------

        fft(
            realChirp,
            imagChirp,
            Config::FFT_SIZE,
            false
        );


        fft(
            realReceived,
            imagReceived,
            Config::FFT_SIZE,
            false
        );


        // ----------------------------------------------
        // 5. conj(X) * Y
        //
        // Correlación:
        //
        // R = IFFT(conj(FFT(x)) * FFT(y))
        // ----------------------------------------------

        for (size_t i = 0;
             i < Config::FFT_SIZE;
             i++)
        {
            const float xReal =
                realChirp[i];

            const float xImag =
                imagChirp[i];

            const float yReal =
                realReceived[i];

            const float yImag =
                imagReceived[i];


            realChirp[i] =
                xReal * yReal
                +
                xImag * yImag;


            imagChirp[i] =
                xReal * yImag
                -
                xImag * yReal;
        }


        // ----------------------------------------------
        // 6. IFFT
        // ----------------------------------------------

        fft(
            realChirp,
            imagChirp,
            Config::FFT_SIZE,
            true
        );


        // ----------------------------------------------
        // 7. Guardar retardos útiles
        // ----------------------------------------------

        for (size_t lag = 0;
             lag <= Config::MAX_LAG;
             lag++)
        {
            correlation[lag] =
                realChirp[lag];
        }


        lastElapsed =
            micros() -
            startTime;


        Serial.println(
            "Correlacion FFT calculada."
        );


        Serial.printf(
            "Tamano FFT: %u\n",
            static_cast<unsigned>(
                Config::FFT_SIZE
            )
        );


        Serial.printf(
            "Retardos utiles: %u\n",
            static_cast<unsigned>(
                Config::CORRELATION_SIZE
            )
        );


        Serial.printf(
            "Tiempo de calculo: %lu us\n",
            lastElapsed
        );


        Serial.printf(
            "Tiempo de calculo: %.3f ms\n",
            lastElapsed /
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
            "=== DETECCION FFT ==="
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


        for (size_t lag =
                 minimumLag + 1;
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


            Detection& lastDetection =
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
            "=== ECOS DETECTADOS POR FFT ==="
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
                "Tiempo de vuelo: %.3f ms\n",
                RadarMath::samplesToTimeMilliseconds(
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


    unsigned long elapsedMicros()
    {
        return lastElapsed;
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