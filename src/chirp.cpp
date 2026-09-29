#include "chirp.h"

#include <math.h>

#include "config.h"

namespace Chirp
{
    static int16_t chirp[
        Config::NUM_SAMPLES
    ];


    void generate()
    {
        const float k =
            (
                Config::F_END -
                Config::F_START
            )
            /
            Config::DURATION;


        for (size_t n = 0;
             n < Config::NUM_SAMPLES;
             n++)
        {
            const float t =
                static_cast<float>(n) /
                Config::SAMPLE_RATE;


            const float phase =
                2.0f *
                PI *
                (
                    Config::F_START * t
                    +
                    0.5f *
                    k *
                    t *
                    t
                );


                        // Ventana Hann para reducir lobulos secundarios
            // en la autocorrelacion del chirp.
            //
            // w[n] = 0.5 * (1 - cos(2*pi*n/(N-1)))
            const float window =
                0.5f *
                (
                    1.0f -
                    cosf(
                        2.0f *
                        PI *
                        static_cast<float>(n)
                        /
                        static_cast<float>(
                            Config::NUM_SAMPLES - 1
                        )
                    )
                );


            chirp[n] =
                static_cast<int16_t>(
                    sinf(phase) *
                    window *
                    Config::SIGNAL_AMPLITUDE
                );
        }
    }


    void showInformation()
    {
        Serial.println();
        Serial.println(
            "=== CHIRP GENERADO ==="
        );


        Serial.printf(
            "Frecuencia de muestreo: %lu Hz\n",
            Config::SAMPLE_RATE
        );


        Serial.printf(
            "Duracion: %.1f ms\n",
            Config::DURATION *
            1000.0f
        );


        Serial.printf(
            "Frecuencia inicial: %.0f Hz\n",
            Config::F_START
        );


        Serial.printf(
            "Frecuencia final: %.0f Hz\n",
            Config::F_END
        );


        Serial.printf(
            "Numero de muestras: %u\n",
            static_cast<unsigned>(
                Config::NUM_SAMPLES
            )
        );


        Serial.printf(
            "Memoria usada por chirp: %u bytes\n",
            static_cast<unsigned>(
                sizeof(chirp)
            )
        );
    }


    const int16_t* data()
    {
        return chirp;
    }
}