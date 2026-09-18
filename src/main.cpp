#include <Arduino.h>
#include "fft_correlation.h"
#include "chirp.h"
#include "echo_simulation.h"
#include "direct_correlation.h"
#include "i2s_output.h"
#include "adc_input.h"


void setup()
{
    Serial.begin(115200);

    delay(2000);


    Serial.println();

    Serial.println(
        "ESP32-S3 - Radar acustico"
    );

    Serial.println(
        "========================="
    );
    if (ADCInput::begin())
{
    if (ADCInput::capture())
    {
        ADCInput::showInformation();
    }
}


    // ==================================================
    // 1. Generar chirp
    // ==================================================

    Chirp::generate();

    Chirp::showInformation();


    // ==================================================
    // 2. Preparar transmisión I2S
    // ==================================================

    I2SOutput::createStereoBuffer(
        Chirp::data()
    );


    Serial.printf(
        "Buffer I2S: %u bytes\n",
        static_cast<unsigned>(
            I2SOutput::bufferSize()
        )
    );


    // ==================================================
    // 3. Simular recepción
    // ==================================================

    EchoSimulation::simulate(
        Chirp::data()
    );


    EchoSimulation::showInformation();


    // ==================================================
    // 4. Correlación directa
    // ==================================================

    DirectCorrelation::calculate(
        Chirp::data(),
        EchoSimulation::data()
    );


    // ==================================================
    // 5. Detectar ecos
    // ==================================================

    DirectCorrelation::detectEchoes();

    DirectCorrelation::showDetectedEchoes();
    // ==================================================
    // 5. Correlación mediante FFT
    // ==================================================

    FFTCorrelation::calculate(
        Chirp::data(),
        EchoSimulation::data()
    );

    FFTCorrelation::detectEchoes();

    FFTCorrelation::showDetectedEchoes();


    // ==================================================
    // 6. Comparación
    // ==================================================

    Serial.println();
    Serial.println(
        "=== COMPARACION DE CORRELACION ==="
    );


    const unsigned long directTime =
        DirectCorrelation::elapsedMicros();

    const unsigned long fftTime =
        FFTCorrelation::elapsedMicros();


    Serial.printf(
        "Directa: %.3f ms\n",
        directTime / 1000.0f
    );

    Serial.printf(
        "FFT: %.3f ms\n",
        fftTime / 1000.0f
    );


    if (fftTime > 0)
    {
        Serial.printf(
            "Relacion Directa/FFT: %.2fx\n",
            static_cast<float>(
                directTime
            )
            /
            static_cast<float>(
                fftTime
            )
        );
    }


    // Comprobar que ambos métodos detectaron
    // los mismos retardos.
    bool sameDetections =
        DirectCorrelation::detectionCount()
        ==
        FFTCorrelation::detectionCount();


    if (sameDetections)
    {
        const auto* direct =
            DirectCorrelation::detections();

        const auto* fft =
            FFTCorrelation::detections();


        for (size_t i = 0;
            i <
            DirectCorrelation::detectionCount();
            i++)
        {
            if (direct[i].lag !=
                fft[i].lag)
            {
                sameDetections = false;

                break;
            }
        }
    }


    Serial.printf(
        "Retardos coinciden: %s\n",
        sameDetections
            ? "SI"
            : "NO"
    );


    // ==================================================
    // 6. Configurar I2S
    // ==================================================

    I2SOutput::configure();


    // ==================================================
    // 7. Transmitir chirp
    // ==================================================

    I2SOutput::transmit();
}


void loop()
{
    delay(5000);


    Serial.println();

    Serial.println(
        "Transmitiendo chirp nuevamente..."
    );


    I2SOutput::transmit();
}