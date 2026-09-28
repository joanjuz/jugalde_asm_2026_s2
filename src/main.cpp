#include <Arduino.h>

#include "adc_input.h"
#include "chirp.h"
#include "fft_correlation.h"
#include "i2s_output.h"
#include "background_calibration.h"


static bool captureRadarFrame()
{
    if (!ADCInput::startCapture())
    {
        return false;
    }


    I2SOutput::transmit();


    if (!ADCInput::finishCapture())
    {
        return false;
    }


    FFTCorrelation::calculate(
        Chirp::data(),
        ADCInput::data()
    );


    FFTCorrelation::detectEchoes();


    return true;
}


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


    // ==================================================
    // 1. Generar chirp
    // ==================================================

    Chirp::generate();

    Chirp::showInformation();


    // ==================================================
    // 2. Crear buffer I2S
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
    // 3. Configurar I2S
    // ==================================================

    if (!I2SOutput::configure())
    {
        Serial.println(
            "No se pudo configurar I2S."
        );

        return;
    }


    // ==================================================
    // 4. Configurar ADC DMA
    // ==================================================

    if (!ADCInput::begin())
    {
        Serial.println(
            "No se pudo configurar ADC."
        );

        return;
    }


    Serial.println();
    Serial.println(
        "Radar fisico listo."
    );
    Serial.println();
    Serial.println(
        "================================"
    );
    Serial.println(
        "RETIRE TABLA Y OBJETOS DEL FRENTE"
    );
    Serial.println(
        "Calibracion comenzara en 5 segundos."
    );
    Serial.println(
        "================================"
    );


    delay(5000);


    BackgroundCalibration::reset();


    size_t attempts =
        0;


    while (
        BackgroundCalibration::capturedFrames()
        <
        BackgroundCalibration::requiredFrames()
        &&
        attempts < 30
    )
    {
        attempts++;


        if (captureRadarFrame())
        {
            BackgroundCalibration::addCurrentFrame();
        }


        delay(250);
    }


    BackgroundCalibration::finalize();


    Serial.println();
    Serial.println(
        "Coloque ahora el objeto."
    );


    delay(3000);
    }


void loop()
{
    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "=== NUEVA MEDICION FISICA ==="
    );

    Serial.println(
        "================================"
    );


    if (captureRadarFrame())
    {
        ADCInput::showInformation();

        FFTCorrelation::showDetectedEchoes();

        BackgroundCalibration::analyzeCurrentFrame();
    }


    delay(1000);
}