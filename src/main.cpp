#include <Arduino.h>

#include "chirp.h"
#include "echo_simulation.h"
#include "direct_correlation.h"
#include "i2s_output.h"


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