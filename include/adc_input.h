#pragma once

#include <Arduino.h>

namespace ADCInput
{
    bool begin();

    // Inicia el ADC DMA pero no espera la captura.
    bool startCapture();

    // Espera a completar la captura y procesa el buffer.
    bool finishCapture();

    // Se conserva para pruebas independientes.
    bool capture();

    void showInformation();

    const float* data();
}