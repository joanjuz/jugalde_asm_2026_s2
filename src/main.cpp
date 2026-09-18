#include <Arduino.h>
#include <math.h>
#include "driver/i2s.h"

// ======================================================
// Parámetros del chirp
// ======================================================

constexpr uint32_t SAMPLE_RATE = 48000;
constexpr float DURATION = 0.020f;
constexpr float F_START = 3000.0f;
constexpr float F_END = 8000.0f;

constexpr size_t NUM_SAMPLES =
    static_cast<size_t>(SAMPLE_RATE * DURATION);

// ======================================================
// Pines I2S
// ======================================================

constexpr int I2S_BCLK = 4;
constexpr int I2S_LRCLK = 5;
constexpr int I2S_DATA = 6;

// Usaremos el periférico I2S 0
constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

// ======================================================
// Buffers
// ======================================================

// Chirp mono
int16_t chirp[NUM_SAMPLES];

// Buffer estéreo.
//
// Por cada muestra mono tenemos:
//
// [Left][Right]
//
// 960 muestras mono → 1920 valores int16_t
//
int16_t stereoBuffer[NUM_SAMPLES * 2];


// ======================================================
// Generación del chirp
// ======================================================

void generateChirp()
{
    const float k =
        (F_END - F_START) / DURATION;

    for (size_t n = 0; n < NUM_SAMPLES; n++)
    {
        const float t =
            static_cast<float>(n) / SAMPLE_RATE;

        const float phase =
            2.0f * PI *
            (
                F_START * t
                + 0.5f * k * t * t
            );

        chirp[n] =
            static_cast<int16_t>(
                sinf(phase) * 30000.0f
            );
    }
}


// ======================================================
// Conversión mono → estéreo
// ======================================================

void createStereoBuffer()
{
    for (size_t i = 0; i < NUM_SAMPLES; i++)
    {
        stereoBuffer[i * 2] = chirp[i];       // Left
        stereoBuffer[i * 2 + 1] = chirp[i];   // Right
    }
}


// ======================================================
// Configuración I2S
// ======================================================

void configureI2S()
{
    i2s_config_t i2sConfig = {};

    i2sConfig.mode =
        static_cast<i2s_mode_t>(
            I2S_MODE_MASTER |
            I2S_MODE_TX
        );

    i2sConfig.sample_rate = SAMPLE_RATE;

    i2sConfig.bits_per_sample =
        I2S_BITS_PER_SAMPLE_16BIT;

    i2sConfig.channel_format =
        I2S_CHANNEL_FMT_RIGHT_LEFT;

    i2sConfig.communication_format =
        I2S_COMM_FORMAT_STAND_I2S;

    i2sConfig.intr_alloc_flags =
        ESP_INTR_FLAG_LEVEL1;

    i2sConfig.dma_buf_count = 8;
    i2sConfig.dma_buf_len = 128;

    i2sConfig.use_apll = false;
    i2sConfig.tx_desc_auto_clear = true;
    i2sConfig.fixed_mclk = 0;


    // Instalar driver
    esp_err_t result =
        i2s_driver_install(
            I2S_PORT,
            &i2sConfig,
            0,
            nullptr
        );

    if (result != ESP_OK)
    {
        Serial.printf(
            "Error instalando I2S: %d\n",
            result
        );

        return;
    }


    // Configurar pines
    i2s_pin_config_t pinConfig = {};

    pinConfig.bck_io_num = I2S_BCLK;
    pinConfig.ws_io_num = I2S_LRCLK;
    pinConfig.data_out_num = I2S_DATA;
    pinConfig.data_in_num = I2S_PIN_NO_CHANGE;


    result =
        i2s_set_pin(
            I2S_PORT,
            &pinConfig
        );

    if (result != ESP_OK)
    {
        Serial.printf(
            "Error configurando pines I2S: %d\n",
            result
        );

        return;
    }


    Serial.println("I2S configurado correctamente.");
}


// ======================================================
// Transmisión
// ======================================================

void transmitChirp()
{
    size_t bytesWritten = 0;

    const size_t bytesToWrite =
        sizeof(stereoBuffer);

    esp_err_t result =
        i2s_write(
            I2S_PORT,
            stereoBuffer,
            bytesToWrite,
            &bytesWritten,
            portMAX_DELAY
        );


    if (result == ESP_OK)
    {
        Serial.println();
        Serial.println("=== TRANSMISION I2S ===");

        Serial.printf(
            "Bytes solicitados: %u\n",
            bytesToWrite
        );

        Serial.printf(
            "Bytes enviados: %u\n",
            bytesWritten
        );

        Serial.printf(
            "Muestras del chirp: %u\n",
            NUM_SAMPLES
        );
    }
    else
    {
        Serial.printf(
            "Error transmitiendo I2S: %d\n",
            result
        );
    }
}


// ======================================================
// Setup
// ======================================================

void setup()
{
    Serial.begin(115200);

    delay(2000);

    Serial.println();
    Serial.println("ESP32-S3 - Radar acustico");
    Serial.println("=========================");

    generateChirp();

    Serial.printf(
        "Chirp generado: %u muestras\n",
        NUM_SAMPLES
    );

    createStereoBuffer();

    Serial.printf(
        "Buffer I2S: %u bytes\n",
        sizeof(stereoBuffer)
    );

    configureI2S();

    transmitChirp();
}


// ======================================================
// Loop
// ======================================================

void loop()
{
    delay(5000);

    Serial.println();
    Serial.println("Transmitiendo chirp nuevamente...");

    transmitChirp();
}