#include "i2s_output.h"

#include "driver/i2s.h"

#include "config.h"

namespace I2SOutput
{
    constexpr i2s_port_t I2S_PORT =
        I2S_NUM_0;


    static int16_t stereoBuffer[
        Config::NUM_SAMPLES * 2
    ];


    static bool ready = false;


    // ==================================================
    // Buffer estéreo
    // ==================================================

    void createStereoBuffer(
        const int16_t* chirp
    )
    {
        for (size_t i = 0;
             i < Config::NUM_SAMPLES;
             i++)
        {
            stereoBuffer[
                i * 2
            ] =
                chirp[i];


            stereoBuffer[
                i * 2 + 1
            ] =
                chirp[i];
        }
    }


    size_t bufferSize()
    {
        return sizeof(
            stereoBuffer
        );
    }


    // ==================================================
    // Configuración I2S
    // ==================================================

    bool configure()
    {
        i2s_config_t i2sConfig = {};


        i2sConfig.mode =
            static_cast<i2s_mode_t>(
                I2S_MODE_MASTER |
                I2S_MODE_TX
            );


        i2sConfig.sample_rate =
            Config::SAMPLE_RATE;


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

        i2sConfig.tx_desc_auto_clear =
            true;

        i2sConfig.fixed_mclk = 0;


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

            ready = false;

            return false;
        }


        i2s_pin_config_t pinConfig = {};


        pinConfig.bck_io_num =
            Config::I2S_BCLK;


        pinConfig.ws_io_num =
            Config::I2S_LRCLK;


        pinConfig.data_out_num =
            Config::I2S_DATA;


        pinConfig.data_in_num =
            I2S_PIN_NO_CHANGE;


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

            ready = false;

            return false;
        }


        ready = true;


        Serial.println();
        Serial.println(
            "I2S configurado correctamente."
        );


        return true;
    }


    // ==================================================
    // Transmisión
    // ==================================================

    void transmit()
    {
        if (!ready)
        {
            Serial.println(
                "I2S no esta configurado."
            );

            return;
        }


        size_t bytesWritten = 0;


        const size_t bytesToWrite =
            sizeof(stereoBuffer);


        const esp_err_t result =
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
            Serial.println(
                "=== TRANSMISION I2S ==="
            );


            Serial.printf(
                "Bytes solicitados: %u\n",
                static_cast<unsigned>(
                    bytesToWrite
                )
            );


            Serial.printf(
                "Bytes enviados: %u\n",
                static_cast<unsigned>(
                    bytesWritten
                )
            );


            Serial.printf(
                "Muestras del chirp: %u\n",
                static_cast<unsigned>(
                    Config::NUM_SAMPLES
                )
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
}