#include "adc_input.h"

#include <Arduino.h>

#include "driver/adc.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "soc/soc_caps.h"

#include "config.h"

namespace ADCInput
{
    // ==================================================
    // Configuración del ADC
    // ==================================================

    constexpr adc1_channel_t ADC_CHANNEL =
        ADC1_CHANNEL_0;

    constexpr adc_atten_t ADC_ATTENUATION =
        ADC_ATTEN_DB_11;

    // ESP32-S3 entrega 4 bytes por conversión
    // en el formato TYPE2.
    constexpr size_t ADC_BYTES_PER_SAMPLE =
        sizeof(adc_digi_output_data_t);

    // Tamaño de cada bloque generado por DMA.
    //
    // Debe ser múltiplo del tamaño de una
    // conversión.
    constexpr size_t DMA_FRAME_BYTES = 256;

    // Buffer interno del driver.
    //
    // 8192 bytes son suficientes para almacenar
    // más de las 1660 muestras que necesitamos.
    constexpr size_t DMA_STORE_BYTES = 8192;

    // Buffer temporal usado para extraer datos
    // desde el DMA.
    constexpr size_t DMA_READ_BYTES = 512;

    constexpr uint8_t ADC_DMA_UNIT = 0;


    // ==================================================
    // Buffers y estado
    // ==================================================

    static float adcBuffer[
        Config::RECEIVED_SAMPLES
    ];

    alignas(4)
    static uint8_t dmaReadBuffer[
        DMA_READ_BYTES
    ];


    static bool initialized = false;

    static bool captureSuccessful = false;

    static uint64_t captureTimeUs = 0;

    static size_t samplesCaptured = 0;

    static float dcOffset = 0.0f;

    static uint16_t rawMinimum = 0;

    static uint16_t rawMaximum = 0;


    // ==================================================
    // Inicialización
    // ==================================================

    bool begin()
    {
        if (initialized)
        {
            return true;
        }


        // ------------------------------------------------
        // Configurar almacenamiento DMA
        // ------------------------------------------------

        adc_digi_init_config_t initConfig = {};

        initConfig.max_store_buf_size =
            DMA_STORE_BYTES;

        initConfig.conv_num_each_intr =
            DMA_FRAME_BYTES;

        initConfig.adc1_chan_mask =
            1UL << ADC_CHANNEL;

        initConfig.adc2_chan_mask =
            0;


        esp_err_t result =
            adc_digi_initialize(
                &initConfig
            );


        if (result != ESP_OK)
        {
            Serial.printf(
                "Error inicializando ADC DMA: %s\n",
                esp_err_to_name(result)
            );

            return false;
        }


        // ------------------------------------------------
        // Patrón de conversión
        // ------------------------------------------------

        adc_digi_pattern_config_t pattern = {};

        pattern.atten =
            ADC_ATTENUATION;

        pattern.channel =
            ADC_CHANNEL;

        pattern.unit = ADC_DMA_UNIT;

        pattern.bit_width =
            SOC_ADC_DIGI_MAX_BITWIDTH;


        // ------------------------------------------------
        // Configuración del controlador digital
        // ------------------------------------------------

        adc_digi_configuration_t config = {};

        // No queremos detenernos después de solo
        // 255 conversiones.
        config.conv_limit_en = false;

        config.conv_limit_num = 255;

        config.pattern_num = 1;

        config.adc_pattern =
            &pattern;

        config.sample_freq_hz =
            Config::SAMPLE_RATE;

        config.conv_mode =
            ADC_CONV_SINGLE_UNIT_1;

        // ESP32-S3 usa TYPE2 para DMA.
        config.format =
            ADC_DIGI_OUTPUT_FORMAT_TYPE2;


        result =
            adc_digi_controller_configure(
                &config
            );


        if (result != ESP_OK)
        {
            Serial.printf(
                "Error configurando ADC DMA: %s\n",
                esp_err_to_name(result)
            );

            adc_digi_deinitialize();

            return false;
        }


        initialized = true;


        Serial.println();
        Serial.println(
            "ADC continuo con DMA configurado."
        );

        Serial.printf(
            "GPIO ADC: %d\n",
            Config::ADC_MIC_PIN
        );

        Serial.printf(
            "Canal ADC: ADC1_CHANNEL_0\n"
        );

        Serial.printf(
            "Frecuencia configurada: %lu Hz\n",
            Config::SAMPLE_RATE
        );

        Serial.printf(
            "Resolucion DMA: %u bits\n",
            static_cast<unsigned>(
                SOC_ADC_DIGI_MAX_BITWIDTH
            )
        );


        return true;
    }


    // ==================================================
    // Captura mediante DMA
    // ==================================================

    bool capture()
    {
        captureSuccessful = false;

        samplesCaptured = 0;

        captureTimeUs = 0;

        dcOffset = 0.0f;


        if (!initialized)
        {
            Serial.println(
                "ADC DMA no inicializado."
            );

            return false;
        }


        // ------------------------------------------------
        // Iniciar conversión continua
        // ------------------------------------------------

        const uint64_t startTime =
            esp_timer_get_time();


        esp_err_t result =
            adc_digi_start();


        if (result != ESP_OK)
        {
            Serial.printf(
                "Error iniciando ADC DMA: %s\n",
                esp_err_to_name(result)
            );

            return false;
        }


        rawMinimum = 0xFFFF;

        rawMaximum = 0;


        // ------------------------------------------------
        // Leer hasta obtener todas las muestras
        // ------------------------------------------------

        while (
            samplesCaptured
            <
            Config::RECEIVED_SAMPLES
        )
        {
            uint32_t bytesRead = 0;


            result =
                adc_digi_read_bytes(
                    dmaReadBuffer,
                    sizeof(dmaReadBuffer),
                    &bytesRead,
                    100
                );


            if (result == ESP_ERR_TIMEOUT)
            {
                continue;
            }


            if (
                result != ESP_OK
                &&
                result != ESP_ERR_INVALID_STATE
            )
            {
                Serial.printf(
                    "Error leyendo ADC DMA: %s\n",
                    esp_err_to_name(result)
                );

                adc_digi_stop();

                return false;
            }


            // --------------------------------------------
            // Cada resultado ocupa 4 bytes en ESP32-S3
            // --------------------------------------------

            for (
                size_t offset = 0;
                offset + ADC_BYTES_PER_SAMPLE
                    <= bytesRead;
                offset += ADC_BYTES_PER_SAMPLE
            )
            {
                if (
                    samplesCaptured
                    >=
                    Config::RECEIVED_SAMPLES
                )
                {
                    break;
                }


                const adc_digi_output_data_t*
                    sample =
                    reinterpret_cast<
                        const adc_digi_output_data_t*
                    >(
                        dmaReadBuffer + offset
                    );


                const uint16_t rawValue =
                    static_cast<uint16_t>(
                        sample->type2.data
                    );


                adcBuffer[
                    samplesCaptured
                ] =
                    static_cast<float>(
                        rawValue
                    );


                if (
                    rawValue
                    <
                    rawMinimum
                )
                {
                    rawMinimum =
                        rawValue;
                }


                if (
                    rawValue
                    >
                    rawMaximum
                )
                {
                    rawMaximum =
                        rawValue;
                }


                samplesCaptured++;
            }
        }


        // ------------------------------------------------
        // Detener ADC
        // ------------------------------------------------

        const esp_err_t stopResult =
            adc_digi_stop();


        captureTimeUs =
            esp_timer_get_time()
            -
            startTime;


        if (stopResult != ESP_OK)
        {
            Serial.printf(
                "Error deteniendo ADC DMA: %s\n",
                esp_err_to_name(stopResult)
            );

            return false;
        }


        // ------------------------------------------------
        // Calcular nivel DC
        // ------------------------------------------------

        float sum = 0.0f;


        for (
            size_t i = 0;
            i < samplesCaptured;
            i++
        )
        {
            sum += adcBuffer[i];
        }


        dcOffset =
            sum /
            static_cast<float>(
                samplesCaptured
            );


        // ------------------------------------------------
        // Eliminar nivel DC
        // ------------------------------------------------

        for (
            size_t i = 0;
            i < samplesCaptured;
            i++
        )
        {
            adcBuffer[i] -=
                dcOffset;
        }


        captureSuccessful = true;


        return true;
    }


    // ==================================================
    // Mostrar información
    // ==================================================

    void showInformation()
    {
        Serial.println();
        Serial.println(
            "=== ADQUISICION ADC DMA ==="
        );


        if (!captureSuccessful)
        {
            Serial.println(
                "La captura no fue exitosa."
            );

            return;
        }


        const float expectedTimeMs =
            (
                static_cast<float>(
                    Config::RECEIVED_SAMPLES
                )
                /
                Config::SAMPLE_RATE
            )
            *
            1000.0f;


        const float actualTimeMs =
            static_cast<float>(
                captureTimeUs
            )
            /
            1000.0f;


        const float measuredRate =
            (
                static_cast<float>(
                    samplesCaptured
                )
                *
                1000000.0f
            )
            /
            static_cast<float>(
                captureTimeUs
            );


        Serial.printf(
            "Muestras capturadas: %u\n",
            static_cast<unsigned>(
                samplesCaptured
            )
        );


        Serial.printf(
            "Frecuencia configurada: %lu Hz\n",
            Config::SAMPLE_RATE
        );


        Serial.printf(
            "Tiempo esperado: %.3f ms\n",
            expectedTimeMs
        );


        Serial.printf(
            "Tiempo medido: %.3f ms\n",
            actualTimeMs
        );


        Serial.printf(
            "Frecuencia aproximada: %.1f Hz\n",
            measuredRate
        );


        Serial.printf(
            "Nivel DC promedio: %.1f\n",
            dcOffset
        );


        Serial.printf(
            "Valor RAW minimo: %u\n",
            rawMinimum
        );


        Serial.printf(
            "Valor RAW maximo: %u\n",
            rawMaximum
        );
    }


    // ==================================================
    // Acceso al buffer
    // ==================================================

    const float* data()
    {
        return adcBuffer;
    }
}