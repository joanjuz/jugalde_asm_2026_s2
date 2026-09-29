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
        ADC_ATTEN_DB_12;


    // ESP32-S3 entrega 4 bytes por conversión
    // en el formato TYPE2.
    constexpr size_t ADC_BYTES_PER_SAMPLE =
        sizeof(adc_digi_output_data_t);


    // Tamaño de cada bloque generado por DMA.
    //
    // Debe ser múltiplo del tamaño de una
    // conversión.
    constexpr size_t DMA_FRAME_BYTES =
        256;


    // Buffer interno del driver.
    //
    // 8192 bytes permiten almacenar más de
    // las 1660 muestras necesarias.
    constexpr size_t DMA_STORE_BYTES =
        8192;


    // Buffer temporal usado para leer desde DMA.
    constexpr size_t DMA_READ_BYTES =
        512;


    // En adc_digi_pattern_config_t:
    //
    // 0 = ADC1
    // 1 = ADC2
    //
    // En ESP32-S3 usamos ADC1 para DMA.
    constexpr uint8_t ADC_DMA_UNIT =
        0;


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


    static bool initialized =
        false;


    static bool captureSuccessful =
        false;


    static bool captureRunning =
        false;


    static uint64_t captureTimeUs =
        0;


    static uint64_t captureStartTimeUs =
        0;


    static size_t samplesCaptured =
        0;


    static float dcOffset =
        0.0f;


    static uint16_t rawMinimum =
        0;


    static uint16_t rawMaximum =
        0;
        // Límites usados solamente para diagnosticar
    // saturación del ADC de 12 bits.
    constexpr uint16_t RAW_LOW_LIMIT =
        50;

    constexpr uint16_t RAW_HIGH_LIMIT =
        4000;


    static size_t lowSaturationCount =
        0;

    static size_t highSaturationCount =
        0;


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


        pattern.unit =
            ADC_DMA_UNIT;


        pattern.bit_width =
            SOC_ADC_DIGI_MAX_BITWIDTH;


        // ------------------------------------------------
        // Configuración del controlador digital
        // ------------------------------------------------

        adc_digi_configuration_t config = {};


        // No limitar la conversión a 255 muestras.
        config.conv_limit_en =
            false;


        config.conv_limit_num =
            255;


        config.pattern_num =
            1;


        config.adc_pattern =
            &pattern;


        config.sample_freq_hz =
            Config::SAMPLE_RATE;


        config.conv_mode =
            ADC_CONV_SINGLE_UNIT_1;


        // ESP32-S3 utiliza TYPE2 en modo DMA.
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


        initialized =
            true;


        Serial.println();

        Serial.println(
            "ADC continuo con DMA configurado."
        );


        Serial.printf(
            "GPIO ADC: %d\n",
            Config::ADC_MIC_PIN
        );


        Serial.println(
            "Canal ADC: ADC1_CHANNEL_0"
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
    // Iniciar captura
    // ==================================================

    bool startCapture()
    {
        captureSuccessful =
            false;


        samplesCaptured =
            0;


        captureTimeUs =
            0;


        dcOffset =
            0.0f;
        


        if (!initialized)
        {
            Serial.println(
                "ADC DMA no inicializado."
            );

            return false;
        }


        if (captureRunning)
        {
            Serial.println(
                "Ya existe una captura ADC en curso."
            );

            return false;
        }


        rawMinimum =
            0xFFFF;


        rawMaximum =
            0;
        lowSaturationCount =
            0;

        highSaturationCount =
            0;


        // ------------------------------------------------
        // Iniciar ADC continuo
        // ------------------------------------------------

        const esp_err_t result =
            adc_digi_start();


        if (result != ESP_OK)
        {
            Serial.printf(
                "Error iniciando ADC DMA: %s\n",
                esp_err_to_name(result)
            );

            return false;
        }


        captureStartTimeUs =
            esp_timer_get_time();


        captureRunning =
            true;


        return true;
    }


    // ==================================================
    // Finalizar captura
    // ==================================================

    bool finishCapture()
    {
        if (!captureRunning)
        {
            Serial.println(
                "No existe una captura ADC en curso."
            );

            return false;
        }


        // ==================================================
        // Leer muestras acumuladas por DMA
        // ==================================================

        while (
            samplesCaptured
            <
            Config::RECEIVED_SAMPLES
        )
        {
            uint32_t bytesRead =
                0;


            const esp_err_t result =
                adc_digi_read_bytes(
                    dmaReadBuffer,
                    sizeof(
                        dmaReadBuffer
                    ),
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


                captureRunning =
                    false;


                return false;
            }


            // --------------------------------------------
            // Extraer conversiones ADC
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
                        dmaReadBuffer
                        +
                        offset
                    );


                const uint16_t rawValue =
                    static_cast<uint16_t>(
                        sample->type2.data
                    );
                if (rawValue <= RAW_LOW_LIMIT)
                {
                    lowSaturationCount++;
                }


                if (rawValue >= RAW_HIGH_LIMIT)
                {
                    highSaturationCount++;
                }


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


        // ==================================================
        // Detener ADC
        // ==================================================

        const esp_err_t stopResult =
            adc_digi_stop();


        captureTimeUs =
            esp_timer_get_time()
            -
            captureStartTimeUs;


        captureRunning =
            false;


        if (stopResult != ESP_OK)
        {
            Serial.printf(
                "Error deteniendo ADC DMA: %s\n",
                esp_err_to_name(stopResult)
            );

            return false;
        }


        if (samplesCaptured == 0)
        {
            Serial.println(
                "No se capturaron muestras."
            );

            return false;
        }


        // ==================================================
        // Calcular componente DC
        // ==================================================

        float sum =
            0.0f;


        for (
            size_t i = 0;
            i < samplesCaptured;
            i++
        )
        {
            sum +=
                adcBuffer[i];
        }


        dcOffset =
            sum
            /
            static_cast<float>(
                samplesCaptured
            );


        // ==================================================
        // Eliminar componente DC
        // ==================================================

        for (
            size_t i = 0;
            i < samplesCaptured;
            i++
        )
        {
            adcBuffer[i] -=
                dcOffset;
        }


        captureSuccessful =
            true;


        return true;
    }


    // ==================================================
    // Captura completa
    //
    // Se conserva para pruebas donde no sea necesario
    // sincronizar la emisión del chirp.
    // ==================================================

    bool capture()
    {
        if (!startCapture())
        {
            return false;
        }


        return finishCapture();
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
        const size_t saturatedSamples =
            lowSaturationCount
            +
            highSaturationCount;


        const float saturationPercentage =
            (
                100.0f
                *
                static_cast<float>(
                    saturatedSamples
                )
            )
            /
            static_cast<float>(
                samplesCaptured
            );


        Serial.printf(
            "Muestras RAW <= %u: %u\n",
            RAW_LOW_LIMIT,
            static_cast<unsigned>(
                lowSaturationCount
            )
        );


        Serial.printf(
            "Muestras RAW >= %u: %u\n",
            RAW_HIGH_LIMIT,
            static_cast<unsigned>(
                highSaturationCount
            )
        );


        Serial.printf(
            "Muestras cerca de saturacion: %u / %u (%.2f %%)\n",
            static_cast<unsigned>(
                saturatedSamples
            ),
            static_cast<unsigned>(
                samplesCaptured
            ),
            saturationPercentage
        );
    }


    // ==================================================
    // Acceso al buffer
    // ==================================================

    const float* data()
    {
        return adcBuffer;
    }

} // namespace ADCInput