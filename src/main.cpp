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
// Parámetros físicos
// ======================================================

constexpr float SOUND_SPEED = 343.0f;


// ======================================================
// Ecos simulados
// ======================================================

struct Echo
{
    float distance;
    float amplitude;
};

constexpr Echo ECHOES[] =
{
    {1.0f, 0.7f},
    {1.5f, 0.5f},
    {2.2f, 0.3f}
};

constexpr size_t NUM_ECHOES =
    sizeof(ECHOES) / sizeof(ECHOES[0]);


// Máximo retardo que vamos a permitir en esta simulación
constexpr size_t MAX_DELAY = 700;

// Tamaño total de la señal recibida
constexpr size_t RECEIVED_SAMPLES =
    NUM_SAMPLES + MAX_DELAY;


// ======================================================
// Pines I2S
// ======================================================

constexpr int I2S_BCLK = 4;
constexpr int I2S_LRCLK = 5;
constexpr int I2S_DATA = 6;

constexpr i2s_port_t I2S_PORT = I2S_NUM_0;


// ======================================================
// Buffers
// ======================================================

// Chirp mono de 16 bits
int16_t chirp[NUM_SAMPLES];

// Buffer estéreo para transmitir por I2S
int16_t stereoBuffer[NUM_SAMPLES * 2];

// Señal recibida simulada
float receivedSignal[RECEIVED_SAMPLES];


// ======================================================
// Generar chirp
// ======================================================

void generateChirp()
{
    // Pendiente de frecuencia del chirp
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

        // Convertir la señal de -1...1 a PCM de 16 bits
        chirp[n] =
            static_cast<int16_t>(
                sinf(phase) * 30000.0f
            );
    }
}


// ======================================================
// Mostrar información del chirp
// ======================================================

void showChirpInformation()
{
    Serial.println();
    Serial.println("=== CHIRP GENERADO ===");

    Serial.printf(
        "Frecuencia de muestreo: %lu Hz\n",
        SAMPLE_RATE
    );

    Serial.printf(
        "Duracion: %.1f ms\n",
        DURATION * 1000.0f
    );

    Serial.printf(
        "Frecuencia inicial: %.0f Hz\n",
        F_START
    );

    Serial.printf(
        "Frecuencia final: %.0f Hz\n",
        F_END
    );

    Serial.printf(
        "Numero de muestras: %u\n",
        static_cast<unsigned>(NUM_SAMPLES)
    );

    Serial.printf(
        "Memoria usada por chirp: %u bytes\n",
        static_cast<unsigned>(sizeof(chirp))
    );
}


// ======================================================
// Convertir chirp mono a estéreo
// ======================================================

void createStereoBuffer()
{
    for (size_t i = 0; i < NUM_SAMPLES; i++)
    {
        // Misma muestra para ambos canales
        stereoBuffer[i * 2] =
            chirp[i];

        stereoBuffer[i * 2 + 1] =
            chirp[i];
    }
}


// ======================================================
// Convertir distancia a retardo en muestras
// ======================================================

size_t distanceToSamples(float distance)
{
    // El sonido va hasta el objeto y regresa
    const float travelTime =
        (2.0f * distance) / SOUND_SPEED;

    return static_cast<size_t>(
        roundf(travelTime * SAMPLE_RATE)
    );
}


// ======================================================
// Simular ecos
// ======================================================

void simulateEchoes()
{
    // Limpiar el buffer recibido
    for (size_t i = 0;
         i < RECEIVED_SAMPLES;
         i++)
    {
        receivedSignal[i] = 0.0f;
    }

    // Agregar cada eco
    for (size_t echoIndex = 0;
         echoIndex < NUM_ECHOES;
         echoIndex++)
    {
        const size_t delay =
            distanceToSamples(
                ECHOES[echoIndex].distance
            );

        // Copiar el chirp desplazado
        for (size_t n = 0;
             n < NUM_SAMPLES;
             n++)
        {
            const size_t destination =
                n + delay;

            if (destination < RECEIVED_SAMPLES)
            {
                receivedSignal[destination] +=
                    static_cast<float>(chirp[n]) *
                    ECHOES[echoIndex].amplitude;
            }
        }
    }
}


// ======================================================
// Mostrar información de los ecos simulados
// ======================================================

void showEchoInformation()
{
    Serial.println();
    Serial.println("=== ECOS SIMULADOS ===");

    for (size_t i = 0;
         i < NUM_ECHOES;
         i++)
    {
        const float distance =
            ECHOES[i].distance;

        const float travelTime =
            (2.0f * distance) /
            SOUND_SPEED;

        const size_t delay =
            distanceToSamples(distance);

        Serial.printf(
            "Eco %u: %.2f m | %.3f ms | %u muestras | amplitud %.2f\n",
            static_cast<unsigned>(i + 1),
            distance,
            travelTime * 1000.0f,
            static_cast<unsigned>(delay),
            ECHOES[i].amplitude
        );
    }

    Serial.printf(
        "Buffer recibido: %u muestras\n",
        static_cast<unsigned>(RECEIVED_SAMPLES)
    );

    Serial.printf(
        "Memoria de señal recibida: %u bytes\n",
        static_cast<unsigned>(
            sizeof(receivedSignal)
        )
    );
}


// ======================================================
// Configuración de I2S
// ======================================================

void configureI2S()
{
    i2s_config_t i2sConfig = {};

    i2sConfig.mode =
        static_cast<i2s_mode_t>(
            I2S_MODE_MASTER |
            I2S_MODE_TX
        );

    i2sConfig.sample_rate =
        SAMPLE_RATE;

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


    // Instalar driver I2S
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


    // Configurar pines I2S
    i2s_pin_config_t pinConfig = {};

    pinConfig.bck_io_num =
        I2S_BCLK;

    pinConfig.ws_io_num =
        I2S_LRCLK;

    pinConfig.data_out_num =
        I2S_DATA;

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

        return;
    }

    Serial.println();
    Serial.println(
        "I2S configurado correctamente."
    );
}


// ======================================================
// Transmitir chirp por I2S
// ======================================================

void transmitChirp()
{
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
                NUM_SAMPLES
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


// ======================================================
// Setup
// ======================================================

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


    // 1. Generar chirp
    generateChirp();

    showChirpInformation();


    // 2. Crear buffer para I2S
    createStereoBuffer();

    Serial.printf(
        "Buffer I2S: %u bytes\n",
        static_cast<unsigned>(
            sizeof(stereoBuffer)
        )
    );


    // 3. Simular señal recibida con ecos
    simulateEchoes();

    showEchoInformation();


    // 4. Configurar salida I2S
    configureI2S();


    // 5. Transmitir chirp
    transmitChirp();
}


// ======================================================
// Loop
// ======================================================

void loop()
{
    delay(5000);

    Serial.println();
    Serial.println(
        "Transmitiendo chirp nuevamente..."
    );

    transmitChirp();
}