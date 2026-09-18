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
// Parámetros de ruido
// ======================================================

// Desviación estándar relativa al nivel de la señal
constexpr float NOISE_STD = 0.15f;

// Misma amplitud usada para generar el chirp
constexpr float SIGNAL_AMPLITUDE = 30000.0f;

// Semilla fija para obtener siempre la misma prueba
constexpr uint32_t RANDOM_SEED = 12345;

// Estado del generador pseudoaleatorio
uint32_t randomState = RANDOM_SEED;

// Variables usadas por Box-Muller
bool hasSpareGaussian = false;
float spareGaussian = 0.0f;


// ======================================================
// Parámetros de detección
// ======================================================

constexpr float MIN_DETECTION_DISTANCE = 0.20f;

constexpr float PEAK_THRESHOLD_RATIO = 0.30f;

constexpr size_t MIN_PEAK_SEPARATION = 50;

constexpr size_t MAX_DETECTIONS = 10;


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


// ======================================================
// Tamaños de buffers
// ======================================================

constexpr size_t MAX_DELAY = 700;

constexpr size_t RECEIVED_SAMPLES =
    NUM_SAMPLES + MAX_DELAY;

// Retardos posibles:
//
// received = 1660
// chirp    = 960
//
// 1660 - 960 = 700
//
// Se prueban retardos 0...700
constexpr size_t MAX_LAG =
    RECEIVED_SAMPLES - NUM_SAMPLES;

constexpr size_t CORRELATION_SIZE =
    MAX_LAG + 1;


// ======================================================
// Pines I2S
// ======================================================

constexpr int I2S_BCLK = 4;
constexpr int I2S_LRCLK = 5;
constexpr int I2S_DATA = 6;

constexpr i2s_port_t I2S_PORT =
    I2S_NUM_0;


// ======================================================
// Buffers
// ======================================================

// Chirp original mono
int16_t chirp[NUM_SAMPLES];

// Buffer estéreo para I2S
int16_t stereoBuffer[NUM_SAMPLES * 2];

// Señal recibida simulada
float receivedSignal[RECEIVED_SAMPLES];

// Resultado de correlación
float correlation[CORRELATION_SIZE];


// ======================================================
// Estructura para ecos detectados
// ======================================================

struct Detection
{
    size_t lag;
    float correlationValue;
    float distance;
};

Detection detections[MAX_DETECTIONS];

size_t detectionCount = 0;


// ======================================================
// Generar chirp
// ======================================================

void generateChirp()
{
    const float k =
        (F_END - F_START) / DURATION;

    for (size_t n = 0; n < NUM_SAMPLES; n++)
    {
        const float t =
            static_cast<float>(n) /
            SAMPLE_RATE;

        const float phase =
            2.0f * PI *
            (
                F_START * t
                + 0.5f * k * t * t
            );

        chirp[n] =
            static_cast<int16_t>(
                sinf(phase) * SIGNAL_AMPLITUDE
            );
    }
}


// ======================================================
// Mostrar información del chirp
// ======================================================

void showChirpInformation()
{
    Serial.println();
    Serial.println(
        "=== CHIRP GENERADO ==="
    );

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
        static_cast<unsigned>(
            NUM_SAMPLES
        )
    );

    Serial.printf(
        "Memoria usada por chirp: %u bytes\n",
        static_cast<unsigned>(
            sizeof(chirp)
        )
    );
}


// ======================================================
// Crear buffer estéreo
// ======================================================

void createStereoBuffer()
{
    for (size_t i = 0; i < NUM_SAMPLES; i++)
    {
        stereoBuffer[i * 2] =
            chirp[i];

        stereoBuffer[i * 2 + 1] =
            chirp[i];
    }
}


// ======================================================
// Conversión distancia -> muestras
// ======================================================

size_t distanceToSamples(float distance)
{
    const float travelTime =
        (2.0f * distance) /
        SOUND_SPEED;

    return static_cast<size_t>(
        roundf(
            travelTime *
            SAMPLE_RATE
        )
    );
}


// ======================================================
// Conversión muestras -> distancia
// ======================================================

float samplesToDistance(size_t samples)
{
    const float travelTime =
        static_cast<float>(samples) /
        SAMPLE_RATE;

    return
        (
            travelTime *
            SOUND_SPEED
        )
        / 2.0f;
}

// ======================================================
// Generador pseudoaleatorio reproducible
// ======================================================

uint32_t nextRandom()
{
    // Generador congruencial lineal
    randomState =
        1664525UL * randomState
        + 1013904223UL;

    return randomState;
}


// ======================================================
// Número uniforme entre 0 y 1
// ======================================================

float randomUniform()
{
    return
        (
            static_cast<float>(
                nextRandom()
            )
            + 1.0f
        )
        /
        4294967297.0f;
}


// ======================================================
// Ruido gaussiano mediante Box-Muller
// ======================================================

float gaussianRandom()
{
    // Box-Muller produce dos valores gaussianos
    // por cada par de números uniformes.
    // Guardamos uno para usarlo después.

    if (hasSpareGaussian)
    {
        hasSpareGaussian = false;

        return spareGaussian;
    }


    const float u1 =
        randomUniform();

    const float u2 =
        randomUniform();


    const float magnitude =
        sqrtf(
            -2.0f *
            logf(u1)
        );

    const float angle =
        2.0f *
        PI *
        u2;


    const float value1 =
        magnitude *
        cosf(angle);

    const float value2 =
        magnitude *
        sinf(angle);


    spareGaussian =
        value2;

    hasSpareGaussian =
        true;


    return value1;
}


// ======================================================
// Reiniciar generador de ruido
// ======================================================

void resetNoiseGenerator()
{
    randomState =
        RANDOM_SEED;

    hasSpareGaussian =
        false;

    spareGaussian =
        0.0f;
}

// ======================================================
// Simular ecos
// ======================================================

// ======================================================
// Simular señal recibida con ecos y ruido
// ======================================================

void simulateEchoes()
{
    // Reiniciar la semilla para que la prueba
    // sea reproducible.
    resetNoiseGenerator();


    // --------------------------------------------------
    // 1. Generar ruido gaussiano
    // --------------------------------------------------

    const float noiseAmplitude =
        NOISE_STD *
        SIGNAL_AMPLITUDE;


    for (size_t i = 0;
         i < RECEIVED_SAMPLES;
         i++)
    {
        receivedSignal[i] =
            gaussianRandom()
            *
            noiseAmplitude;
    }


    // --------------------------------------------------
    // 2. Agregar los ecos encima del ruido
    // --------------------------------------------------

    for (size_t echoIndex = 0;
         echoIndex < NUM_ECHOES;
         echoIndex++)
    {
        const size_t delay =
            distanceToSamples(
                ECHOES[echoIndex].distance
            );


        for (size_t n = 0;
             n < NUM_SAMPLES;
             n++)
        {
            const size_t destination =
                n + delay;


            if (destination <
                RECEIVED_SAMPLES)
            {
                receivedSignal[destination] +=
                    static_cast<float>(
                        chirp[n]
                    )
                    *
                    ECHOES[
                        echoIndex
                    ].amplitude;
            }
        }
    }
}


// ======================================================
// Mostrar ecos que fueron simulados
// ======================================================

void showEchoInformation()
{
    Serial.println();
    Serial.println(
        "=== ECOS SIMULADOS ==="
    );
    Serial.printf(
    "Ruido gaussiano: sigma = %.2f\n",
    NOISE_STD
);

    Serial.printf(
        "Desviacion PCM: %.0f\n",
        NOISE_STD *
        SIGNAL_AMPLITUDE
    );

    Serial.printf(
        "Semilla: %lu\n",
        RANDOM_SEED
    );

    Serial.println();

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
            distanceToSamples(
                distance
            );

        Serial.printf(
            "Eco %u: %.2f m | "
            "%.3f ms | "
            "%u muestras | "
            "amplitud %.2f\n",

            static_cast<unsigned>(
                i + 1
            ),

            distance,

            travelTime *
            1000.0f,

            static_cast<unsigned>(
                delay
            ),

            ECHOES[i].amplitude
        );
    }

    Serial.printf(
        "Buffer recibido: %u muestras\n",
        static_cast<unsigned>(
            RECEIVED_SAMPLES
        )
    );

    Serial.printf(
        "Memoria de señal recibida: "
        "%u bytes\n",
        static_cast<unsigned>(
            sizeof(receivedSignal)
        )
    );
}


// ======================================================
// Correlación directa
// ======================================================

void calculateDirectCorrelation()
{
    Serial.println();
    Serial.println(
        "=== CORRELACION DIRECTA ==="
    );

    const unsigned long startTime =
        micros();


    // Probar cada posible retardo
    for (size_t lag = 0;
         lag <= MAX_LAG;
         lag++)
    {
        float sum = 0.0f;

        // Producto punto entre:
        //
        // chirp[n]
        //
        // y
        //
        // receivedSignal[n + lag]
        //
        for (size_t n = 0;
             n < NUM_SAMPLES;
             n++)
        {
            sum +=
                static_cast<float>(
                    chirp[n]
                )
                *
                receivedSignal[
                    n + lag
                ];
        }

        correlation[lag] = sum;
    }


    const unsigned long endTime =
        micros();

    const unsigned long elapsed =
        endTime - startTime;


    Serial.println(
        "Correlacion calculada."
    );

    Serial.printf(
        "Retardos evaluados: %u\n",
        static_cast<unsigned>(
            CORRELATION_SIZE
        )
    );

    Serial.printf(
        "Tiempo de calculo: %lu us\n",
        elapsed
    );

    Serial.printf(
        "Tiempo de calculo: %.3f ms\n",
        elapsed / 1000.0f
    );
}


// ======================================================
// Encontrar máximo de correlación
// ======================================================

float findMaximumCorrelation()
{
    float maximum = 0.0f;

    const size_t minimumLag =
        distanceToSamples(
            MIN_DETECTION_DISTANCE
        );

    for (size_t lag = minimumLag;
         lag <= MAX_LAG;
         lag++)
    {
        if (correlation[lag] >
            maximum)
        {
            maximum =
                correlation[lag];
        }
    }

    return maximum;
}


// ======================================================
// Detectar picos de correlación
// ======================================================

void detectEchoes()
{
    detectionCount = 0;

    const size_t minimumLag =
        distanceToSamples(
            MIN_DETECTION_DISTANCE
        );

    const float maximumCorrelation =
        findMaximumCorrelation();

    const float threshold =
        maximumCorrelation *
        PEAK_THRESHOLD_RATIO;


    Serial.println();
    Serial.println(
        "=== DETECCION DE PICOS ==="
    );

    Serial.printf(
        "Retardo minimo: %u muestras\n",
        static_cast<unsigned>(
            minimumLag
        )
    );

    Serial.printf(
        "Umbral relativo: %.2f\n",
        PEAK_THRESHOLD_RATIO
    );


    // Buscar máximos locales
    for (size_t lag = minimumLag + 1;
         lag < MAX_LAG;
         lag++)
    {
        const float current =
            correlation[lag];

        const bool aboveThreshold =
            current >= threshold;

        const bool localMaximum =
            current >
                correlation[lag - 1]
            &&
            current >=
                correlation[lag + 1];


        if (!aboveThreshold ||
            !localMaximum)
        {
            continue;
        }


        // Si este es el primer pico,
        // lo guardamos directamente.
        if (detectionCount == 0)
        {
            detections[0].lag =
                lag;

            detections[0].correlationValue =
                current;

            detections[0].distance =
                samplesToDistance(lag);

            detectionCount = 1;

            continue;
        }


        Detection &lastDetection =
            detections[
                detectionCount - 1
            ];


        const size_t separation =
            lag -
            lastDetection.lag;


        // Si está muy cerca de otro pico,
        // conservar solo el más fuerte.
        if (separation <
            MIN_PEAK_SEPARATION)
        {
            if (current >
                lastDetection.correlationValue)
            {
                lastDetection.lag =
                    lag;

                lastDetection.correlationValue =
                    current;

                lastDetection.distance =
                    samplesToDistance(
                        lag
                    );
            }

            continue;
        }


        // Guardar nuevo pico
        if (detectionCount <
            MAX_DETECTIONS)
        {
            detections[
                detectionCount
            ].lag =
                lag;

            detections[
                detectionCount
            ].correlationValue =
                current;

            detections[
                detectionCount
            ].distance =
                samplesToDistance(
                    lag
                );

            detectionCount++;
        }
    }
}


// ======================================================
// Mostrar ecos detectados
// ======================================================

void showDetectedEchoes()
{
    Serial.println();
    Serial.println(
        "=== ECOS DETECTADOS ==="
    );

    if (detectionCount == 0)
    {
        Serial.println(
            "No se detectaron ecos."
        );

        return;
    }


    for (size_t i = 0;
         i < detectionCount;
         i++)
    {
        Serial.printf(
            "Eco %u\n",
            static_cast<unsigned>(
                i + 1
            )
        );

        Serial.printf(
            "Retardo: %u muestras\n",
            static_cast<unsigned>(
                detections[i].lag
            )
        );

        Serial.printf(
            "Distancia: %.3f m\n",
            detections[i].distance
        );

        Serial.println();
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

        return;
    }


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
// Transmitir chirp
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


    // --------------------------------------------------
    // 1. Generar chirp
    // --------------------------------------------------

    generateChirp();

    showChirpInformation();


    // --------------------------------------------------
    // 2. Crear buffer para transmisión I2S
    // --------------------------------------------------

    createStereoBuffer();

    Serial.printf(
        "Buffer I2S: %u bytes\n",
        static_cast<unsigned>(
            sizeof(stereoBuffer)
        )
    );


    // --------------------------------------------------
    // 3. Simular recepción de ecos
    // --------------------------------------------------

    simulateEchoes();

    showEchoInformation();


    // --------------------------------------------------
    // 4. Correlacionar señal recibida con el chirp
    // --------------------------------------------------

    calculateDirectCorrelation();


    // --------------------------------------------------
    // 5. Detectar los picos
    // --------------------------------------------------

    detectEchoes();

    showDetectedEchoes();


    // --------------------------------------------------
    // 6. Preparar salida física I2S
    // --------------------------------------------------

    configureI2S();


    // --------------------------------------------------
    // 7. Transmitir chirp
    // --------------------------------------------------

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