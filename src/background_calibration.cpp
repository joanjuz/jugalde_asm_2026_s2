#include "background_calibration.h"

#include <Arduino.h>

#include "config.h"
#include "fft_correlation.h"
#include "radar_math.h"


namespace BackgroundCalibration
{
    // ==================================================
    // Perfil de fondo
    // ==================================================

    static float backgroundSum[
        Config::CORRELATION_SIZE
    ] = {0.0f};

    static float backgroundProfile[
        Config::CORRELATION_SIZE
    ] = {0.0f};

    // ==================================================
    // Envolvente maxima del fondo
    //
    // Guarda para cada retardo el mayor valor
    // normalizado observado durante la calibracion
    // con el frente vacio.
    // ==================================================

    static float backgroundMax[
        Config::CORRELATION_SIZE
    ] = {0.0f};


    // ==================================================
    // Envolvente minima del fondo
    //
    // Guarda para cada retardo el menor valor
    // normalizado observado durante la calibracion.
    //
    // Se usa SOLO para diagnostico bidireccional.
    // El detector actual continua usando backgroundMax.
    // ==================================================

    static float backgroundMin[
        Config::CORRELATION_SIZE
    ] = {0.0f};

    // ==================================================
    // Envolvente energetica RMS del fondo
    //
    // Para cada retardo guarda el mayor RMS local
    // observado durante la calibracion vacia.
    //
    // Ventana:
    //     lag - 4 ... lag + 4
    //
    // Este arreglo se usa SOLO para diagnostico.
    // No modifica el detector actual.
    // ==================================================

    static float backgroundEnergyMax[
        Config::CORRELATION_SIZE
    ] = {0.0f};


    static size_t backgroundCount[
        Config::CORRELATION_SIZE
    ] = {0};

    static size_t calibrationFrameCount =
        0;

    static bool calibrationReady =
        false;


    // ==================================================
    // Seguimiento temporal
    // ==================================================

    static bool trackingActive =
        false;

    static size_t trackedLag =
        0;

    static float filteredTrackedLag =
        0.0f;

    static size_t trackingMisses =
        0;


    // El objeto se coloca físicamente cerca de 50 cm.
    //
    // Experimentalmente el eco útil aparece alrededor
    // de 0.53 m, por eso usamos esta distancia solamente
    // para la adquisición inicial.
    constexpr float TRACKING_START_DISTANCE =
        0.53f;

    // Tolerancia inicial alrededor del eco esperado.
    constexpr size_t TRACKING_START_TOLERANCE =
        15;

    // Máximo salto permitido entre capturas.
    constexpr size_t TRACKING_MAX_JUMP =
        25;

    // Cambio mínimo para adquirir inicialmente.
    constexpr float TRACKING_ACQUIRE_CHANGE =
        0.025f;

    // Una vez adquirido permitimos cambios menores.
    constexpr float TRACKING_KEEP_CHANGE =
        0.015f;

    // Número de capturas que podemos perder antes
    // de cancelar el seguimiento.
    constexpr size_t TRACKING_MAX_MISSES =
        3;

    // Suavizado de la distancia mostrada.
    constexpr float TRACKING_FILTER_ALPHA =
        0.35f;

    // Para la adquisición inicial, un candidato debe
    // conservar al menos el 60 % del mayor cambio de
    // la captura. Durante seguimiento no se aplica
    // este filtro relativo: manda la continuidad.
    constexpr float TRACKING_RELATIVE_SCORE_GATE =
        0.60f;

    // Ventana para comparar el perfil actual contra
    // el perfil de fondo. Permite pequeños corrimientos
    // de 1 a 3 muestras entre capturas.
    constexpr int BACKGROUND_COMPARE_RADIUS =
        3;

    // ==================================================
    // GEOMETRIA BIESTATICA FUNCIONAL
    // ==================================================
    //
    // TARGET_MIN_DISTANCE y TARGET_MAX_DISTANCE
    // representan distancia FISICA real.
    //
    // Los lags internos siguen usando la diferencia
    // temporal medida respecto al camino directo TX-RX.
    // ==================================================

    static float physicalDistanceToRawDistance(
        float physicalDistance
    )
    {
        const float halfBaseline =
            Config::TX_RX_BASELINE *
            0.5f;


        return
            sqrtf(
                physicalDistance *
                    physicalDistance
                +
                halfBaseline *
                    halfBaseline
            )
            -
            halfBaseline;
    }


    static float rawDistanceToPhysicalDistance(
        float rawDistance
    )
    {
        if (rawDistance <= 0.0f)
        {
            return 0.0f;
        }


        return
            sqrtf(
                rawDistance *
                    rawDistance
                +
                Config::TX_RX_BASELINE *
                    rawDistance
            );
    }


    static size_t physicalDistanceToLag(
        float physicalDistance
    )
    {
        return
            RadarMath::distanceToSamples(
                physicalDistanceToRawDistance(
                    physicalDistance
                )
            );
    }


    static float lagToPhysicalDistance(
        float lagSamples
    )
    {
        const float rawDistance =
            (
                lagSamples
                /
                static_cast<float>(
                    Config::SAMPLE_RATE
                )
                *
                Config::SOUND_SPEED
            )
            /
            2.0f;


        return
            rawDistanceToPhysicalDistance(
                rawDistance
            );
    }


    // ==================================================
    // Estructura temporal para candidatos
    // ==================================================

    struct TrackingCandidate
    {
        size_t lag;
        float score;
        float strength;
    };

    constexpr size_t MAX_TRACKING_CANDIDATES =
        16;


    // ==================================================
    // Encontrar referencia temporal
    // ==================================================

    static bool findReference(
        size_t& referenceLag,
        float& normalizationValue
    )
    {
        const size_t count =
            FFTCorrelation::detectionCount();

        if (count == 0)
        {
            return false;
        }

        const FFTCorrelation::Detection* detections =
            FFTCorrelation::detections();

        // Usamos SIEMPRE el primer pico detectado como
        // referencia temporal. Su posición absoluta puede
        // variar porque ADC e I2S no arrancan en la misma
        // muestra, pero mantiene el mismo significado:
        // primer camino acústico detectado.
        referenceLag =
            detections[0].lag;

        // Para normalización sí usamos la mayor amplitud
        // de correlación de la captura.
        normalizationValue =
            detections[0].correlationValue;

        for (
            size_t i = 1;
            i < count;
            i++
        )
        {
            if (
                detections[i].correlationValue >
                normalizationValue
            )
            {
                normalizationValue =
                    detections[i].correlationValue;
            }
        }

        return normalizationValue >
            0.0f;
    }


    // ==================================================
    // Reiniciar calibración
    // ==================================================

    void reset()
    {
        calibrationFrameCount =
            0;

        calibrationReady =
            false;

        for (
            size_t i = 0;
            i < Config::CORRELATION_SIZE;
            i++
        )
        {
            backgroundSum[i] =
                0.0f;

            backgroundProfile[i] =
                0.0f;


            backgroundMax[i] =
                0.0f;


            backgroundMin[i] =
                0.0f;

            backgroundEnergyMax[i] =
                0.0f;


            backgroundCount[i] =
                0;
        }

        trackingActive =
            false;

        trackedLag =
            0;

        filteredTrackedLag =
            0.0f;

        trackingMisses =
            0;

        Serial.println();
        Serial.println(
            "Calibracion de fondo reiniciada."
        );
        Serial.println(
            "Mantenga el frente del radar sin objetos."
        );
    }


    // ==================================================
    // Estado de calibración
    // ==================================================

    bool isReady()
    {
        return calibrationReady;
    }


    // ==================================================
    // Agregar una captura al perfil de fondo
    // ==================================================

    void addCalibrationFrame()
    {
        if (calibrationReady)
        {
            return;
        }

        size_t referenceLag =
            0;

        float normalizationValue =
            0.0f;

        if (
            !findReference(
                referenceLag,
                normalizationValue
            )
        )
        {
            Serial.println(
                "Calibracion: no se encontro referencia."
            );

            return;
        }

        // ==================================================
        // Diagnostico de referencia durante calibracion
        //
        // Este bloque NO modifica ningun calculo.
        // Solo muestra que picos esta usando cada captura.
        // ==================================================

        const size_t calibrationDetectionCount =
            FFTCorrelation::detectionCount();

        const FFTCorrelation::Detection*
            calibrationDetections =
                FFTCorrelation::detections();

        Serial.println();
        Serial.println(
            "--- DIAGNOSTICO DE CALIBRACION ---"
        );

        Serial.printf(
            "Captura de calibracion: %u / %u\n",
            static_cast<unsigned>(
                calibrationFrameCount + 1
            ),
            static_cast<unsigned>(
                Config::BACKGROUND_CALIBRATION_FRAMES
            )
        );

        Serial.printf(
            "Referencia temporal: %u muestras\n",
            static_cast<unsigned>(
                referenceLag
            )
        );

        Serial.printf(
            "Picos detectados: %u\n",
            static_cast<unsigned>(
                calibrationDetectionCount
            )
        );

        for (
            size_t i = 0;
            i < calibrationDetectionCount;
            i++
        )
        {
            const size_t absoluteLag =
                calibrationDetections[i].lag;

            const size_t relativeLag =
                (
                    absoluteLag >=
                    referenceLag
                )
                ?
                absoluteLag -
                    referenceLag
                :
                0;

            float relativeStrength =
                0.0f;

            if (
                normalizationValue >
                0.0f
            )
            {
                relativeStrength =
                    calibrationDetections[
                        i
                    ].correlationValue
                    /
                    normalizationValue
                    *
                    100.0f;
            }

            Serial.printf(
                "Pico calib %u: abs=%u, rel=%u, fuerza=%.1f %%\n",
                static_cast<unsigned>(
                    i + 1
                ),
                static_cast<unsigned>(
                    absoluteLag
                ),
                static_cast<unsigned>(
                    relativeLag
                ),
                relativeStrength
            );
        }

        Serial.println();
        const float* correlation =
            FFTCorrelation::correlationData();

        for (
            size_t relativeLag = 0;
            relativeLag < Config::CORRELATION_SIZE;
            relativeLag++
        )
        {
            const size_t absoluteLag =
                referenceLag +
                relativeLag;

            if (
                absoluteLag >
                Config::MAX_LAG
            )
            {
                break;
            }

            const float normalizedValue =
                correlation[absoluteLag]
                /
                normalizationValue;

            // ------------------------------------------
            // Actualizar envolvente maxima del fondo.
            //
            // Conservamos tambien el promedio existente
            // para no eliminar informacion durante este
            // experimento.
            // ------------------------------------------

            if (
                normalizedValue >
                backgroundMax[
                    relativeLag
                ]
            )
            {
                backgroundMax[
                    relativeLag
                ] =
                    normalizedValue;
            }


            // ------------------------------------------
            // Actualizar envolvente minima del fondo.
            //
            // La primera muestra valida inicializa
            // directamente backgroundMin.
            //
            // Luego conservamos el menor valor observado.
            //
            // Esto es SOLO para diagnostico.
            // backgroundMax y el detector actual
            // permanecen sin cambios.
            // ------------------------------------------

            if (
                backgroundCount[
                    relativeLag
                ] == 0
                ||
                normalizedValue <
                    backgroundMin[
                        relativeLag
                    ]
            )
            {
                backgroundMin[
                    relativeLag
                ] =
                    normalizedValue;
            }

            backgroundSum[
                relativeLag
            ] += normalizedValue;

            backgroundCount[
                relativeLag
            ]++;
        }

        // ==================================================
        // ENVOLVENTE RMS DURANTE CALIBRACION
        //
        // Calculamos energia local sobre 9 muestras:
        //
        //     lag - 4 ... lag + 4
        //
        // Al elevar al cuadrado desaparece el signo de la
        // correlacion. Esto permite representar el paquete
        // oscilatorio completo en vez de una sola cresta.
        //
        // Solo almacenamos el mayor RMS observado en vacio.
        //
        // NO modifica backgroundMax ni backgroundMin.
        // ==================================================

        constexpr int
            RMS_ENVELOPE_RADIUS =
                4;


        for (
            size_t relativeLag = 0;
            relativeLag <
                Config::CORRELATION_SIZE;
            relativeLag++
        )
        {
            float energySumSquares =
                0.0f;


            size_t energySampleCount =
                0;


            for (
                int offset =
                    -RMS_ENVELOPE_RADIUS;
                offset <=
                    RMS_ENVELOPE_RADIUS;
                offset++
            )
            {
                const int neighborLag =
                    static_cast<int>(
                        relativeLag
                    )
                    +
                    offset;


                if (
                    neighborLag < 0
                    ||
                    neighborLag >=
                        static_cast<int>(
                            Config::
                                CORRELATION_SIZE
                        )
                )
                {
                    continue;
                }


                const size_t absoluteLag =
                    referenceLag
                    +
                    static_cast<size_t>(
                        neighborLag
                    );


                if (
                    absoluteLag >
                    Config::MAX_LAG
                )
                {
                    continue;
                }


                const float normalizedSample =
                    correlation[
                        absoluteLag
                    ]
                    /
                    normalizationValue;


                energySumSquares +=
                    normalizedSample
                    *
                    normalizedSample;


                energySampleCount++;
            }


            if (energySampleCount == 0)
            {
                continue;
            }


            const float rmsEnergy =
                sqrtf(
                    energySumSquares
                    /
                    static_cast<float>(
                        energySampleCount
                    )
                );


            if (
                rmsEnergy >
                backgroundEnergyMax[
                    relativeLag
                ]
            )
            {
                backgroundEnergyMax[
                    relativeLag
                ] =
                    rmsEnergy;
            }
        }

        calibrationFrameCount++;

        Serial.printf(
            "Calibracion de fondo: %u / %u\n",
            static_cast<unsigned>(
                calibrationFrameCount
            ),
            static_cast<unsigned>(
                Config::BACKGROUND_CALIBRATION_FRAMES
            )
        );

        if (
            calibrationFrameCount <
            Config::BACKGROUND_CALIBRATION_FRAMES
        )
        {
            return;
        }

        // Calcular promedio final del fondo.
        for (
            size_t i = 0;
            i < Config::CORRELATION_SIZE;
            i++
        )
        {
            if (
                backgroundCount[i] >
                0
            )
            {
                backgroundProfile[i] =
                    backgroundSum[i]
                    /
                    static_cast<float>(
                        backgroundCount[i]
                    );
            }
        }

        calibrationReady =
            true;

        trackingActive =
            false;

        trackedLag =
            0;

        filteredTrackedLag =
            0.0f;

        trackingMisses =
            0;

        Serial.println();
        Serial.println(
            "================================"
        );
        Serial.println(
            "=== FONDO CALIBRADO ==="
        );
        Serial.println(
            "================================"
        );
        Serial.println(
            "Puede colocar el objeto."
        );
    }


    // ==================================================
    // Compatibilidad con la interfaz usada por main.cpp
    // ==================================================

    size_t capturedFrames()
    {
        return calibrationFrameCount;
    }


    size_t requiredFrames()
    {
        return
            Config::BACKGROUND_CALIBRATION_FRAMES;
    }


    bool addCurrentFrame()
    {
        const size_t framesBefore =
            calibrationFrameCount;

        addCalibrationFrame();

        // Éxito si se agregó una nueva captura
        // o si la calibración ya quedó completa.
        return
            calibrationFrameCount > framesBefore
            ||
            calibrationReady;
    }


    bool finalize()
    {
        // Si ya quedó finalizada automáticamente
        // al capturar la última muestra, todo está bien.
        if (calibrationReady)
        {
            return true;
        }

        if (calibrationFrameCount == 0)
        {
            Serial.println(
                "No hay capturas para finalizar "
                "la calibracion."
            );

            return false;
        }

        // Calcular el perfil promedio disponible.
        for (
            size_t i = 0;
            i < Config::CORRELATION_SIZE;
            i++
        )
        {
            if (
                backgroundCount[i] >
                0
            )
            {
                backgroundProfile[i] =
                    backgroundSum[i]
                    /
                    static_cast<float>(
                        backgroundCount[i]
                    );
            }
        }

        calibrationReady =
            true;

        trackingActive =
            false;

        trackedLag =
            0;

        filteredTrackedLag =
            0.0f;

        trackingMisses =
            0;

        Serial.println();
        Serial.println(
            "================================"
        );
        Serial.println(
            "=== FONDO CALIBRADO ==="
        );
        Serial.println(
            "================================"
        );
        Serial.println(
            "Puede colocar el objeto."
        );

        return true;
    }


    // ==================================================
    // Analizar captura actual
    // ==================================================

    void analyzeCurrentFrame()
    {
        Serial.println();
        Serial.println(
            "=== SUSTRACCION DE FONDO ==="
        );

        if (!calibrationReady)
        {
            Serial.println(
                "El fondo aun no esta calibrado."
            );

            return;
        }

        const size_t detectionCount =
            FFTCorrelation::detectionCount();

        if (detectionCount == 0)
        {
            Serial.println();
            Serial.println(
                "--- CANDIDATOS DE OBJETO ---"
            );
            Serial.println();

            if (trackingActive)
            {
                trackingMisses++;

                if (
                    trackingMisses <=
                    TRACKING_MAX_MISSES
                )
                {
                    const size_t filteredLag =
                        static_cast<size_t>(
                            filteredTrackedLag +
                            0.5f
                        );

                    Serial.printf(
                        "Seguimiento temporal: captura perdida %u / %u\n",
                        static_cast<unsigned>(
                            trackingMisses
                        ),
                        static_cast<unsigned>(
                            TRACKING_MAX_MISSES
                        )
                    );
                    Serial.println(
                        "Objetivo: SEGUIDO"
                    );
                    Serial.printf(
                        "Distancia mantenida: %.3f m\n",
                        lagToPhysicalDistance(
                            filteredLag
                        )
                    );

                    return;
                }
            }

            trackingActive =
                false;
            trackedLag =
                0;
            filteredTrackedLag =
                0.0f;
            trackingMisses =
                0;

            Serial.println(
                "Objetivo: NO DETECTADO"
            );

            return;
        }

        size_t referenceLag =
            0;

        float normalizationValue =
            0.0f;

        if (
            !findReference(
                referenceLag,
                normalizationValue
            )
        )
        {
            Serial.println(
                "No se pudo determinar la referencia temporal."
            );

            return;
        }

        const float* correlation =
            FFTCorrelation::correlationData();

        const size_t minimumPhysicalLag =
            physicalDistanceToLag(
                Config::TARGET_MIN_DISTANCE
            );

        const size_t maximumPhysicalLag =
            physicalDistanceToLag(
                Config::TARGET_MAX_DISTANCE
            );


        // ==================================================
        // DIAGNOSTICO DE SIGNO DEL FONDO
        //
        // Solo imprime informacion.
        // NO modifica residual, candidatos, tracker,
        // umbrales ni seleccion del objetivo.
        //
        // Para cada pico FFT dentro del rango fisico:
        //
        // actual
        // promedio_fondo
        // max_fondo_local
        // delta_promedio = actual - promedio
        // delta_max      = actual - max_local
        //
        // Esto permite observar tambien cambios negativos.
        // ==================================================

        const FFTCorrelation::Detection*
            diagnosticDetections =
                FFTCorrelation::detections();


        constexpr int
            DIAGNOSTIC_BACKGROUND_RADIUS =
                2;


        Serial.println();
        Serial.println(
            "--- DIAGNOSTICO DE SIGNO ---"
        );


        for (
            size_t i = 0;
            i < detectionCount;
            i++
        )
        {
            if (
                diagnosticDetections[
                    i
                ].lag <
                referenceLag
            )
            {
                continue;
            }


            const size_t diagnosticLag =
                diagnosticDetections[
                    i
                ].lag
                -
                referenceLag;


            if (
                diagnosticLag <
                    minimumPhysicalLag
                ||
                diagnosticLag >
                    maximumPhysicalLag
            )
            {
                continue;
            }


            if (
                diagnosticLag >=
                Config::CORRELATION_SIZE
            )
            {
                continue;
            }


            const size_t absoluteDiagnosticLag =
                referenceLag +
                diagnosticLag;


            if (
                absoluteDiagnosticLag >
                Config::MAX_LAG
            )
            {
                continue;
            }


            const float currentNormalized =
                correlation[
                    absoluteDiagnosticLag
                ]
                /
                normalizationValue;


            const float averageBackground =
                backgroundProfile[
                    diagnosticLag
                ];


            float localBackgroundMax =
                0.0f;


            for (
                int offset =
                    -DIAGNOSTIC_BACKGROUND_RADIUS;
                offset <=
                    DIAGNOSTIC_BACKGROUND_RADIUS;
                offset++
            )
            {
                const int backgroundLag =
                    static_cast<int>(
                        diagnosticLag
                    )
                    +
                    offset;


                if (
                    backgroundLag < 0
                    ||
                    backgroundLag >=
                        static_cast<int>(
                            Config::
                                CORRELATION_SIZE
                        )
                )
                {
                    continue;
                }


                const float backgroundValue =
                    backgroundMax[
                        static_cast<size_t>(
                            backgroundLag
                        )
                    ];


                if (
                    backgroundValue >
                    localBackgroundMax
                )
                {
                    localBackgroundMax =
                        backgroundValue;
                }
            }


            const float deltaAverage =
                currentNormalized
                -
                averageBackground;


            const float deltaMaximum =
                currentNormalized
                -
                localBackgroundMax;


            Serial.printf(
                "DIAG lag=%u, dist=%.3f m, "
                "actual=%.1f %%, "
                "promedio=%.1f %%, "
                "max_local=%.1f %%, "
                "delta_promedio=%.1f %%, "
                "delta_max=%.1f %%\n",
                static_cast<unsigned>(
                    diagnosticLag
                ),
                RadarMath::samplesToDistance(
                    diagnosticLag
                ),
                currentNormalized *
                    100.0f,
                averageBackground *
                    100.0f,
                localBackgroundMax *
                    100.0f,
                deltaAverage *
                    100.0f,
                deltaMaximum *
                    100.0f
            );
        }


        Serial.println(
            "--- FIN DIAGNOSTICO DE SIGNO ---"
        );


        // ==================================================
        // DIAGNOSTICO RESIDUAL ENERGETICO VERDADERO
        //
        // Primero restamos el perfil promedio del fondo:
        //
        // residual =
        //     correlacion_actual_normalizada
        //     -
        //     backgroundProfile
        //
        // DESPUES calculamos energia:
        //
        // RMS residual local +/-4 muestras.
        //
        // De esta forma tanto un cambio positivo como un
        // cambio negativo contribuyen al mismo paquete.
        //
        // Tambien calculamos un centroide energetico del
        // residual^2 usando +/-12 muestras alrededor de cada
        // maximo RMS.
        //
        // ESTE BLOQUE ES SOLO DIAGNOSTICO.
        //
        // NO modifica:
        // - positiveResidual()
        // - backgroundProfile
        // - backgroundMax
        // - backgroundMin
        // - backgroundEnergyMax
        // - trackingCandidates
        // - NMS
        // - tracker
        // - seleccion final
        // ==================================================

        constexpr int
            TRUE_RESIDUAL_RMS_RADIUS =
                4;


        constexpr int
            TRUE_RESIDUAL_CENTROID_RADIUS =
                12;


        constexpr size_t
            TRUE_RESIDUAL_MAX_PEAKS =
                10;


        // ----------------------------------------------
        // Residual firmado exacto en un retardo fisico.
        // ----------------------------------------------

        auto signedResidualAt =
            [&](
                size_t physicalLag
            )
            -> float
        {
            if (
                physicalLag >=
                Config::CORRELATION_SIZE
            )
            {
                return 0.0f;
            }


            const size_t absoluteLag =
                referenceLag +
                physicalLag;


            if (
                absoluteLag >
                Config::MAX_LAG
            )
            {
                return 0.0f;
            }


            const float currentNormalized =
                correlation[
                    absoluteLag
                ]
                /
                normalizationValue;


            return
                currentNormalized
                -
                backgroundProfile[
                    physicalLag
                ];
        };


        // ----------------------------------------------
        // RMS local del RESIDUAL.
        //
        // Importante:
        //
        // primero restamos el fondo;
        // despues elevamos al cuadrado.
        // ----------------------------------------------

        auto trueResidualRmsAt =
            [&](
                size_t physicalLag
            )
            -> float
        {
            float sumSquares =
                0.0f;


            size_t sampleCount =
                0;


            for (
                int offset =
                    -TRUE_RESIDUAL_RMS_RADIUS;
                offset <=
                    TRUE_RESIDUAL_RMS_RADIUS;
                offset++
            )
            {
                const int neighborLag =
                    static_cast<int>(
                        physicalLag
                    )
                    +
                    offset;


                if (
                    neighborLag < 0
                    ||
                    neighborLag >=
                        static_cast<int>(
                            Config::
                                CORRELATION_SIZE
                        )
                )
                {
                    continue;
                }


                const float residual =
                    signedResidualAt(
                        static_cast<size_t>(
                            neighborLag
                        )
                    );


                sumSquares +=
                    residual *
                    residual;


                sampleCount++;
            }


            if (sampleCount == 0)
            {
                return 0.0f;
            }


            return sqrtf(
                sumSquares
                /
                static_cast<float>(
                    sampleCount
                )
            );
        };


        // ----------------------------------------------
        // Centroide energetico del paquete residual.
        //
        // Usa residual^2, no la correlacion cruda.
        // ----------------------------------------------

        auto trueResidualCentroidAt =
            [&](
                size_t centerLag
            )
            -> float
        {
            int firstLag =
                static_cast<int>(
                    centerLag
                )
                -
                TRUE_RESIDUAL_CENTROID_RADIUS;


            int lastLag =
                static_cast<int>(
                    centerLag
                )
                +
                TRUE_RESIDUAL_CENTROID_RADIUS;


            if (firstLag < 0)
            {
                firstLag = 0;
            }


            if (
                lastLag >=
                static_cast<int>(
                    Config::CORRELATION_SIZE
                )
            )
            {
                lastLag =
                    static_cast<int>(
                        Config::CORRELATION_SIZE
                    )
                    -
                    1;
            }


            float energySum =
                0.0f;


            float weightedLagSum =
                0.0f;


            for (
                int lag = firstLag;
                lag <= lastLag;
                lag++
            )
            {
                const float residual =
                    signedResidualAt(
                        static_cast<size_t>(
                            lag
                        )
                    );


                const float energy =
                    residual *
                    residual;


                energySum +=
                    energy;


                weightedLagSum +=
                    static_cast<float>(
                        lag
                    )
                    *
                    energy;
            }


            if (energySum <= 0.0f)
            {
                return static_cast<float>(
                    centerLag
                );
            }


            return
                weightedLagSum
                /
                energySum;
        };


        TrackingCandidate trueResidualPeaks[
            TRUE_RESIDUAL_MAX_PEAKS
        ];


        size_t trueResidualPeakCount =
            0;


        size_t residualLastLag =
            maximumPhysicalLag;


        if (
            residualLastLag >=
            Config::CORRELATION_SIZE
        )
        {
            residualLastLag =
                Config::CORRELATION_SIZE
                -
                1;
        }


        // ----------------------------------------------
        // Buscar maximos locales de RMS residual.
        // ----------------------------------------------

        if (
            residualLastLag >
            minimumPhysicalLag + 1
        )
        {
            for (
                size_t physicalLag =
                    minimumPhysicalLag + 1;
                physicalLag <
                    residualLastLag;
                physicalLag++
            )
            {
                const float previous =
                    trueResidualRmsAt(
                        physicalLag - 1
                    );


                const float current =
                    trueResidualRmsAt(
                        physicalLag
                    );


                const float next =
                    trueResidualRmsAt(
                        physicalLag + 1
                    );


                const bool localMaximum =
                    current >
                        previous
                    &&
                    current >=
                        next;


                if (
                    !localMaximum
                    ||
                    current <= 0.0f
                )
                {
                    continue;
                }


                size_t insertAt =
                    0;


                while (
                    insertAt <
                        trueResidualPeakCount
                    &&
                    trueResidualPeaks[
                        insertAt
                    ].score >=
                        current
                )
                {
                    insertAt++;
                }


                if (
                    insertAt >=
                        TRUE_RESIDUAL_MAX_PEAKS
                )
                {
                    continue;
                }


                if (
                    trueResidualPeakCount <
                    TRUE_RESIDUAL_MAX_PEAKS
                )
                {
                    trueResidualPeakCount++;
                }


                for (
                    size_t move =
                        trueResidualPeakCount - 1;
                    move > insertAt;
                    move--
                )
                {
                    trueResidualPeaks[
                        move
                    ] =
                        trueResidualPeaks[
                            move - 1
                        ];
                }


                trueResidualPeaks[
                    insertAt
                ].lag =
                    physicalLag;


                trueResidualPeaks[
                    insertAt
                ].score =
                    current;


                trueResidualPeaks[
                    insertAt
                ].strength =
                    current;
            }
        }


        Serial.println();

        Serial.println(
            "--- DIAGNOSTICO RESIDUAL ENERGETICO VERDADERO ---"
        );


        Serial.println(
            "Operacion: RMS(actual_normalizado - promedio_fondo)"
        );


        Serial.println(
            "Ventana RMS residual: +/-4 muestras"
        );


        Serial.println(
            "Ventana centroide residual: +/-12 muestras"
        );


        if (trueResidualPeakCount == 0)
        {
            Serial.println(
                "Sin maximos RMS residuales."
            );
        }
        else
        {
            for (
                size_t i = 0;
                i <
                    trueResidualPeakCount;
                i++
            )
            {
                const size_t lag =
                    trueResidualPeaks[
                        i
                    ].lag;


                const float centroid =
                    trueResidualCentroidAt(
                        lag
                    );


                const float centroidDistance =
                    (
                        centroid
                        /
                        static_cast<float>(
                            Config::SAMPLE_RATE
                        )
                        *
                        Config::SOUND_SPEED
                    )
                    /
                    2.0f;


                Serial.printf(
                    "RESIDUAL-RMS %u: "
                    "lag=%u, "
                    "dist=%.3f m, "
                    "rms=%.1f %%, "
                    "centroide=%.2f, "
                    "dist_centroide=%.3f m, "
                    "delta_centroide=%.2f muestras\n",
                    static_cast<unsigned>(
                        i + 1
                    ),
                    static_cast<unsigned>(
                        lag
                    ),
                    RadarMath::
                        samplesToDistance(
                            lag
                        ),
                    trueResidualPeaks[
                        i
                    ].score *
                        100.0f,
                    centroid,
                    centroidDistance,
                    centroid
                        -
                        static_cast<float>(
                            lag
                        )
                );
            }
        }


        Serial.println(
            "--- FIN RESIDUAL ENERGETICO VERDADERO ---"
        );


        Serial.println();


        // ==================================================
        // DIAGNOSTICO GEOMETRIA BIESTATICA
        //
        // El parlante y el microfono estan separados
        // fisicamente 9.5 cm.
        //
        // La distancia obtenida actualmente del retardo:
        //
        //     x = DeltaL / 2
        //
        // no corresponde directamente a la distancia real
        // al objeto porque se resta primero el camino
        // directo TX -> RX.
        //
        // Para un objeto aproximadamente frente al punto
        // medio entre parlante y microfono:
        //
        //     D = sqrt(
        //             x*x +
        //             baseline*x
        //         )
        //
        // SOLO DIAGNOSTICO.
        //
        // NO modifica:
        // - deteccion
        // - residual
        // - candidatos
        // - NMS
        // - tracker
        // - distancia final
        // ==================================================

        Serial.println(
            "--- DIAGNOSTICO GEOMETRIA BIESTATICA ---"
        );


        Serial.printf(
            "Separacion TX-RX: %.3f m\n",
            Config::TX_RX_BASELINE
        );


        if (trueResidualPeakCount == 0)
        {
            Serial.println(
                "Sin picos residuales para corregir."
            );
        }
        else
        {
            for (
                size_t i = 0;
                i < trueResidualPeakCount;
                i++
            )
            {
                const size_t bistaticLag =
                    trueResidualPeaks[
                        i
                    ].lag;


                const float currentDistance =
                    RadarMath::
                        samplesToDistance(
                            bistaticLag
                        );


                const float correctedDistance =
                    sqrtf(
                        currentDistance *
                            currentDistance
                        +
                        Config::
                            TX_RX_BASELINE *
                            currentDistance
                    );


                Serial.printf(
                    "BIESTATICA %u: "
                    "lag=%u, "
                    "actual=%.3f m, "
                    "corregida=%.3f m, "
                    "correccion=%.3f m\n",
                    static_cast<unsigned>(
                        i + 1
                    ),
                    static_cast<unsigned>(
                        bistaticLag
                    ),
                    currentDistance,
                    correctedDistance,
                    correctedDistance
                        -
                        currentDistance
                );
            }
        }


        Serial.println(
            "--- FIN GEOMETRIA BIESTATICA ---"
        );


        Serial.println();


        TrackingCandidate trackingCandidates[
            MAX_TRACKING_CANDIDATES
        ];

        size_t trackingCandidateCount =
            0;

        float maximumCandidateScore =
            0.0f;


        // ==================================================
        // Residual positivo completo
        //
        // El perfil actual se alinea con referenceLag y se
        // normaliza de la misma forma usada al calibrar.
        //
        // Para cada retardo fisico:
        //
        // residual =
        //     correlacion_actual_normalizada
        //     -
        //     perfil_de_fondo
        //
        // Solo conservamos cambios positivos.
        // ==================================================

        auto positiveResidual =
            [&](
                size_t physicalLag
            )
            -> float
        {
            if (
                physicalLag >=
                Config::CORRELATION_SIZE
            )
            {
                return 0.0f;
            }

            const size_t absoluteLag =
                referenceLag +
                physicalLag;

            if (
                absoluteLag >
                Config::MAX_LAG
            )
            {
                return 0.0f;
            }

            const float currentNormalized =
                correlation[
                    absoluteLag
                ]
                /
                normalizationValue;


            // ==============================================
            // Envolvente local del fondo
            //
            // En lugar de comparar solamente contra el
            // mismo retardo exacto, usamos el mayor valor
            // observado en el fondo dentro de +/-3 muestras.
            //
            // Esto absorbe pequenos corrimientos temporales
            // de los ecos fijos del entorno.
            // ==============================================

            constexpr int
                BACKGROUND_ENVELOPE_RADIUS =
                    2;


            float localBackgroundMax =
                0.0f;


            for (
                int offset =
                    -BACKGROUND_ENVELOPE_RADIUS;
                offset <=
                    BACKGROUND_ENVELOPE_RADIUS;
                offset++
            )
            {
                const int backgroundLag =
                    static_cast<int>(
                        physicalLag
                    )
                    +
                    offset;


                if (
                    backgroundLag < 0
                    ||
                    backgroundLag >=
                        static_cast<int>(
                            Config::
                                CORRELATION_SIZE
                        )
                )
                {
                    continue;
                }


                const float backgroundValue =
                    backgroundMax[
                        static_cast<size_t>(
                            backgroundLag
                        )
                    ];


                if (
                    backgroundValue >
                    localBackgroundMax
                )
                {
                    localBackgroundMax =
                        backgroundValue;
                }
            }


            const float change =
                currentNormalized
                -
                localBackgroundMax;


            if (change <= 0.0f)
            {
                return 0.0f;
            }


            return change;
        };


        size_t firstPhysicalLag =
            minimumPhysicalLag;

        if (firstPhysicalLag < 1)
        {
            firstPhysicalLag =
                1;
        }


        size_t lastPhysicalLag =
            maximumPhysicalLag;

        if (
            referenceLag +
            lastPhysicalLag >
            Config::MAX_LAG
        )
        {
            lastPhysicalLag =
                Config::MAX_LAG -
                referenceLag;
        }

        if (
            lastPhysicalLag >=
            Config::CORRELATION_SIZE - 1
        )
        {
            lastPhysicalLag =
                Config::CORRELATION_SIZE - 2;
        }


        // ==================================================
        // Buscar maximos locales DEL RESIDUAL
        //
        // Ya no dependemos de que el punto haya sido
        // detectado previamente como pico de la
        // correlacion cruda.
        // ==================================================

        if (
            firstPhysicalLag <=
            lastPhysicalLag
        )
        {
            // ==================================================
            // DIAGNOSTICO ENVOLVENTE RMS
            //
            // Convierte la correlacion oscilatoria local en
            // una medida energetica independiente del signo.
            //
            // RMS:
            //
            // sqrt(
            //     promedio(
            //         correlacion_normalizada^2
            //     )
            // )
            //
            // usando lag +/-4.
            //
            // Luego:
            //
            // exceso =
            //     energia_actual
            //     -
            //     maxima_energia_observada_en_vacio
            //
            // Solo se muestran los 10 maximos locales de
            // exceso energetico.
            //
            // ESTE BLOQUE ES SOLO DIAGNOSTICO.
            //
            // NO modifica:
            // - positiveResidual()
            // - backgroundMax
            // - backgroundMin
            // - trackingCandidates
            // - NMS
            // - tracker
            // - seleccion final
            // ==================================================

            constexpr size_t
                RMS_DIAGNOSTIC_MAX =
                    10;


            constexpr int
                RMS_DIAGNOSTIC_RADIUS =
                    4;


            TrackingCandidate rmsPeaks[
                RMS_DIAGNOSTIC_MAX
            ];


            size_t rmsPeakCount =
                0;


            // ------------------------------------------
            // Energia RMS actual en un retardo.
            // ------------------------------------------

            auto rmsEnergyAt =
                [&](
                    size_t physicalLag
                )
                -> float
            {
                if (
                    physicalLag >=
                    Config::CORRELATION_SIZE
                )
                {
                    return 0.0f;
                }


                float sumSquares =
                    0.0f;


                size_t sampleCount =
                    0;


                for (
                    int offset =
                        -RMS_DIAGNOSTIC_RADIUS;
                    offset <=
                        RMS_DIAGNOSTIC_RADIUS;
                    offset++
                )
                {
                    const int neighborLag =
                        static_cast<int>(
                            physicalLag
                        )
                        +
                        offset;


                    if (
                        neighborLag < 0
                        ||
                        neighborLag >=
                            static_cast<int>(
                                Config::
                                    CORRELATION_SIZE
                            )
                    )
                    {
                        continue;
                    }


                    const size_t absoluteLag =
                        referenceLag
                        +
                        static_cast<size_t>(
                            neighborLag
                        );


                    if (
                        absoluteLag >
                        Config::MAX_LAG
                    )
                    {
                        continue;
                    }


                    const float normalizedSample =
                        correlation[
                            absoluteLag
                        ]
                        /
                        normalizationValue;


                    sumSquares +=
                        normalizedSample
                        *
                        normalizedSample;


                    sampleCount++;
                }


                if (sampleCount == 0)
                {
                    return 0.0f;
                }


                return sqrtf(
                    sumSquares
                    /
                    static_cast<float>(
                        sampleCount
                    )
                );
            };


            // ------------------------------------------
            // Exceso energetico respecto al maximo
            // observado durante calibracion.
            // ------------------------------------------

            auto rmsEnergyResidual =
                [&](
                    size_t physicalLag
                )
                -> float
            {
                if (
                    physicalLag >=
                    Config::CORRELATION_SIZE
                )
                {
                    return 0.0f;
                }


                const float currentEnergy =
                    rmsEnergyAt(
                        physicalLag
                    );


                const float change =
                    currentEnergy
                    -
                    backgroundEnergyMax[
                        physicalLag
                    ];


                if (change <= 0.0f)
                {
                    return 0.0f;
                }


                return change;
            };


            // ------------------------------------------
            // Insertar candidato en TOP 10.
            // ------------------------------------------

            auto insertRmsPeak =
                [&](
                    size_t lag,
                    float score,
                    float energy
                )
            {
                size_t insertionIndex =
                    rmsPeakCount;


                for (
                    size_t i = 0;
                    i < rmsPeakCount;
                    i++
                )
                {
                    if (
                        score >
                            rmsPeaks[
                                i
                            ].score
                        ||
                        (
                            score ==
                                rmsPeaks[
                                    i
                                ].score
                            &&
                            energy >
                                rmsPeaks[
                                    i
                                ].strength
                        )
                    )
                    {
                        insertionIndex =
                            i;

                        break;
                    }
                }


                if (
                    rmsPeakCount <
                    RMS_DIAGNOSTIC_MAX
                )
                {
                    for (
                        size_t i =
                            rmsPeakCount;
                        i >
                            insertionIndex;
                        i--
                    )
                    {
                        rmsPeaks[i] =
                            rmsPeaks[
                                i - 1
                            ];
                    }


                    rmsPeakCount++;
                }
                else
                {
                    if (
                        insertionIndex >=
                        RMS_DIAGNOSTIC_MAX
                    )
                    {
                        return;
                    }


                    for (
                        size_t i =
                            RMS_DIAGNOSTIC_MAX
                            - 1;
                        i >
                            insertionIndex;
                        i--
                    )
                    {
                        rmsPeaks[i] =
                            rmsPeaks[
                                i - 1
                            ];
                    }
                }


                rmsPeaks[
                    insertionIndex
                ].lag =
                    lag;


                rmsPeaks[
                    insertionIndex
                ].score =
                    score;


                rmsPeaks[
                    insertionIndex
                ].strength =
                    energy;
            };


            // ------------------------------------------
            // Buscar maximos locales del exceso RMS.
            // ------------------------------------------

            for (
                size_t physicalLag =
                    firstPhysicalLag;
                physicalLag <=
                    lastPhysicalLag;
                physicalLag++
            )
            {
                const float score =
                    rmsEnergyResidual(
                        physicalLag
                    );


                if (score <= 0.0f)
                {
                    continue;
                }


                const float previousScore =
                    rmsEnergyResidual(
                        physicalLag - 1
                    );


                const float nextScore =
                    rmsEnergyResidual(
                        physicalLag + 1
                    );


                const bool localMaximum =
                    score >
                        previousScore
                    &&
                    score >=
                        nextScore;


                if (!localMaximum)
                {
                    continue;
                }


                const float energy =
                    rmsEnergyAt(
                        physicalLag
                    );


                insertRmsPeak(
                    physicalLag,
                    score,
                    energy
                );
            }


            Serial.println();
            Serial.println(
                "--- DIAGNOSTICO ENVOLVENTE RMS ---"
            );


            Serial.println(
                "Ventana RMS: +/-4 muestras"
            );


            if (rmsPeakCount == 0)
            {
                Serial.println(
                    "Sin excesos RMS positivos."
                );
            }
            else
            {
                for (
                    size_t i = 0;
                    i < rmsPeakCount;
                    i++
                )
                {
                    const size_t lag =
                        rmsPeaks[
                            i
                        ].lag;


                    Serial.printf(
                        "RMS %u: "
                        "lag=%u, "
                        "distancia=%.3f m, "
                        "energia=%.1f %%, "
                        "fondo_max=%.1f %%, "
                        "exceso=%.1f %%\n",
                        static_cast<unsigned>(
                            i + 1
                        ),
                        static_cast<unsigned>(
                            lag
                        ),
                        RadarMath::
                            samplesToDistance(
                                lag
                            ),
                        rmsPeaks[
                            i
                        ].strength *
                            100.0f,
                        backgroundEnergyMax[
                            lag
                        ] *
                            100.0f,
                        rmsPeaks[
                            i
                        ].score *
                            100.0f
                    );
                }
            }


            Serial.println(
                "--- FIN DIAGNOSTICO RMS ---"
            );

            // ==================================================
            // DIAGNOSTICO BIDIRECCIONAL RANGO COMPLETO
            //
            // Analiza TODOS los retardos del rango fisico,
            // no solamente los picos detectados por FFT.
            //
            // Exceso positivo:
            //
            //   actual - maximo_local_del_fondo
            //
            // Exceso negativo:
            //
            //   minimo_local_del_fondo - actual
            //
            // Se usa una envolvente local de +/-2 muestras
            // para ambos limites.
            //
            // Solo se imprimen los 10 maximos locales mas
            // fuertes de cada direccion.
            //
            // ESTE BLOQUE ES SOLO DIAGNOSTICO.
            //
            // NO modifica:
            // - positiveResidual()
            // - trackingCandidates
            // - NMS
            // - tracker
            // - umbrales
            // - distancia seleccionada
            // ==================================================

            constexpr size_t
                BIDIRECTIONAL_DIAGNOSTIC_MAX =
                    10;


            constexpr int
                BIDIRECTIONAL_BACKGROUND_RADIUS =
                    2;


            TrackingCandidate
                bidirectionalPositive[
                    BIDIRECTIONAL_DIAGNOSTIC_MAX
                ];


            TrackingCandidate
                bidirectionalNegative[
                    BIDIRECTIONAL_DIAGNOSTIC_MAX
                ];


            size_t bidirectionalPositiveCount =
                0;


            size_t bidirectionalNegativeCount =
                0;


            // ------------------------------------------
            // Insertar un pico en un TOP 10 ordenado.
            // ------------------------------------------

            auto insertDiagnosticPeak =
                [&](
                    TrackingCandidate* peaks,
                    size_t& peakCount,
                    size_t lag,
                    float score,
                    float strength
                )
            {
                size_t insertionIndex =
                    peakCount;


                for (
                    size_t i = 0;
                    i < peakCount;
                    i++
                )
                {
                    if (
                        score >
                            peaks[i].score
                        ||
                        (
                            score ==
                                peaks[i].score
                            &&
                            strength >
                                peaks[i].strength
                        )
                    )
                    {
                        insertionIndex =
                            i;

                        break;
                    }
                }


                if (
                    peakCount <
                    BIDIRECTIONAL_DIAGNOSTIC_MAX
                )
                {
                    for (
                        size_t i =
                            peakCount;
                        i >
                            insertionIndex;
                        i--
                    )
                    {
                        peaks[i] =
                            peaks[
                                i - 1
                            ];
                    }


                    peakCount++;
                }
                else
                {
                    if (
                        insertionIndex >=
                        BIDIRECTIONAL_DIAGNOSTIC_MAX
                    )
                    {
                        return;
                    }


                    for (
                        size_t i =
                            BIDIRECTIONAL_DIAGNOSTIC_MAX
                            - 1;
                        i >
                            insertionIndex;
                        i--
                    )
                    {
                        peaks[i] =
                            peaks[
                                i - 1
                            ];
                    }
                }


                peaks[
                    insertionIndex
                ].lag =
                    lag;


                peaks[
                    insertionIndex
                ].score =
                    score;


                peaks[
                    insertionIndex
                ].strength =
                    strength;
            };


            // ------------------------------------------
            // Residual negativo respecto al limite
            // inferior local del fondo.
            // ------------------------------------------

            auto negativeResidual =
                [&](
                    size_t physicalLag
                )
                -> float
            {
                if (
                    physicalLag >=
                    Config::CORRELATION_SIZE
                )
                {
                    return 0.0f;
                }


                const size_t absoluteLag =
                    referenceLag +
                    physicalLag;


                if (
                    absoluteLag >
                    Config::MAX_LAG
                )
                {
                    return 0.0f;
                }


                const float currentNormalized =
                    correlation[
                        absoluteLag
                    ]
                    /
                    normalizationValue;


                bool haveLocalMinimum =
                    false;


                float localBackgroundMin =
                    0.0f;


                for (
                    int offset =
                        -BIDIRECTIONAL_BACKGROUND_RADIUS;
                    offset <=
                        BIDIRECTIONAL_BACKGROUND_RADIUS;
                    offset++
                )
                {
                    const int backgroundLag =
                        static_cast<int>(
                            physicalLag
                        )
                        +
                        offset;


                    if (
                        backgroundLag < 0
                        ||
                        backgroundLag >=
                            static_cast<int>(
                                Config::
                                    CORRELATION_SIZE
                            )
                    )
                    {
                        continue;
                    }


                    const size_t backgroundIndex =
                        static_cast<size_t>(
                            backgroundLag
                        );


                    if (
                        backgroundCount[
                            backgroundIndex
                        ] == 0
                    )
                    {
                        continue;
                    }


                    const float backgroundValue =
                        backgroundMin[
                            backgroundIndex
                        ];


                    if (
                        !haveLocalMinimum
                        ||
                        backgroundValue <
                            localBackgroundMin
                    )
                    {
                        localBackgroundMin =
                            backgroundValue;

                        haveLocalMinimum =
                            true;
                    }
                }


                if (!haveLocalMinimum)
                {
                    return 0.0f;
                }


                const float change =
                    localBackgroundMin
                    -
                    currentNormalized;


                if (change <= 0.0f)
                {
                    return 0.0f;
                }


                return change;
            };


            // ------------------------------------------
            // Recorrer TODO el rango de 48 a 80 cm.
            //
            // Conservamos solamente maximos locales de
            // cada curva para no llenar el TOP 10 con
            // muestras vecinas del mismo lobulo.
            // ------------------------------------------

            for (
                size_t physicalLag =
                    firstPhysicalLag;
                physicalLag <=
                    lastPhysicalLag;
                physicalLag++
            )
            {
                const size_t absoluteLag =
                    referenceLag +
                    physicalLag;


                const float currentNormalized =
                    correlation[
                        absoluteLag
                    ]
                    /
                    normalizationValue;


                // ======================================
                // Cambio positivo
                // ======================================

                const float positiveScore =
                    positiveResidual(
                        physicalLag
                    );


                if (positiveScore > 0.0f)
                {
                    const float
                        previousPositiveScore =
                            positiveResidual(
                                physicalLag - 1
                            );


                    const float
                        nextPositiveScore =
                            positiveResidual(
                                physicalLag + 1
                            );


                    const bool positiveLocalMaximum =
                        positiveScore >
                            previousPositiveScore
                        &&
                        positiveScore >=
                            nextPositiveScore;


                    if (positiveLocalMaximum)
                    {
                        insertDiagnosticPeak(
                            bidirectionalPositive,
                            bidirectionalPositiveCount,
                            physicalLag,
                            positiveScore,
                            currentNormalized
                        );
                    }
                }


                // ======================================
                // Cambio negativo
                // ======================================

                const float negativeScore =
                    negativeResidual(
                        physicalLag
                    );


                if (negativeScore > 0.0f)
                {
                    const float
                        previousNegativeScore =
                            negativeResidual(
                                physicalLag - 1
                            );


                    const float
                        nextNegativeScore =
                            negativeResidual(
                                physicalLag + 1
                            );


                    const bool negativeLocalMaximum =
                        negativeScore >
                            previousNegativeScore
                        &&
                        negativeScore >=
                            nextNegativeScore;


                    if (negativeLocalMaximum)
                    {
                        insertDiagnosticPeak(
                            bidirectionalNegative,
                            bidirectionalNegativeCount,
                            physicalLag,
                            negativeScore,
                            currentNormalized
                        );
                    }
                }
            }


            Serial.println();
            Serial.println(
                "--- DIAGNOSTICO BIDIRECCIONAL RANGO COMPLETO ---"
            );


            // ==========================================
            // TOP positivo
            // ==========================================

            Serial.println(
                "TOP CAMBIOS POSITIVOS:"
            );


            if (
                bidirectionalPositiveCount ==
                0
            )
            {
                Serial.println(
                    "Sin excesos positivos."
                );
            }
            else
            {
                for (
                    size_t i = 0;
                    i <
                        bidirectionalPositiveCount;
                    i++
                )
                {
                    Serial.printf(
                        "BIDIR +%u: "
                        "lag=%u, "
                        "distancia=%.3f m, "
                        "actual=%.1f %%, "
                        "exceso=%.1f %%\n",
                        static_cast<unsigned>(
                            i + 1
                        ),
                        static_cast<unsigned>(
                            bidirectionalPositive[
                                i
                            ].lag
                        ),
                        RadarMath::
                            samplesToDistance(
                                bidirectionalPositive[
                                    i
                                ].lag
                            ),
                        bidirectionalPositive[
                            i
                        ].strength *
                            100.0f,
                        bidirectionalPositive[
                            i
                        ].score *
                            100.0f
                    );
                }
            }


            // ==========================================
            // TOP negativo
            // ==========================================

            Serial.println(
                "TOP CAMBIOS NEGATIVOS:"
            );


            if (
                bidirectionalNegativeCount ==
                0
            )
            {
                Serial.println(
                    "Sin excesos negativos."
                );
            }
            else
            {
                for (
                    size_t i = 0;
                    i <
                        bidirectionalNegativeCount;
                    i++
                )
                {
                    Serial.printf(
                        "BIDIR -%u: "
                        "lag=%u, "
                        "distancia=%.3f m, "
                        "actual=%.1f %%, "
                        "exceso=%.1f %%\n",
                        static_cast<unsigned>(
                            i + 1
                        ),
                        static_cast<unsigned>(
                            bidirectionalNegative[
                                i
                            ].lag
                        ),
                        RadarMath::
                            samplesToDistance(
                                bidirectionalNegative[
                                    i
                                ].lag
                            ),
                        bidirectionalNegative[
                            i
                        ].strength *
                            100.0f,
                        bidirectionalNegative[
                            i
                        ].score *
                            100.0f
                    );
                }
            }


            Serial.println(
                "--- FIN DIAGNOSTICO BIDIRECCIONAL ---"
            );


            // ==================================================
            // DIAGNOSTICO PRE-NMS
            //
            // Recorremos TODOS los maximos locales positivos
            // del residual antes de aplicar la supresion
            // espacial MIN_PEAK_SEPARATION.
            //
            // Se conservan solamente los 10 de mayor cambio
            // para evitar saturar el puerto Serial.
            //
            // Este bloque NO modifica:
            //
            // - trackingCandidates
            // - maximumCandidateScore
            // - NMS
            // - tracker
            // - umbrales
            // - seleccion final
            // ==================================================

            constexpr size_t
                PRE_NMS_DIAGNOSTIC_MAX =
                    10;


            TrackingCandidate preNmsPeaks[
                PRE_NMS_DIAGNOSTIC_MAX
            ];


            size_t preNmsPeakCount =
                0;


            for (
                size_t physicalLag =
                    firstPhysicalLag;
                physicalLag <=
                    lastPhysicalLag;
                physicalLag++
            )
            {
                const float score =
                    positiveResidual(
                        physicalLag
                    );


                if (score <= 0.0f)
                {
                    continue;
                }


                const float previousScore =
                    positiveResidual(
                        physicalLag - 1
                    );


                const float nextScore =
                    positiveResidual(
                        physicalLag + 1
                    );


                const bool localMaximum =
                    score >
                        previousScore
                    &&
                    score >=
                        nextScore;


                if (!localMaximum)
                {
                    continue;
                }


                const size_t absoluteLag =
                    referenceLag +
                    physicalLag;


                const float strength =
                    correlation[
                        absoluteLag
                    ]
                    /
                    normalizationValue;


                if (strength <= 0.0f)
                {
                    continue;
                }


                // ------------------------------------------
                // Buscar posicion en el TOP 10.
                //
                // Orden:
                // 1. mayor residual
                // 2. mayor fuerza en caso de empate
                //
                // IMPORTANTE:
                // aqui NO existe separacion espacial.
                // ------------------------------------------

                size_t insertionIndex =
                    preNmsPeakCount;


                for (
                    size_t i = 0;
                    i < preNmsPeakCount;
                    i++
                )
                {
                    if (
                        score >
                            preNmsPeaks[
                                i
                            ].score
                        ||
                        (
                            score ==
                                preNmsPeaks[
                                    i
                                ].score
                            &&
                            strength >
                                preNmsPeaks[
                                    i
                                ].strength
                        )
                    )
                    {
                        insertionIndex =
                            i;

                        break;
                    }
                }


                if (
                    preNmsPeakCount <
                    PRE_NMS_DIAGNOSTIC_MAX
                )
                {
                    for (
                        size_t i =
                            preNmsPeakCount;
                        i >
                            insertionIndex;
                        i--
                    )
                    {
                        preNmsPeaks[i] =
                            preNmsPeaks[
                                i - 1
                            ];
                    }


                    preNmsPeakCount++;
                }
                else
                {
                    if (
                        insertionIndex >=
                        PRE_NMS_DIAGNOSTIC_MAX
                    )
                    {
                        continue;
                    }


                    for (
                        size_t i =
                            PRE_NMS_DIAGNOSTIC_MAX
                            - 1;
                        i >
                            insertionIndex;
                        i--
                    )
                    {
                        preNmsPeaks[i] =
                            preNmsPeaks[
                                i - 1
                            ];
                    }
                }


                preNmsPeaks[
                    insertionIndex
                ].lag =
                    physicalLag;


                preNmsPeaks[
                    insertionIndex
                ].score =
                    score;


                preNmsPeaks[
                    insertionIndex
                ].strength =
                    strength;
            }


            Serial.println();
            Serial.println(
                "--- MAXIMOS RESIDUALES PRE-NMS ---"
            );


            if (preNmsPeakCount == 0)
            {
                Serial.println(
                    "Sin maximos positivos."
                );
            }
            else
            {
                for (
                    size_t i = 0;
                    i < preNmsPeakCount;
                    i++
                )
                {
                    Serial.printf(
                        "PRE-NMS %u: "
                        "lag=%u, "
                        "distancia=%.3f m, "
                        "fuerza=%.1f %%, "
                        "cambio=%.1f %%\n",
                        static_cast<unsigned>(
                            i + 1
                        ),
                        static_cast<unsigned>(
                            preNmsPeaks[
                                i
                            ].lag
                        ),
                        RadarMath::
                            samplesToDistance(
                                preNmsPeaks[
                                    i
                                ].lag
                            ),
                        preNmsPeaks[
                            i
                        ].strength *
                            100.0f,
                        preNmsPeaks[
                            i
                        ].score *
                            100.0f
                    );
                }
            }


            Serial.println(
                "--- FIN MAXIMOS PRE-NMS ---"
            );


            // ==================================================
            // Supresion no maxima del residual
            //
            // En cada pasada escogemos el maximo local con
            // mayor residual que no este demasiado cerca de
            // un candidato ya conservado.
            //
            // Asi eliminamos la "peineta" de maximos cercanos
            // sin favorecer automaticamente los retardos bajos.
            // ==================================================

            for (
                size_t selection = 0;
                selection <
                    MAX_TRACKING_CANDIDATES;
                selection++
            )
            {
                bool bestFound =
                    false;

                TrackingCandidate bestCandidate =
                {
                    0,
                    0.0f,
                    0.0f
                };


                for (
                    size_t physicalLag =
                        firstPhysicalLag;
                    physicalLag <=
                        lastPhysicalLag;
                    physicalLag++
                )
                {
                    const float score =
                        positiveResidual(
                            physicalLag
                        );

                    if (score <= 0.0f)
                    {
                        continue;
                    }


                    const float previousScore =
                        positiveResidual(
                            physicalLag - 1
                        );

                    const float nextScore =
                        positiveResidual(
                            physicalLag + 1
                        );


                    const bool localMaximum =
                        score >
                            previousScore
                        &&
                        score >=
                            nextScore;

                    if (!localMaximum)
                    {
                        continue;
                    }


                    const size_t absoluteLag =
                        referenceLag +
                        physicalLag;

                    const float strength =
                        correlation[
                            absoluteLag
                        ]
                        /
                        normalizationValue;


                    // Un residual positivo puede aparecer incluso
                    // cuando la correlacion actual es negativa.
                    // Ese punto no se conserva como eco fisico.
                    if (strength <= 0.0f)
                    {
                        continue;
                    }


                    // ------------------------------------------
                    // Supresion espacial
                    // ------------------------------------------

                    bool tooClose =
                        false;

                    for (
                        size_t i = 0;
                        i <
                            trackingCandidateCount;
                        i++
                    )
                    {
                        const size_t separation =
                            (
                                physicalLag >
                                trackingCandidates[
                                    i
                                ].lag
                            )
                            ?
                            physicalLag -
                                trackingCandidates[
                                    i
                                ].lag
                            :
                            trackingCandidates[
                                i
                            ].lag -
                                physicalLag;


                        if (
                            separation <
                            Config::
                                MIN_PEAK_SEPARATION
                        )
                        {
                            tooClose =
                                true;

                            break;
                        }
                    }

                    if (tooClose)
                    {
                        continue;
                    }


                    // ------------------------------------------
                    // Mejor maximo disponible en esta pasada
                    // ------------------------------------------

                    if (
                        !bestFound
                        ||
                        score >
                            bestCandidate.score
                        ||
                        (
                            score ==
                                bestCandidate.score
                            &&
                            strength >
                                bestCandidate.strength
                        )
                    )
                    {
                        bestFound =
                            true;

                        bestCandidate.lag =
                            physicalLag;

                        bestCandidate.score =
                            score;

                        bestCandidate.strength =
                            strength;
                    }
                }


                if (!bestFound)
                {
                    break;
                }


                trackingCandidates[
                    trackingCandidateCount
                ] =
                    bestCandidate;

                trackingCandidateCount++;


                if (
                    bestCandidate.score >
                    maximumCandidateScore
                )
                {
                    maximumCandidateScore =
                        bestCandidate.score;
                }
            }
        }

        Serial.println();
        Serial.println(
            "--- CANDIDATOS DE OBJETO ---"
        );


        for (
            size_t i = 0;
            i < trackingCandidateCount;
            i++
        )
        {
            const TrackingCandidate& candidate =
                trackingCandidates[i];

            Serial.printf(
                "Candidato %u: lag=%u, distancia=%.3f m, fuerza=%.1f %%, cambio=%.1f %%\n",
                static_cast<unsigned>(
                    i + 1
                ),
                static_cast<unsigned>(
                    candidate.lag
                ),
                RadarMath::samplesToDistance(
                    candidate.lag
                ),
                candidate.strength *
                    100.0f,
                candidate.score *
                    100.0f
            );
        }

        Serial.println();
        Serial.printf(
            "Referencia de alineacion: %u muestras\n",
            static_cast<unsigned>(
                referenceLag
            )
        );

        // ==================================================
        // Seleccionar candidato
        // ==================================================

        bool candidateSelected =
            false;

        size_t selectedPhysicalLag =
            0;

        float selectedScore =
            0.0f;

        float selectedStrength =
            0.0f;

        size_t bestTrackingDifference =
            999999;

        // Durante adquisición inicial conservamos el gate
        // relativo. Durante seguimiento basta el umbral mínimo
        // y la continuidad temporal.
        const float relativeThreshold =
            maximumCandidateScore
            *
            TRACKING_RELATIVE_SCORE_GATE;

        float requiredScore =
            TRACKING_KEEP_CHANGE;

        if (!trackingActive)
        {
            requiredScore =
                TRACKING_ACQUIRE_CHANGE;

            if (
                relativeThreshold >
                requiredScore
            )
            {
                requiredScore =
                    relativeThreshold;
            }
        }
        else
        {
            // ==============================================
            // Gate relativo durante seguimiento
            //
            // Primero buscamos el mejor cambio solamente
            // entre candidatos temporalmente alcanzables.
            // ==============================================

            float bestNearbyTrackingScore =
                0.0f;

            for (
                size_t i = 0;
                i < trackingCandidateCount;
                i++
            )
            {
                const TrackingCandidate& candidate =
                    trackingCandidates[i];

                if (
                    candidate.score <
                    TRACKING_KEEP_CHANGE
                )
                {
                    continue;
                }

                const size_t trackingDifference =
                    (
                        candidate.lag >
                        trackedLag
                    )
                    ?
                    candidate.lag -
                        trackedLag
                    :
                    trackedLag -
                        candidate.lag;

                if (
                    trackingDifference >
                    TRACKING_MAX_JUMP
                )
                {
                    continue;
                }

                if (
                    candidate.score >
                    bestNearbyTrackingScore
                )
                {
                    bestNearbyTrackingScore =
                        candidate.score;
                }
            }


            const float trackingRelativeThreshold =
                bestNearbyTrackingScore
                *
                TRACKING_RELATIVE_SCORE_GATE;

            if (
                trackingRelativeThreshold >
                requiredScore
            )
            {
                requiredScore =
                    trackingRelativeThreshold;
            }
        }

        for (
            size_t i = 0;
            i < trackingCandidateCount;
            i++
        )
        {
            const TrackingCandidate& candidate =
                trackingCandidates[i];

            if (
                candidate.score <
                requiredScore
            )
            {
                continue;
            }

            // ==============================================
            // ADQUISICIÓN INICIAL
            // ==============================================

            if (!trackingActive)
            {

                const float scoreDifference =
                    (
                        candidate.score >
                        selectedScore
                    )
                    ?
                    candidate.score -
                        selectedScore
                    :
                    selectedScore -
                        candidate.score;

                if (
                    !candidateSelected
                    ||
                    candidate.score >
                        selectedScore +
                        0.01f
                    ||
                    (
                        scoreDifference <=
                            0.01f
                        &&
                        candidate.strength >
                            selectedStrength
                    )
                )
                {
                    candidateSelected =
                        true;

                    selectedPhysicalLag =
                        candidate.lag;

                    selectedScore =
                        candidate.score;

                    selectedStrength =
                        candidate.strength;
                }

                continue;
            }

            // ==============================================
            // SEGUIMIENTO TEMPORAL
            //
            // 1) El candidato debe quedar a no más de
            //    TRACKING_MAX_JUMP muestras del anterior.
            // 2) Dentro de esa ventana se escoge el que
            //    presenta MAYOR cambio respecto al fondo.
            // ==============================================

            const size_t trackingDifference =
                (
                    candidate.lag >
                    trackedLag
                )
                ?
                candidate.lag -
                    trackedLag
                :
                trackedLag -
                    candidate.lag;

            if (
                trackingDifference >
                TRACKING_MAX_JUMP
            )
            {
                continue;
            }

            if (
                !candidateSelected
                ||
                trackingDifference <
                    bestTrackingDifference
                ||
                (
                    trackingDifference ==
                        bestTrackingDifference
                    &&
                    candidate.score >
                        selectedScore
                )
            )
            {
                candidateSelected =
                    true;

                bestTrackingDifference =
                    trackingDifference;

                selectedPhysicalLag =
                    candidate.lag;

                selectedScore =
                    candidate.score;

                selectedStrength =
                    candidate.strength;
            }
        }

        // ==================================================
        // No se encontró candidato
        // ==================================================

        if (!candidateSelected)
        {
            if (trackingActive)
            {
                trackingMisses++;

                if (
                    trackingMisses <=
                    TRACKING_MAX_MISSES
                )
                {
                    const size_t filteredLag =
                        static_cast<size_t>(
                            filteredTrackedLag +
                            0.5f
                        );

                    Serial.printf(
                        "Seguimiento temporal: captura perdida %u / %u\n",
                        static_cast<unsigned>(
                            trackingMisses
                        ),
                        static_cast<unsigned>(
                            TRACKING_MAX_MISSES
                        )
                    );
                    Serial.println(
                        "Objetivo: SEGUIDO"
                    );
                    Serial.printf(
                        "Distancia mantenida: %.3f m\n",
                        lagToPhysicalDistance(
                            filteredLag
                        )
                    );

                    return;
                }
            }

            trackingActive =
                false;
            trackedLag =
                0;
            filteredTrackedLag =
                0.0f;
            trackingMisses =
                0;

            Serial.println(
                "Objetivo: NO DETECTADO"
            );

            return;
        }

        // ==================================================
        // Actualizar seguimiento
        // ==================================================

        if (!trackingActive)
        {
            trackingActive =
                true;

            trackedLag =
                selectedPhysicalLag;

            filteredTrackedLag =
                static_cast<float>(
                    selectedPhysicalLag
                );
        }
        else
        {
            trackedLag =
                selectedPhysicalLag;

            filteredTrackedLag =
                TRACKING_FILTER_ALPHA
                *
                static_cast<float>(
                    selectedPhysicalLag
                )
                +
                (
                    1.0f -
                    TRACKING_FILTER_ALPHA
                )
                *
                filteredTrackedLag;
        }

        trackingMisses =
            0;

        const size_t filteredLag =
            static_cast<size_t>(
                filteredTrackedLag +
                0.5f
            );

        Serial.printf(
            "Cambio seleccionado: %.1f %%\n",
            selectedScore *
            100.0f
        );

        Serial.printf(
            "Fuerza seleccionada: %.1f %%\n",
            selectedStrength *
            100.0f
        );

        Serial.printf(
            "Retardo instantaneo: %u muestras\n",
            static_cast<unsigned>(
                selectedPhysicalLag
            )
        );

        Serial.printf(
            "Retardo filtrado: %u muestras\n",
            static_cast<unsigned>(
                filteredLag
            )
        );

        Serial.println(
            "Objetivo: SEGUIDO"
        );

        Serial.printf(
            "Distancia estimada: %.3f m\n",
            lagToPhysicalDistance(
                filteredLag
            )
        );
    }
}
