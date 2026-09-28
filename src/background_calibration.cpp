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

            backgroundSum[
                relativeLag
            ] += normalizedValue;

            backgroundCount[
                relativeLag
            ]++;
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
                        RadarMath::samplesToDistance(
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

        const FFTCorrelation::Detection* detections =
            FFTCorrelation::detections();

        const float* correlation =
            FFTCorrelation::correlationData();

        const size_t minimumPhysicalLag =
            RadarMath::distanceToSamples(
                Config::TARGET_MIN_DISTANCE
            );

        const size_t maximumPhysicalLag =
            RadarMath::distanceToSamples(
                Config::TARGET_MAX_DISTANCE
            );

        TrackingCandidate trackingCandidates[
            MAX_TRACKING_CANDIDATES
        ];

        size_t trackingCandidateCount =
            0;

        float maximumCandidateScore =
            0.0f;

        Serial.println();
        Serial.println(
            "--- CANDIDATOS DE OBJETO ---"
        );

        // ==================================================
        // Construir candidatos usando la MISMA referencia
        // para distancia física y para sustracción de fondo.
        // ==================================================

        for (
            size_t i = 0;
            i < detectionCount;
            i++
        )
        {
            if (
                detections[i].lag <
                referenceLag
            )
            {
                continue;
            }

            const size_t physicalLag =
                detections[i].lag -
                referenceLag;

            const size_t alignmentLag =
                physicalLag;

            if (
                physicalLag <
                    minimumPhysicalLag
                ||
                physicalLag >
                    maximumPhysicalLag
            )
            {
                continue;
            }

            if (
                alignmentLag >=
                Config::CORRELATION_SIZE
            )
            {
                continue;
            }

            const float strength =
                detections[i].correlationValue
                /
                normalizationValue;

            // ==================================================
            // Cambio positivo respecto al fondo
            //
            // Se busca el mejor residual en ±3 muestras para
            // absorber pequeños corrimientos entre capturas.
            // ==================================================

            float score =
                0.0f;

            for (
                int offset =
                    -BACKGROUND_COMPARE_RADIUS;
                offset <=
                    BACKGROUND_COMPARE_RADIUS;
                offset++
            )
            {
                const int candidateAlignment =
                    static_cast<int>(
                        alignmentLag
                    )
                    +
                    offset;

                if (
                    candidateAlignment < 0
                    ||
                    candidateAlignment >=
                        static_cast<int>(
                            Config::CORRELATION_SIZE
                        )
                )
                {
                    continue;
                }

                const size_t compareLag =
                    static_cast<size_t>(
                        candidateAlignment
                    );

                const size_t absoluteLag =
                    referenceLag +
                    compareLag;

                if (
                    absoluteLag >
                    Config::MAX_LAG
                )
                {
                    continue;
                }

                const float currentNormalized =
                    correlation[absoluteLag]
                    /
                    normalizationValue;

                const float change =
                    currentNormalized
                    -
                    backgroundProfile[
                        compareLag
                    ];

                if (change > score)
                {
                    score =
                        change;
                }
            }

            Serial.printf(
                "Candidato %u: lag=%u, distancia=%.3f m, fuerza=%.1f %%, cambio=%.1f %%\n",
                static_cast<unsigned>(
                    i + 1
                ),
                static_cast<unsigned>(
                    physicalLag
                ),
                RadarMath::samplesToDistance(
                    physicalLag
                ),
                strength * 100.0f,
                score * 100.0f
            );

            if (
                trackingCandidateCount <
                MAX_TRACKING_CANDIDATES
            )
            {
                trackingCandidates[
                    trackingCandidateCount
                ].lag =
                    physicalLag;

                trackingCandidates[
                    trackingCandidateCount
                ].score =
                    score;

                trackingCandidates[
                    trackingCandidateCount
                ].strength =
                    strength;

                trackingCandidateCount++;
            }

            if (
                score >
                maximumCandidateScore
            )
            {
                maximumCandidateScore =
                    score;
            }
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

        size_t bestStartDifference =
            999999;

        size_t bestTrackingDifference =
            999999;

        const size_t startLag =
            RadarMath::distanceToSamples(
                TRACKING_START_DISTANCE
            );

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
                const size_t startDifference =
                    (
                        candidate.lag >
                        startLag
                    )
                    ?
                    candidate.lag -
                        startLag
                    :
                    startLag -
                        candidate.lag;

                if (
                    startDifference >
                    TRACKING_START_TOLERANCE
                )
                {
                    continue;
                }

                if (
                    !candidateSelected
                    ||
                    startDifference <
                        bestStartDifference
                    ||
                    (
                        startDifference ==
                            bestStartDifference
                        &&
                        candidate.score >
                            selectedScore
                    )
                )
                {
                    candidateSelected =
                        true;

                    bestStartDifference =
                        startDifference;

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
                candidate.score >
                    selectedScore
                ||
                (
                    candidate.score ==
                        selectedScore
                    &&
                    trackingDifference <
                        bestTrackingDifference
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
                        RadarMath::samplesToDistance(
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
            RadarMath::samplesToDistance(
                filteredLag
            )
        );
    }
}
