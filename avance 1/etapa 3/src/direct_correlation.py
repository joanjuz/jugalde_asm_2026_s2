import numpy as np


def direct_correlation(reference, received):
    """Calcula la correlación directa para retardos no negativos."""

    # Mayor retardo que puede analizarse sin salir del arreglo
    max_delay = len(received) - len(reference)

    # Arreglo donde se guardará la correlación de cada retardo
    correlation = np.zeros(max_delay + 1)

    # Probar cada posible retardo
    for delay in range(max_delay + 1):

        # Extraer una sección de la señal recibida
        segment = received[
            delay:delay + len(reference)
        ]

        # Calcular similitud entre el chirp y la sección recibida
        correlation[delay] = np.sum(
            reference * segment
        )

    return correlation


def find_maximum_delay(correlation):
    """Encuentra el retardo con mayor magnitud de correlación."""

    # Buscar posición del máximo de correlación
    maximum_delay = np.argmax(
        np.abs(correlation)
    )

    return maximum_delay


def find_echo_peaks(
    correlation,
    fs,
    sound_speed,
    min_distance,
    threshold_ratio,
    min_peak_separation
):
    """Detecta los picos asociados a ecos en la correlación."""

    # Magnitud de la correlación
    magnitude = np.abs(correlation)

    # Convertir distancia mínima a retardo mínimo en muestras
    min_delay = round(
        (2 * min_distance / sound_speed) * fs
    )

    # Obtener el máximo fuera de la zona de la señal directa
    search_maximum = np.max(
        magnitude[min_delay:]
    )

    # Calcular umbral mínimo para aceptar un pico
    threshold = threshold_ratio * search_maximum

    # Lista de posibles picos
    candidates = []

    # Buscar máximos locales
    for delay in range(
        min_delay,
        len(magnitude)
    ):

        # Obtener valores vecinos
        left = (
            magnitude[delay - 1]
            if delay > 0
            else -np.inf
        )

        right = (
            magnitude[delay + 1]
            if delay < len(magnitude) - 1
            else -np.inf
        )

        # Verificar si el punto es un máximo local
        if (
            magnitude[delay] >= left
            and magnitude[delay] >= right
            and magnitude[delay] >= threshold
        ):
            candidates.append(delay)

    # Ordenar candidatos desde el pico más fuerte
    candidates.sort(
        key=lambda delay: magnitude[delay],
        reverse=True
    )

    # Lista de picos seleccionados
    selected_peaks = []

    # Evitar seleccionar varios picos del mismo eco
    for delay in candidates:

        sufficiently_far = all(
            abs(delay - selected)
            >= min_peak_separation
            for selected in selected_peaks
        )

        if sufficiently_far:
            selected_peaks.append(delay)

    # Ordenar los ecos según su retardo
    selected_peaks.sort()

    return selected_peaks