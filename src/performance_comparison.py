import numpy as np
from time import perf_counter

from direct_correlation import direct_correlation
from fft_correlation import fft_correlation


def measure_execution_time(function, reference, received, repetitions=5):
    """Mide el tiempo promedio de ejecución de una función."""

    # Lista para almacenar los tiempos de cada ejecución
    execution_times = []

    # Ejecutar varias veces para reducir variaciones
    for _ in range(repetitions):

        # Registrar tiempo inicial
        start_time = perf_counter()

        # Ejecutar función
        function(
            reference,
            received
        )

        # Registrar tiempo final
        end_time = perf_counter()

        # Guardar tiempo transcurrido
        execution_times.append(
            end_time - start_time
        )

    # Calcular tiempo promedio
    average_time = np.mean(
        execution_times
    )

    return average_time


def compare_execution_times(
    sizes,
    repetitions=5,
    random_seed=12345
):
    """Compara los tiempos de correlación directa y mediante FFT."""

    # Generador de números aleatorios
    rng = np.random.default_rng(
        random_seed
    )

    # Lista para almacenar resultados
    results = []

    # Probar cada tamaño
    for size in sizes:

        # Generar señal de referencia
        reference = rng.normal(
            loc=0.0,
            scale=1.0,
            size=size
        )

        # Generar señal recibida con el doble de longitud
        received = rng.normal(
            loc=0.0,
            scale=1.0,
            size=2 * size
        )

        # Calentar implementación directa
        direct_correlation(
            reference,
            received
        )

        # Calentar implementación FFT
        fft_correlation(
            reference,
            received
        )

        # Medir correlación directa
        direct_time = measure_execution_time(
            direct_correlation,
            reference,
            received,
            repetitions
        )

        # Medir correlación mediante FFT
        fft_time = measure_execution_time(
            fft_correlation,
            reference,
            received,
            repetitions
        )

        # Guardar resultados
        results.append(
            (
                size,
                direct_time,
                fft_time
            )
        )

    return results