import numpy as np


def compare_correlations(correlation_direct, correlation_fft):
    """Compara numéricamente las correlaciones directa y mediante FFT."""

    # Calcular diferencia absoluta entre ambos métodos
    differences = np.abs(
        correlation_direct - correlation_fft
    )

    # Obtener la mayor diferencia encontrada
    maximum_difference = np.max(
        differences
    )

    # Obtener la diferencia promedio
    average_difference = np.mean(
        differences
    )

    return maximum_difference, average_difference