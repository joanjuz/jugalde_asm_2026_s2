import numpy as np


def fft_correlation(reference, received):
    """Calcula la correlación mediante FFT para retardos no negativos."""

    # Tamaño necesario para obtener una correlación lineal
    linear_size = (
        len(received)
        + len(reference)
        - 1
    )

    # Buscar la siguiente potencia de dos
    # para realizar la FFT de forma eficiente
    fft_size = 1 << (linear_size - 1).bit_length()

    # Transformada de Fourier de la señal de referencia
    reference_fft = np.fft.fft(
        reference,
        fft_size
    )

    # Transformada de Fourier de la señal recibida
    received_fft = np.fft.fft(
        received,
        fft_size
    )

    # Correlación en el dominio de la frecuencia
    correlation_spectrum = (
        received_fft
        * np.conj(reference_fft)
    )

    # Regresar al dominio temporal
    correlation_full = np.fft.ifft(
        correlation_spectrum
    ).real

    # Mayor retardo válido para nuestra señal
    max_delay = (
        len(received)
        - len(reference)
    )

    # Conservar únicamente los retardos no negativos válidos
    correlation = correlation_full[
        :max_delay + 1
    ]

    return correlation