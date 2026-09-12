import numpy as np
import matplotlib.pyplot as plt


# Parámetros de la señal
FS = 48_000          # Frecuencia de muestreo [Hz]
DURATION = 0.020     # Duración del chirp [s]
F_START = 3_000      # Frecuencia inicial [Hz]
F_END = 8_000        # Frecuencia final [Hz]


def generate_chirp(fs, duration, f_start, f_end):
    """Genera un chirp lineal entre f_start y f_end."""

    num_samples = int(fs * duration)

    t = np.arange(num_samples) / fs

    k = (f_end - f_start) / duration

    phase = 2 * np.pi * (
        f_start * t
        + 0.5 * k * t**2
    )

    signal = np.sin(phase)

    return t, signal


def main():
    t, signal = generate_chirp(
        FS,
        DURATION,
        F_START,
        F_END
    )

    print(f"Frecuencia de muestreo: {FS} Hz")
    print(f"Duración: {DURATION * 1000:.1f} ms")
    print(f"Número de muestras: {len(signal)}")
    print(f"Frecuencia inicial: {F_START} Hz")
    print(f"Frecuencia final: {F_END} Hz")

    plt.plot(t * 1000, signal)

    plt.title("Chirp lineal transmitido")
    plt.xlabel("Tiempo [ms]")
    plt.ylabel("Amplitud")
    plt.grid()

    plt.show()


if __name__ == "__main__":
    main()