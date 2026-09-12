import numpy as np
import matplotlib.pyplot as plt


# Parámetros de la señal
FS = 48_000          # Frecuencia de muestreo [Hz]
DURATION = 0.020     # Duración del chirp [s]
F_START = 3_000      # Frecuencia inicial [Hz]
F_END = 8_000        # Frecuencia final [Hz]
SOUND_SPEED = 343.0     # Velocidad aproximada del sonido [m/s]
OBJECT_DISTANCE = 1.5   # Distancia al objeto [m]
ECHO_AMPLITUDE = 0.5    # Atenuación simulada del eco


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
def generate_echo(signal, fs, distance, sound_speed, amplitude):
    """Genera una copia retardada y atenuada de una señal."""

    delay_time = (2 * distance) / sound_speed

    delay_samples = round(delay_time * fs)

    received = np.zeros(delay_samples + len(signal))

    received[delay_samples:] = amplitude * signal

    return received, delay_time, delay_samples

def main():
    t, signal = generate_chirp(
        FS,
        DURATION,
        F_START,
        F_END
    )
    received, delay_time, delay_samples = generate_echo(
        signal,
        FS,
        OBJECT_DISTANCE,
        SOUND_SPEED,
        ECHO_AMPLITUDE
    )

    print(f"Frecuencia de muestreo: {FS} Hz")
    print(f"Duración: {DURATION * 1000:.1f} ms")
    print(f"Número de muestras: {len(signal)}")
    print(f"Frecuencia inicial: {F_START} Hz")
    print(f"Frecuencia final: {F_END} Hz")
    print()
    print(f"Distancia simulada: {OBJECT_DISTANCE} m")
    print(f"Tiempo de vuelo: {delay_time * 1000:.3f} ms")
    print(f"Retardo: {delay_samples} muestras")

    plt.plot(t * 1000, signal)

    plt.title("Chirp lineal transmitido")
    plt.xlabel("Tiempo [ms]")
    plt.ylabel("Amplitud")
    plt.grid()

    plt.show()
    received_time = np.arange(len(received)) / FS

    plt.plot(received_time * 1000, received)

    plt.title("Eco acústico simulado")
    plt.xlabel("Tiempo [ms]")
    plt.ylabel("Amplitud")
    plt.grid()

    plt.show()


if __name__ == "__main__":
    main()