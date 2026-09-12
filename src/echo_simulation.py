import numpy as np
import matplotlib.pyplot as plt


# Parámetros de la señal
FS = 48_000          # Frecuencia de muestreo [Hz]
DURATION = 0.020     # Duración del chirp [s]
F_START = 3_000      # Frecuencia inicial [Hz]
F_END = 8_000        # Frecuencia final [Hz]

# Parámetros físicos
SOUND_SPEED = 343.0  # Velocidad aproximada del sonido [m/s]

# Ecos simulados:
# (distancia al objeto [m], amplitud relativa del eco)
ECHOES = [
    (1.0, 0.7),
    (1.5, 0.5),
    (2.2, 0.3)
]


def generate_chirp(fs, duration, f_start, f_end):
    """Genera un chirp lineal entre f_start y f_end."""

    # Cantidad total de muestras
    num_samples = int(fs * duration)

    # Vector de tiempo
    t = np.arange(num_samples) / fs

    # Tasa de cambio de frecuencia del chirp
    k = (f_end - f_start) / duration

    # Fase instantánea
    phase = 2 * np.pi * (
        f_start * t
        + 0.5 * k * t**2
    )

    # Señal chirp
    signal = np.sin(phase)

    return t, signal


def generate_echo(signal, fs, distance, sound_speed, amplitude):
    """Genera una copia retardada y atenuada de una señal."""

    # Tiempo que tarda el sonido en ir al objeto y regresar
    delay_time = (2 * distance) / sound_speed

    # Conversión del tiempo de retardo a número de muestras
    delay_samples = round(delay_time * fs)

    # Se crea un arreglo suficientemente largo para contener el retardo
    echo = np.zeros(delay_samples + len(signal))

    # Se coloca la señal después del retardo y se atenúa su amplitud
    echo[delay_samples:] = amplitude * signal

    return echo, delay_time, delay_samples


def main():
    # ---------------------------------------------------------
    # 1. Generar señal transmitida
    # ---------------------------------------------------------
    t, signal = generate_chirp(
        FS,
        DURATION,
        F_START,
        F_END
    )

    # ---------------------------------------------------------
    # 2. Generar todos los ecos
    # ---------------------------------------------------------
    echoes = []

    for distance, amplitude in ECHOES:
        echo, delay_time, delay_samples = generate_echo(
            signal,
            FS,
            distance,
            SOUND_SPEED,
            amplitude
        )

        echoes.append(
            (
                echo,
                distance,
                amplitude,
                delay_time,
                delay_samples
            )
        )

    # ---------------------------------------------------------
    # 3. Crear señal recibida
    # ---------------------------------------------------------

    # La longitud debe alcanzar para contener el eco más lejano
    received_length = max(
        len(echo)
        for echo, _, _, _, _ in echoes
    )

    # Inicialmente no se recibe ninguna señal
    received = np.zeros(received_length)

    # Agregar señal directa
    received[:len(signal)] += signal

    # Agregar todos los ecos
    for echo, _, _, _, _ in echoes:
        received[:len(echo)] += echo

    # ---------------------------------------------------------
    # 4. Mostrar información
    # ---------------------------------------------------------
    print(f"Frecuencia de muestreo: {FS} Hz")
    print(f"Duración del chirp: {DURATION * 1000:.1f} ms")
    print(f"Número de muestras del chirp: {len(signal)}")
    print(f"Frecuencia inicial: {F_START} Hz")
    print(f"Frecuencia final: {F_END} Hz")

    print("\nEcos simulados:")

    for i, (
        _,
        distance,
        amplitude,
        delay_time,
        delay_samples
    ) in enumerate(echoes, start=1):

        print(
            f"Eco {i}: "
            f"distancia = {distance:.2f} m, "
            f"amplitud = {amplitude:.2f}, "
            f"tiempo = {delay_time * 1000:.3f} ms, "
            f"retardo = {delay_samples} muestras"
        )

    # ---------------------------------------------------------
    # 5. Gráfica de la señal transmitida
    # ---------------------------------------------------------
    plt.plot(t * 1000, signal)

    plt.title("Chirp lineal transmitido")
    plt.xlabel("Tiempo [ms]")
    plt.ylabel("Amplitud")
    plt.grid()

    plt.show()

    # ---------------------------------------------------------
    # 6. Gráfica de los ecos por separado
    # ---------------------------------------------------------
    for i, (
        echo,
        distance,
        _,
        _,
        _
    ) in enumerate(echoes, start=1):

        echo_time = np.arange(len(echo)) / FS

        plt.plot(
            echo_time * 1000,
            echo,
            label=f"Eco {i}: {distance:.1f} m"
        )

    plt.title("Ecos acústicos simulados")
    plt.xlabel("Tiempo [ms]")
    plt.ylabel("Amplitud")
    plt.grid()
    plt.legend()

    plt.show()

    # ---------------------------------------------------------
    # 7. Gráfica de la señal total recibida
    # ---------------------------------------------------------
    received_time = np.arange(len(received)) / FS

    plt.plot(
        received_time * 1000,
        received
    )

    plt.title("Señal recibida con múltiples ecos")
    plt.xlabel("Tiempo [ms]")
    plt.ylabel("Amplitud")
    plt.grid()

    plt.show()


if __name__ == "__main__":
    main()