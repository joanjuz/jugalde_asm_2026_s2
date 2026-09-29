import numpy as np


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


def generate_echoes(signal, fs, echoes_config, sound_speed):
    """Genera todos los ecos configurados."""

    # Lista para almacenar los ecos generados
    echoes = []

    # Generar cada eco
    for distance, amplitude in echoes_config:
        echo, delay_time, delay_samples = generate_echo(
            signal,
            fs,
            distance,
            sound_speed,
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

    return echoes


def build_received_signal(signal, echoes):
    """Combina la señal directa con todos los ecos."""

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

    return received


def add_noise(received, noise_std, random_seed):
    """Agrega ruido gaussiano a la señal recibida."""

    # Crear generador de números aleatorios
    rng = np.random.default_rng(random_seed)

    # Generar ruido gaussiano
    noise = rng.normal(
        loc=0.0,
        scale=noise_std,
        size=len(received)
    )

    # Agregar ruido a la señal recibida
    received_noisy = received + noise

    return received_noisy, noise