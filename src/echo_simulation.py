from config import (
    FS,
    DURATION,
    F_START,
    F_END,
    SOUND_SPEED,
    NOISE_STD,
    RANDOM_SEED,
    ECHOES,
    MIN_DETECTION_DISTANCE,
    PEAK_THRESHOLD_RATIO,
    MIN_PEAK_SEPARATION
)

from signal_simulation import (
    generate_chirp,
    generate_echoes,
    build_received_signal,
    add_noise
)

from direct_correlation import (
    direct_correlation,
    find_maximum_delay,
    find_echo_peaks
)

from plots import (
    plot_transmitted_signal,
    plot_echoes,
    plot_received_signal,
    plot_noise,
    plot_noisy_signal,
    plot_direct_correlation
)


def main():
    # Generar señal transmitida
    t, signal = generate_chirp(
        FS,
        DURATION,
        F_START,
        F_END
    )

    # Generar todos los ecos
    echoes = generate_echoes(
        signal,
        FS,
        ECHOES,
        SOUND_SPEED
    )

    # Crear señal recibida
    received = build_received_signal(
        signal,
        echoes
    )

    # Agregar ruido a la señal recibida
    received_noisy, noise = add_noise(
        received,
        NOISE_STD,
        RANDOM_SEED
    )

    # Calcular correlación directa
    correlation_direct = direct_correlation(
        signal,
        received_noisy
    )

    # Buscar máximo de correlación
    maximum_delay = find_maximum_delay(
        correlation_direct
    )
    # Detectar picos correspondientes a ecos
    detected_peaks = find_echo_peaks(
        correlation_direct,
        FS,
        SOUND_SPEED,
        MIN_DETECTION_DISTANCE,
        PEAK_THRESHOLD_RATIO,
        MIN_PEAK_SEPARATION
    )

    # Mostrar información de la señal
    print(f"Frecuencia de muestreo: {FS} Hz")
    print(f"Duración del chirp: {DURATION * 1000:.1f} ms")
    print(f"Número de muestras del chirp: {len(signal)}")
    print(f"Frecuencia inicial: {F_START} Hz")
    print(f"Frecuencia final: {F_END} Hz")

    # Mostrar información de los ecos
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

    # Mostrar información del ruido
    print(
        f"\nDesviación estándar del ruido: "
        f"{NOISE_STD}"
    )

    # Mostrar resultado de correlación
    print(
        f"\nMáximo de correlación directa: "
        f"{maximum_delay} muestras"
    )

    # Mostrar gráficas
    plot_transmitted_signal(
        t,
        signal
    )

    plot_echoes(
        echoes,
        FS
    )

    plot_received_signal(
        received,
        FS
    )

    plot_noise(
        noise,
        FS
    )

    plot_noisy_signal(
        received_noisy,
        FS
    )

    plot_direct_correlation(
        correlation_direct
    )
    # Mostrar ecos detectados
    print("\nEcos detectados mediante correlación directa:")

    for i, delay in enumerate(
        detected_peaks,
        start=1
    ):
        # Convertir retardo a tiempo
        delay_time = delay / FS

        # Convertir tiempo de vuelo a distancia
        distance = (
            SOUND_SPEED * delay_time
            / 2
        )

        print(
            f"Eco {i}: "
            f"retardo = {delay} muestras, "
            f"distancia = {distance:.3f} m"
    )


if __name__ == "__main__":
    main()