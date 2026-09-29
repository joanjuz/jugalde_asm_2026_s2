import numpy as np
import matplotlib.pyplot as plt


def plot_transmitted_signal(t, signal):
    """Muestra la señal chirp transmitida."""

    # Gráfica de la señal transmitida
    plt.plot(
        t * 1000,
        signal
    )

    plt.title("Chirp lineal transmitido")
    plt.xlabel("Tiempo [ms]")
    plt.ylabel("Amplitud")
    plt.grid()

    plt.show()


def plot_echoes(echoes, fs):
    """Muestra todos los ecos simulados."""

    # Gráfica de los ecos por separado
    for i, (
        echo,
        distance,
        _,
        _,
        _
    ) in enumerate(echoes, start=1):

        echo_time = np.arange(len(echo)) / fs

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


def plot_received_signal(received, fs):
    """Muestra la señal recibida sin ruido."""

    # Vector de tiempo
    received_time = np.arange(len(received)) / fs

    # Gráfica de la señal recibida
    plt.plot(
        received_time * 1000,
        received
    )

    plt.title("Señal recibida con múltiples ecos")
    plt.xlabel("Tiempo [ms]")
    plt.ylabel("Amplitud")
    plt.grid()

    plt.show()


def plot_noise(noise, fs):
    """Muestra el ruido gaussiano generado."""

    # Vector de tiempo
    noise_time = np.arange(len(noise)) / fs

    # Gráfica del ruido gaussiano
    plt.plot(
        noise_time * 1000,
        noise
    )

    plt.title("Ruido gaussiano simulado")
    plt.xlabel("Tiempo [ms]")
    plt.ylabel("Amplitud")
    plt.grid()

    plt.show()


def plot_noisy_signal(received_noisy, fs):
    """Muestra la señal recibida con ruido."""

    # Vector de tiempo
    received_time = np.arange(len(received_noisy)) / fs

    # Gráfica de la señal recibida con ruido
    plt.plot(
        received_time * 1000,
        received_noisy
    )

    plt.title("Señal recibida con ecos y ruido")
    plt.xlabel("Tiempo [ms]")
    plt.ylabel("Amplitud")
    plt.grid()

    plt.show()



def plot_direct_correlation(correlation):
    """Muestra la correlación directa."""

    # Vector de retardos
    delays = np.arange(
        len(correlation)
    )

    # Gráfica de la correlación directa
    plt.plot(
        delays,
        correlation
    )

    plt.title("Correlación directa")
    plt.xlabel("Retardo [muestras]")
    plt.ylabel("Correlación")
    plt.grid()

    plt.show()
    
    
def plot_performance_comparison(results):
    """Muestra la comparación de tiempos entre correlación directa y FFT."""

    # Extraer tamaños y tiempos
    sizes = [
        size
        for size, _, _ in results
    ]

    direct_times = [
        direct_time * 1000
        for _, direct_time, _ in results
    ]

    fft_times = [
        fft_time * 1000
        for _, _, fft_time in results
    ]

    # Gráfica de tiempos
    plt.plot(
        sizes,
        direct_times,
        marker="o",
        label="Correlación directa"
    )

    plt.plot(
        sizes,
        fft_times,
        marker="o",
        label="Correlación FFT"
    )

    plt.title("Comparación de tiempos de ejecución")
    plt.xlabel("Tamaño de la señal N")
    plt.ylabel("Tiempo promedio [ms]")
    plt.grid()
    plt.legend()

    plt.show()
    
def plot_correlation_comparison(correlation_direct, correlation_fft):
    """Compara gráficamente la correlación directa y mediante FFT."""

    # Vector de retardos
    delays = np.arange(
        len(correlation_direct)
    )

    # Correlación directa
    plt.plot(
        delays,
        correlation_direct,
        label="Directa"
    )

    # Correlación mediante FFT
    plt.plot(
        delays,
        correlation_fft,
        linestyle="--",
        label="FFT"
    )

    plt.title("Comparación de métodos de correlación")
    plt.xlabel("Retardo [muestras]")
    plt.ylabel("Correlación")
    plt.grid()
    plt.legend()

    plt.show()