"""
Responsabilidad: comparar los tiempos de la DFT y la FFT manuales, estudiar
la magnitud y la fase de tres señales, y guardar las gráficas de los experimentos.
"""

from pathlib import Path
import time

import matplotlib

#  permite guardar las imágenes sin abrir ventanas gráficas.
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from dft_manual import dft_manual
from fft_manual import fft_manual


# -------- Configuración de salida --------
# La ruta se construye junto al programa para no depender del directorio de ejecución.
CARPETA_IMAGENES = Path(__file__).resolve().parent / "imagenes"


# -------- Limpieza de resultados anteriores --------
def limpiar_imagenes_anteriores():
    """Elimina solamente los archivos PNG de ejecuciones anteriores."""

    # La limpieza evita mezclar gráficas antiguas con las generadas actualmente.
    for archivo in CARPETA_IMAGENES.glob("*.png"):
        archivo.unlink()


# -------- Medición y comparación de tiempos --------
def medir_tiempo(funcion, x, repeticiones=10):
    """Devuelve el tiempo promedio dedicado exclusivamente a ejecutar una función."""

    # El calentamiento reduce el efecto de las condiciones iniciales y no se cronometra.
    funcion(x)

    # Las mediciones individuales permiten promediar variaciones puntuales del sistema.
    tiempos = []
    for _ in range(repeticiones):
        inicio = time.perf_counter()
        funcion(x)
        fin = time.perf_counter()
        tiempos.append(fin - inicio)

    # Varias repeticiones reducen la influencia de cambios momentáneos del sistema operativo.
    return np.mean(tiempos)


def comparar_tiempos():
    """Compara los tiempos promedio de la DFT y la FFT para varias potencias de 2."""

    # La FFT radix-2 requiere longitudes que sean potencias de 2.
    tamanos = np.asarray([16, 32, 64, 128, 256, 512])
    tiempos_dft = np.zeros(len(tamanos))
    tiempos_fft = np.zeros(len(tamanos))
    factores_aceleracion = np.zeros(len(tamanos))
    fs = 1024
    frecuencia = 64
    repeticiones = 10

    print("COMPARACION DE TIEMPOS")
    print(f"{'N':>6} {'DFT (s)':>16} {'FFT (s)':>16} {'DFT/FFT':>14}")

    for indice, N in enumerate(tamanos):
        # Esta frecuencia coincide con un bin para todos los tamaños seleccionados.
        n = np.arange(N)
        x = np.sin(2 * np.pi * frecuencia * n / fs)

        # Ambas funciones reciben exactamente la misma señal para una comparación justa.
        tiempos_dft[indice] = medir_tiempo(dft_manual, x, repeticiones)
        tiempos_fft[indice] = medir_tiempo(fft_manual, x, repeticiones)

        # DFT/FFT expresa cuántas veces tarda más la DFT en esta medición.
        factores_aceleracion[indice] = tiempos_dft[indice] / tiempos_fft[indice]
        print(
            f"{N:6d} {tiempos_dft[indice]:16.8f} "
            f"{tiempos_fft[indice]:16.8f} {factores_aceleracion[indice]:14.2f}"
        )

    # Para N grande se espera que O(N log2(N)) crezca menos que O(N^2).
    plt.figure(figsize=(8, 5))
    plt.plot(tamanos, tiempos_dft, marker="o", label="DFT manual")
    plt.plot(tamanos, tiempos_fft, marker="o", label="FFT manual")
    plt.title("Comparación de tiempos: DFT manual vs. FFT manual")
    plt.xlabel("Número de muestras N")
    plt.ylabel("Tiempo promedio [s]")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(CARPETA_IMAGENES / "comparacion_tiempos_dft_fft.png", dpi=150)
    plt.close()

    # La escala logarítmica hace visibles ambas curvas cuando sus tiempos difieren mucho.
    plt.figure(figsize=(8, 5))
    plt.plot(tamanos, tiempos_dft, marker="o", label="DFT manual")
    plt.plot(tamanos, tiempos_fft, marker="o", label="FFT manual")
    plt.yscale("log")
    plt.title("Comparación de tiempos DFT vs. FFT - escala logarítmica")
    plt.xlabel("Número de muestras N")
    plt.ylabel("Tiempo promedio [s]")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(CARPETA_IMAGENES / "comparacion_tiempos_dft_fft_log.png", dpi=150)
    plt.close()

    return tamanos, tiempos_dft, tiempos_fft, factores_aceleracion


# -------- Preparación de magnitud y fase --------
def preparar_magnitud_fase(X):
    """Calcula la magnitud y conserva solo las fases espectralmente significativas."""

    # La magnitud |X[k]| mide el aporte del coeficiente de frecuencia k.
    magnitud = np.abs(X)

    # La fase indica el ángulo de cada coeficiente complejo, expresado en radianes.
    fase = np.angle(X)

    # La fase es inestable y no tiene interpretación útil si la magnitud es casi cero.
    tolerancia = 1e-10 * np.max(magnitud)
    fase_filtrada = fase.copy()
    fase_filtrada[magnitud <= tolerancia] = np.nan

    return magnitud, fase_filtrada, tolerancia


def obtener_frecuencias_principales(frecuencias, magnitud, tolerancia):
    """Obtiene las frecuencias positivas cuyos coeficientes superan la tolerancia."""

    # La misma máscara usada para la fase evita reportar residuos numéricos como picos.
    indices = np.where(magnitud > tolerancia)[0]
    return frecuencias[indices]


# -------- Gráficas de cada experimento --------
def graficar_senal_temporal(t, x, numero, descripcion):
    """Guarda una gráfica de la señal en el dominio temporal."""

    plt.figure(figsize=(8, 4.5))
    plt.plot(t, x, marker=".")
    plt.title(f"Señal {numero}: {descripcion} - dominio temporal")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Amplitud")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(CARPETA_IMAGENES / f"senal_{numero}_tiempo.png", dpi=150)
    plt.close()


def graficar_magnitud(frecuencias, magnitud, numero, descripcion):
    """Guarda la magnitud sin normalizar de los coeficientes calculados."""

    plt.figure(figsize=(8, 4.5))
    plt.plot(frecuencias, magnitud, marker="o")
    plt.title(f"Señal {numero}: {descripcion} - espectro de magnitud")
    plt.xlabel("Frecuencia [Hz]")
    plt.ylabel("Magnitud |X[k]|")
    plt.xlim(frecuencias[0], frecuencias[-1])
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(CARPETA_IMAGENES / f"senal_{numero}_magnitud.png", dpi=150)
    plt.close()


def graficar_fase(frecuencias, fase, numero, descripcion):
    """Guarda la fase de los coeficientes cuya magnitud es significativa."""

    plt.figure(figsize=(8, 4.5))
    plt.plot(frecuencias, fase, marker="o", linestyle="none")
    plt.title(f"Señal {numero}: {descripcion} - espectro de fase")
    plt.xlabel("Frecuencia [Hz]")
    plt.ylabel("Fase [rad]")
    plt.xlim(frecuencias[0], frecuencias[-1])
    plt.ylim(-np.pi, np.pi)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(CARPETA_IMAGENES / f"senal_{numero}_fase.png", dpi=150)
    plt.close()


# -------- Experimentos de magnitud y fase --------
def analizar_senal(x, t, fs, numero, descripcion):
    """Calcula la FFT manual, crea sus tres gráficas y devuelve sus frecuencias principales."""

    # La FFT manual calcula los mismos X[k] definidos por la DFT, con menor costo.
    X = fft_manual(x)
    magnitud, fase, tolerancia = preparar_magnitud_fase(X)
    N = len(x)

    # Cada k corresponde a f[k] = k*fs/N; fs/N es la resolución frecuencial.
    frecuencias = np.arange(N) * fs / N

    # Para una señal real se presenta solo el intervalo no negativo hasta fs/2.
    limite_positivo = N // 2 + 1
    frecuencias_positivas = frecuencias[:limite_positivo]
    magnitud_positiva = magnitud[:limite_positivo]
    fase_positiva = fase[:limite_positivo]

    graficar_senal_temporal(t, x, numero, descripcion)
    graficar_magnitud(frecuencias_positivas, magnitud_positiva, numero, descripcion)
    graficar_fase(frecuencias_positivas, fase_positiva, numero, descripcion)

    return obtener_frecuencias_principales(
        frecuencias_positivas, magnitud_positiva, tolerancia
    )


def experimentos_espectrales():
    """Ejecuta los experimentos de una sinusoidal, una suma y una señal desfasada."""

    fs = 1024
    N = 128
    n = np.arange(N)
    t = n / fs

    # Señal 1: una sinusoidal simple de 128 Hz.
    x1 = np.sin(2 * np.pi * 128 * t)
    principales_1 = analizar_senal(x1, t, fs, 1, "sinusoidal de 128 Hz")

    # Señal 2: suma de 64 Hz y 192 Hz; la segunda componente tiene amplitud 0.5.
    x2 = np.sin(2 * np.pi * 64 * t) + 0.5 * np.sin(2 * np.pi * 192 * t)
    principales_2 = analizar_senal(x2, t, fs, 2, "sinusoides de 64 Hz y 192 Hz")

    # Señal 3: el desfase cambia la fase espectral, pero mantiene el pico en 128 Hz.
    phi = np.pi / 4
    x3 = np.sin(2 * np.pi * 128 * t + phi)
    principales_3 = analizar_senal(x3, t, fs, 3, "sinusoidal de 128 Hz con fase pi/4")

    # Una verificación numérica confirma que ambos algoritmos producen los mismos X[k].
    X_dft = dft_manual(x1)
    X_fft = fft_manual(x1)
    error = np.max(np.abs(X_dft - X_fft))

    print("\nVALIDACION NUMERICA")
    print(f"Error máximo entre DFT manual y FFT manual: {error:.6e}")
    print("\nFRECUENCIAS PRINCIPALES DETECTADAS")
    print(f"Señal 1: {principales_1} Hz")
    print(f"Señal 2: {principales_2} Hz")
    print(f"Señal 3: {principales_3} Hz")


# -------- Ejecución principal --------
if __name__ == "__main__":
    # La carpeta separa todas las figuras generadas del código fuente.
    CARPETA_IMAGENES.mkdir(parents=True, exist_ok=True)
    limpiar_imagenes_anteriores()

    comparar_tiempos()
    experimentos_espectrales()

    # Resumen conceptual de la diferencia de costo entre ambos algoritmos.
    print("\nCOMPLEJIDAD COMPUTACIONAL")
    print("DFT directa: O(N^2).")
    print("FFT radix-2: O(N log2(N)), debido a la división recursiva en problemas N/2.")
    print(f"\nLas imágenes se guardaron en: {CARPETA_IMAGENES}")
