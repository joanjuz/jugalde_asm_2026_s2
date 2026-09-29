# Parámetros de la señal
FS = 48_000          # Frecuencia de muestreo [Hz]
DURATION = 0.020     # Duración del chirp [s]
F_START = 3_000      # Frecuencia inicial [Hz]
F_END = 8_000        # Frecuencia final [Hz]

# Parámetros físicos
SOUND_SPEED = 343.0  # Velocidad aproximada del sonido [m/s]

# Parámetros del ruido
NOISE_STD = 0.15      # Desviación estándar del ruido
RANDOM_SEED = 12345   # Semilla para reproducibilidad

# Ecos simulados:
# (distancia al objeto [m], amplitud relativa del eco)
ECHOES = [
    (1.0, 0.7),
    (1.5, 0.5),
    (2.2, 0.3)
]
# Parámetros para detección de ecos
MIN_DETECTION_DISTANCE = 0.20  # Distancia mínima detectable [m]
PEAK_THRESHOLD_RATIO = 0.30    # Umbral relativo para detección de picos
MIN_PEAK_SEPARATION = 50       # Separación mínima entre picos [muestras]

# Parámetros para comparación de rendimiento
PERFORMANCE_SIZES = [
    128,
    256,
    512,
    1024,
    2048,
    4096
]

PERFORMANCE_REPETITIONS = 5