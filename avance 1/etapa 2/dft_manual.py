"""
Responsabilidad: calcular la Transformada Discreta de Fourier de forma manual
y demostrar su uso con una señal sinusoidal sencilla.
"""

import numpy as np


# -------- Cálculo manual de la DFT --------
def dft_manual(x):
    """Calcula la DFT de un arreglo unidimensional usando su definición."""

    # Se convierte la entrada en un arreglo y se verifica que sea un vector.
    x = np.asarray(x)
    if x.ndim != 1:
        raise ValueError("x debe ser un arreglo unidimensional")

    # N representa la cantidad total de muestras de la señal de entrada.
    N = len(x)

    # El arreglo X almacena los N coeficientes complejos de la DFT.
    X = np.zeros(N, dtype=complex)

    # k identifica el coeficiente o componente de frecuencia que se calcula.
    for k in range(N):
        # La suma de todos los aportes x[n] comienza en cero para cada k.
        suma = 0.0 + 0.0j

        # n identifica cada muestra de la señal en el dominio del tiempo.
        for n in range(N):
            # Este es el factor exponencial complejo exp(-j*2*pi*k*n/N).
            factor_exponencial = np.exp(-1j * 2 * np.pi * k * n / N)

            # Se acumula el término x[n] multiplicado por el factor complejo.
            suma += x[n] * factor_exponencial

        # X[k] representa el coeficiente complejo de la DFT para el índice k.
        X[k] = suma

    return X


# -------- Prueba con una señal sinusoidal --------
if __name__ == "__main__":
    # Se definen la frecuencia de muestreo, la frecuencia sinusoidal y N muestras.
    fs = 1000
    f0 = 100
    N = 100

    # Se construye el vector de tiempo y la señal x[n] = sin(2*pi*f0*n/fs).
    n = np.arange(N)
    t = n / fs
    x = np.sin(2 * np.pi * f0 * t)

    # Se calcula la DFT mediante la implementación manual.
    X_manual = dft_manual(x)

    # Se muestran resultados parciales para facilitar su lectura en consola.
    # Se muestran 12 valores para incluir el pico esperado en k = 10 (100 Hz).
    cantidad_mostrada = 12
    print("Primeras muestras de la señal:")
    print(x[:cantidad_mostrada])

    print("\nPrimeros coeficientes complejos de la DFT manual:")
    print(X_manual[:cantidad_mostrada])

    print("\nMagnitud de los primeros coeficientes:")
    print(np.abs(X_manual[:cantidad_mostrada]))

    # NumPy se usa solo como referencia para verificar el resultado manual.
    X_numpy = np.fft.fft(x)
    error = np.max(np.abs(X_manual - X_numpy))
    print("\nError máximo respecto a np.fft.fft:")
    print(error)

    # Los dos ciclos recorren N valores cada uno: N * N operaciones principales.
    print("\nComplejidad aproximada de la DFT directa: O(N^2).")
