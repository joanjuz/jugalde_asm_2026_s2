"""
Responsabilidad: calcular manualmente la FFT radix-2 de Cooley-Tukey
y validar sus coeficientes con la DFT manual y con NumPy.
"""

import numpy as np

from dft_manual import dft_manual


# -------- Cálculo manual de la FFT radix-2 --------
def fft_manual(x):
    """Calcula la FFT mediante el algoritmo recursivo Cooley-Tukey radix-2."""

    # La entrada se convierte en arreglo y se comprueba que sea un vector.
    x = np.asarray(x)
    if x.ndim != 1:
        raise ValueError("x debe ser un arreglo unidimensional")

    # N representa la cantidad total de muestras procesadas en esta llamada.
    N = len(x)
    if N == 0:
        raise ValueError("x debe contener al menos una muestra")

    # Radix-2 exige que N sea potencia de 2 para dividirlo sucesivamente entre 2.
    if N & (N - 1):
        raise ValueError("la cantidad de muestras debe ser una potencia de 2")

    # El caso base ocurre con una muestra: su FFT es la propia muestra compleja.
    if N == 1:
        return np.asarray([x[0]], dtype=complex)

    # Se separan pares e impares para formar dos problemas de tamaño N/2.
    x_pares = x[::2]
    x_impares = x[1::2]

    # E[k] y O[k] son las FFT de las muestras pares e impares, respectivamente.
    E = fft_manual(x_pares)
    O = fft_manual(x_impares)

    # X guardará los N coeficientes complejos de la transformada completa.
    X = np.zeros(N, dtype=complex)

    # Cada k permite combinar un coeficiente de E con uno de O.
    for k in range(N // 2):
        # W_N^k es el factor de giro complejo exp(-j*2*pi*k/N).
        W_N_k = np.exp(-1j * 2 * np.pi * k / N)

        # El producto aplica el factor de giro al coeficiente impar O[k].
        termino_impar = W_N_k * O[k]

        # Estas ecuaciones producen las mitades superior e inferior de X.
        X[k] = E[k] + termino_impar
        X[k + N // 2] = E[k] - termino_impar

    # X contiene los mismos coeficientes DFT, calculados de manera más eficiente.
    return X


# -------- Validación con una señal sinusoidal --------
if __name__ == "__main__":
    # N = 128 es una potencia de 2 y, por ello, es compatible con radix-2.
    fs = 1024
    f0 = 128
    N = 128

    # Se construye una sinusoidal de f0 Hz muestreada a fs Hz.
    n = np.arange(N)
    t = n / fs
    x = np.sin(2 * np.pi * f0 * t)

    # Se calculan los coeficientes mediante la FFT manual.
    X_fft_manual = fft_manual(x)

    # Se muestran suficientes valores para incluir el pico esperado en k = 16.
    cantidad_mostrada = 18
    print("Primeras muestras de la señal:")
    print(x[:cantidad_mostrada])

    print("\nPrimeros coeficientes complejos de la FFT manual:")
    print(X_fft_manual[:cantidad_mostrada])

    print("\nMagnitudes de los primeros coeficientes:")
    print(np.abs(X_fft_manual[:cantidad_mostrada]))

    # NumPy se utiliza después del algoritmo manual y solo como referencia.
    X_numpy = np.fft.fft(x)
    error = np.max(np.abs(X_fft_manual - X_numpy))
    print("\nError máximo respecto a np.fft.fft:")
    print(error)

    # La DFT y la FFT calculan los mismos X[k]; cambia el algoritmo empleado.
    X_dft = dft_manual(x)
    error_dft_fft = np.max(np.abs(X_dft - X_fft_manual))
    print("\nError máximo entre la DFT manual y la FFT manual:")
    print(error_dft_fft)

    # La división recursiva en problemas N/2 reduce el trabajo computacional.
    print("\nComplejidad de la DFT directa: O(N^2).")
    print("Complejidad de la FFT radix-2: O(N log2(N)).")
    print("La mejora ocurre al dividir recursivamente cada problema en dos de tamaño N/2.")
