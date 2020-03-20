import numpy as np
import CoordinateTransformations as C
import matplotlib.pyplot as plt

def FK_plotter_2D(T_array):
    """
    T_array = T01, T12, ... T(N-1)(N)
    """
    n = len(T_array)
    X = np.zeros((n+1,))
    Y = np.zeros((n+1,))
    X[1] = T_array[0][:3,3][0]
    Y[1] = T_array[0][:3,3][1]
    # no Z computations
    T_temp = C.T_compound(T_array[0], T_array[1])
    for i in range(2,n+1):
        if i == 2:
            T_temp = C.T_compound(T_array[i-2], T_array[i-1])
        else:
            T_temp = C.T_compound(T_temp, T_array[i-1])
        X[i] = T_temp[:3,3][0]
        Y[i] = T_temp[:3,3][1]
    
    plt.figure(figsize=(16,9))
    plt.plot(X,Y)
    plt.xlabel("X")
    plt.xlabel("Y")
    plt.title("3R Robot")
    plt.show()
