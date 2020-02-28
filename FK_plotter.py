import numpy as np
import CoordinateTransformations as C
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

def FK_plotter(T_array, name='robot'):
    """
    T_array = T01, T12, ... T(N-1)(N)
    """
    n = len(T_array)
    X = np.zeros((n+1,))
    Y = np.zeros((n+1,))
    Z = np.zeros((n+1,))
    X[1] = T_array[0][:3,3][0]
    Y[1] = T_array[0][:3,3][1]
    Z[1] = T_array[0][:3,3][2]
    T_temp = C.T_compound(T_array[0], T_array[1])
    for i in range(2,n+1):
        if i == 2:
            T_temp = C.T_compound(T_array[i-2], T_array[i-1])
        else:
            T_temp = C.T_compound(T_temp, T_array[i-1])
        X[i] = T_temp[:3,3][0]
        Y[i] = T_temp[:3,3][1]
        Z[i] = T_temp[:3,3][2]
    
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(X, Y, Z, 'gray')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(name)
    plt.show()
