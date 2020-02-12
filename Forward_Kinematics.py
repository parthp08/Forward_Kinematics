import numpy as np
import CoordinateTransformations as C
from DH_transformation import DH_transformation
from FK_plotter import FK_plotter
    
def Forward_Kinematics(a_arr, alpha_arr, d_arr, theta_arr, unit='rad', plotting=False):
    """
    TODO
    """
    n = len(a_arr)
    assert (len(alpha_arr)==n and len(d_arr)==n and len(theta_arr)==n) # TODO

    T_array = np.zeros((n,), dtype=object)
    
    # to compute T01, T12, ..., T(N-1)(N)
    for i in range(0,n):
        T_array[i] = DH_transformation(a_arr[i], alpha_arr[i], d_arr[i], theta_arr[i], unit=unit)
    
    # to compute T_0N
    for i in range(0,n):
        if i == 0:
            T_0N = T_array[i]
        else:
            T_0N = C.compound_transforms(T_0N, T_array[i])

    # plotting
    if plotting:
        FK_plotter(T_array)
    
    return T_0N


