import numpy as np
import CoordinateTransformations as C

def DH_transformation(a, alpha, d, theta, unit='rad'):
    """
    TODO
    """

    T_x = C.Screw_X(a, alpha, unit=unit)
    T_z = C.Screw_Z(d, theta, unit=unit)
    T = C.T_compound(T_x,T_z)
    return T
