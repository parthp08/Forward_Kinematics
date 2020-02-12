import numpy as np
from Forward_Kinematics import Forward_Kinematics

def puma560_FK(q1,q2,q3,q4,q5,q6,unit='deg',plotting=False):

    # constants
    a2 = 431.8   # mm
    a3 = -20.32  # mm
    d3 = 149.09  # mm
    d4 = 433.07  # mm
    d6 = -20.32  # mm

    # DH Parameters
    # for link 1    2    3      4,      5,      6
    a = [      0,  0,   a2,    a3,      0,      0]
    alpha = [  0, -90,   0,    90,    -90,     90]
    d = [      0,   0,  d3,    d4,      0,     d6]
    theta = [ q1,  q2,  q3,    q4,     q5,     q6]

    if unit.lower() == 'rad':
        alpha = np.deg2rad(alpha)

    # Forward Kinematics
    T_0N = Forward_Kinematics(a, alpha, d, theta, unit=unit, plotting=plotting)
    return T_0N

if __name__ == "__main__":

    print(puma560_FK(np.deg2rad(90),0,np.deg2rad(90),0,0,0,unit='rad',plotting=True))
