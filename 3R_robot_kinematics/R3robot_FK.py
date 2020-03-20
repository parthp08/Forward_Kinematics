import numpy as np
from Forward_Kinematics import Forward_Kinematics

def R3robot_FK(q1,q2,q3,unit='deg',plotting=False,include_hand=False):

    # constants
    L1 = 4 # m
    L2 = 3 # m
    # L3 = 2 # m

    # DH Parameters
    # for link 1    2    3    H
    a = [      0,  L1,   L2]#, L3]
    alpha = [  0,   0,    0]#,  0]
    d = [      0,   0,    0]#,  0]
    theta = [ q1,  q2,  q3]#,   0]
    
    if include_hand: # tool attached on link 3
        L3 = 2 # m  # constant
        a.append(L3)
        alpha.append(0)
        d.append(0)
        theta.append(0)

    if unit.lower() == 'rad':
        alpha = np.deg2rad(alpha)

    # Forward Kinematics
    T_0N = Forward_Kinematics(a, alpha, d, theta, unit=unit, plotting=plotting)
    return T_0N

if __name__ == "__main__":

    print(R3robot_FK(10,20,30,unit='deg',plotting=True,include_hand=False))
    print(R3robot_FK(np.deg2rad(10), np.deg2rad(20), np.deg2rad(30),unit='rad'))
    