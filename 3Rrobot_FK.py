import numpy as np
from Forward_Kinematics import Forward_Kinematics

def 3Rrobot_FK(q1,q2,q3,unit='deg',plotting=False):

    # constants
    L1 = 4 # m
    L2 = 3 # m
    L3 = 2 # m

    # DH Parameters
    # for link 1    2    3    H
    a = [      0,  L1,   L2, L3]
    alpha = [  0,   0,    0,  0]
    d = [      0,   0,    0,  0]
    theta = [ q1,  q2,  q3,   0]

    if unit.lower() == 'rad':
        alpha = np.deg2rad(alpha)

    # Forward Kinematics
    T_0N = Forward_Kinematics(a, alpha, d, theta, unit=unit, plotting=plotting)
    return T_0N

if __name__ == "__main__":

    print(puma560_FK(np.deg2rad(90),0,np.deg2rad(90),0,0,0,unit='rad',plotting=True))


# 3DOF 3R Robot



# Joint Variables
#    theta1   theta2   theta3
# q = [  0,       0,        0]
# q = [ 10,       20,       30]
q = [ 90,       90,       90]


# D-H Parameters
Robot_3R = pd.DataFrame({
    #    link(i):  1   2   3   H
    'a(i-1)':     [0, L1, L2, L3],
    'alpha(i-1)': [0,  0,  0,  0],
    'd(i)':       [0,  0,  0,  0],
    'theta(i)':   [q[0], q[1], q[2], 0]
})

Robot_3R