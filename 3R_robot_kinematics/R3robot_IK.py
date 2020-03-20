# Inverse Kinematics 
# Analytical Solution 
# Given in "Intro to Robotics by craig" p.109

import numpy as np

def R3robot_IK(T_0N, output_unit='deg', solution1or2=1):
    """
    TODO
    """

    # constants
    L1 = 4 # m
    L2 = 3 # m
    L3 = 2 # m

    # 3 parameters for Planar Robots
    # x, y, phi
    cos_phi = T_03[0,0]
    sin_phi = T_03[0,1]
    phi = np.arccos(cos_phi)
    x = T_03[0,3]
    y = T_03[1,3]
    if x==0 and y==0:
        return "theta 1 is arbitrary for (x,y)=(0,0)"
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2*L1*L2)
    print(cos_theta2)
    if cos_phi<-1 or cos_phi>1:
        return "solution does not exist"
    if solution1or2==1: # first solution
        sin_theta2 = (1 - (cos_theta2**2))**0.5
        theta2 = np.arctan2(sin_theta2, cos_theta2)
        k2 = L2*sin_theta2
    elif solution1or2==2: # seconds solution
        sin_theta2 = -1*(1 - (cos_theta2**2))**0.5
        theta2 = np.arctan2(sin_theta2, cos_theta2) # second solution
        k2 = L2*sin_theta2
    k1 = L1 + L2*cos_theta2
    theta1 = np.arctan2(y,x) - np.arctan2(k2,k1)
    theta3 = phi - theta1 - theta2

    if output_unit.lower()=="deg":
        return np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3)
    return theta1, theta2, theta3

if __name__ == "__main__":
    from R3robot_FK import R3robot_FK
    T_03 = R3robot_FK(10,20,30,unit='deg')
    print(R3robot_IK(T_03,solution1or2=2))
