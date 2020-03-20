import numpy as np
from R3robot_FK import R3robot_FK
from N_raphson import n_raphson, n_raphson_matrix

# using Finite difference to calculate Jacobian
def Jacobian_matrix(theta1,theta2,theta3):
    tol = 1e-3
    delta = 0.01
    J = np.zeros((3,3))
    # theta1 = 10
    # theta2 = 10
    # theta3 = 10
    # for dp_x / d_theta1 and dp_y / delta_theta1
    T_03_1 = R3robot_FK(theta1+delta, theta2, theta3, unit='deg',include_hand=True)
    T_03_2 = R3robot_FK(theta1-delta, theta2, theta3, unit='deg',include_hand=True)
    J[:,0] = ((T_03_1[:3,3] - T_03_2[:3,3]) / (2*delta)).reshape(3,)
    # for dp_x / d_theta2 and dp_y / delta_theta2
    T_03_1 = R3robot_FK(theta1, theta2+delta, theta3, unit='deg',include_hand=True)
    T_03_2 = R3robot_FK(theta1, theta2-delta, theta3, unit='deg',include_hand=True)
    J[:,1] = ((T_03_1[:3,3] - T_03_2[:3,3]) / (2*delta)).reshape(3,)
    # for dp_x / d_theta3 and dp_y / delta_theta3
    T_03_1 = R3robot_FK(theta1, theta2, theta3+delta, unit='deg',include_hand=True)
    T_03_2 = R3robot_FK(theta1, theta2, theta3-delta, unit='deg',include_hand=True)
    J[:,2] = ((T_03_1[:3,3] - T_03_2[:3,3]) / (2*delta)).reshape(3,)
    # print(J)
    return J  # Find a good way to do this ## 😅

def Jacobian_inverse(J):
    # pseudo-inverse
    return np.linalg.pinv(J)



# Desired position 
x_d = 5
y_d = 6

# initital guess to the soultion is
theta1 = 10#11
theta2 = 20#22
theta3 = 30#33

tol = 1e-3

T_03 = R3robot_FK(theta1,theta2,theta3,include_hand=True)
e = np.array([x_d, y_d, 0]).reshape(3,1) - T_03[:3,3].reshape(3,1)
theta_vec = np.array([theta1, theta2, theta3])

i= 0
while (abs(e) > tol).any():
    J = Jacobian_matrix(theta1,theta2,theta3)
    theta_vec_new = theta_vec.reshape(3,1) + Jacobian_inverse(J)*e
    print(theta_vec_new.shape)
    theta1 = theta_vec_new[0]
    theta2 = theta_vec_new[1]
    theta3 = theta_vec_new[2]
    theta_vec = theta_vec_new
    i += 1
    T_03 = R3robot_FK(theta1,theta2,theta3,include_hand=True)
    e = np.array([x_d, y_d,0]).reshape(3,1) - T_03[:3,3].reshape(3,1)

print(theta1,theta2,theta3)

# TO vertify run this thorugh FK solver and see of the X and Y matches
print((R3robot_FK(theta1[0][0], theta2[0][0], theta3[0][0], include_hand=True))[:3,3][:2])
# YES IT IS 😍😍
print(Jacobian_matrix(10,20,30))
