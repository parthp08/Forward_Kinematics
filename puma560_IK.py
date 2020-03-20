import numpy as np
from puma560_FK import puma560_FK

m = 6
n = 6
DOF = 3


# using Finite difference to calculate Jacobian
def Jacobian_matrix(fk_func, q_vec, DOF, m):
    # J --  n x m matrix
    # DOF -- number of DOF, 2 for planar and 3 for 3D
    # m -- number of actuators
    delta = 0.01
    q_vec = np.array(q_vec, dtype=float)
    J = np.zeros((DOF,m))
    for i in range(m):
        q_vec_t = q_vec
        q_vec_t[i] += delta
        T_0N_1 = fk_func(q_vec_t[0],q_vec_t[1],q_vec_t[2],q_vec_t[3],q_vec_t[4],q_vec_t[5])
        q_vec_t = q_vec
        q_vec_t[i] -= delta
        T_0N_2 = fk_func(q_vec_t[0],q_vec_t[1],q_vec_t[2],q_vec_t[3],q_vec_t[4],q_vec_t[5])
        J[:,i] = ((T_0N_1[:3,3][:DOF] - T_0N_2[:3,3][:DOF]) / (2*delta)).reshape(DOF,)
    return J*2

import numpy as np
def Jacobian_inverse(J):
    # pseudo-inverse
    return np.linalg.pinv(J)
    
# Desired position 
x_d = 0.0001
y_d = 0.0001
z_d = 0.0001

# initital guess to the soultion is
q1 = 10
q2 = 10
q3 = 10
q4 = 10
q5 = 10
q6 = 10


def IK(fk_func, x_desired, y_desired, z_desired, DOF, m, q_init=0):
    if not q_init:
        q_init = np.zeros((m,))
    T_0N = fk_func(q_init[0],q_init[1],q_init[2],q_init[3],q_init[4],q_init[5])
    e = np.array([x_desired, y_desired, z_desired]).reshape(3,1) - T_0N[:3,3].reshape(3,1)
    q_vec = np.array(q_init)

    tol = 1e-3

    i= 0
    while (abs(e) > tol).any():
        J = Jacobian_matrix(fk_func, q_vec, DOF, m)
        q_vec_new = q_vec.reshape(m,1) + Jacobian_inverse(J)*e[:DOF]
        q_vec = q_vec_new
        i += 1
        T_0N = fk_func(q_vec_new[0],q_vec_new[1],q_vec_new[2],q_vec_new[3],q_vec_new[4],q_vec_new[5])
        e = np.array([x_desired, y_desired, z_desired]).reshape(3,1) - T_0N[:3,3].reshape(3,1)
        if i == 500:
            print("process terminated")
            break

    return q_vec

if __name__ == "__main__":
    q_vec = IK(puma560_FK, x_d, y_d, z_d, DOF=3, m=6, q_init=[10,10,10,10,10,10])
    print(q_vec)
    print(puma560_FK(q_vec[0][0],q_vec[1][0],q_vec[2][0],q_vec[3][0],q_vec[4][0],q_vec[5][0]))
    # not matching # i think problem is with the units
    # maybe try this with stanford arm
    