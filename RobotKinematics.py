import numpy as np
import CoordinateTransformations as C
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

class RobotKinematicsModel:
    """
    Robot Manipualator Kinematics
        - Forward Kinematics
        - Inverse Kinematics
        - Jacobians
        - Trajectory Generation
        - Velocity and Statics Forces
    """

    def __init__(self):
        pass

    def numberOfLinks(self, n):
        assert type(n) == int, "numberOfLinks must be an int"
        self.numberOfLinks = n
    
    def numberOfActuators(self, m):
        assert type(m) == int, "numberOfActuators must be an int"
        self.numberOfActuators = m    
    
    def set_a_arr(self, a_arr):
        self.a_arr = np.array(a_arr)
    
    def set_alpha_arr(self, alpha_arr):
        self.alpha_arr = np.array(alpha_arr)
    
    def set_d_arr(self, d_arr):
        self.d_arr = np.array(d_arr)
    
    def set_theta_arr(self, theta_arr):
        self.theta_arr = np.array(theta_arr)

    def set_angle_unit(self, unit):
        assert (unit.lower()=="deg" or unit.lower()=="rad"), "angle must be either rad or deg"
        self.unit=unit

    def get_angle_unit(self):
        return self.unit
    
    def _FK(self, plotting=False, isPlot2D=False):
        n = len(self.a_arr)
        assert (len(self.alpha_arr)==n and len(self.d_arr)==n and len(self.theta_arr)==n) # TODO

        T_array = np.zeros((n,), dtype=object)
        
        # to compute T01, T12, ..., T(N-1)(N)
        for i in range(0,n):
            T_array[i] = self._DH_transformation(self.a_arr[i], self.alpha_arr[i], self.d_arr[i], self.theta_arr[i])

        # to compute T_0N = T01 * T12 * T23 ... T(N-1)(N)
        T_0N = C.T_compound_v2(T_array)

        if plotting:
            self._FK_plotter(T_array, planar=isPlot2D)

        return T_0N

    def _IK(self, fk_func, x_desired, y_desired, z_desired, DOF, q_init=0, plotting=False):
        if not q_init:
            q_init = np.zeros((self.m,))
        T_0N = fk_func(q_init)
        e = np.array([x_desired, y_desired, z_desired]).reshape(3,1) - T_0N[:3,3].reshape(3,1)
        q_vec = np.array(q_init)

        tol = 1e-3

        i= 0
        while (abs(e) > tol).any():
            J = self._Jacobian_matrix(fk_func, q_vec, DOF)
            q_vec_new = q_vec.reshape(self.m,1) + self._Jacobian_inverse(J)*e[:DOF]
            q_vec = q_vec_new
            i += 1
            T_0N = fk_func(q_vec_new)
            e = np.array([x_desired, y_desired, z_desired]).reshape(3,1) - T_0N[:3,3].reshape(3,1)
            if i == 500:
                print("process terminated")
                break
        
        if plotting:
            fk_func(q_vec, plotting=plotting)

        return q_vec

    @staticmethod
    def _FK_plotter(T_array, planar=False):
        """
        T_array = T01, T12, ... T(N-1)(N)
        planar = True # if want 2D plot
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
            print(f"T_0{i} = {T_temp}")
        
        fig = plt.figure(figsize=(16,9))
        if planar:
            plt.plot(X,Y)
            plt.xlabel("X")
            plt.xlabel("Y")
        else:
            ax = plt.axes(projection='3d')
            ax.plot3D(X, Y, Z, 'gray')
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
        plt.show()

    def _Jacobian_matrix(self, fk_func, q_vec, DOF):
        # J --  n x m matrix
        # DOF -- number of DOF, 2 for planar and 3 for 3D
        # m -- number of actuators
        delta = 0.01
        q_vec = np.array(q_vec, dtype=float)
        J = np.zeros((DOF,self.m))
        for i in range(self.m):
            q_vec_t = q_vec
            q_vec_t[i] += delta
            T_0N_1 = fk_func(q_vec_t)
            q_vec_t = q_vec
            q_vec_t[i] -= delta
            T_0N_2 = fk_func(q_vec_t)
            J[:,i] = ((T_0N_1[:3,3][:DOF] - T_0N_2[:3,3][:DOF]) / (2*delta)).reshape(DOF,)

        return J*2

    def _DH_transformation(self, a, alpha, d, theta):
        T_x = C.Screw_X(a, alpha, unit=self.get_angle_unit())
        T_z = C.Screw_Z(d, theta, unit=self.get_angle_unit())
        T = C.T_compound(T_x,T_z)
        return T
    
    @staticmethod
    def _Jacobian_inverse(J):
    # pseudo-inverse
        return np.linalg.pinv(J)


if __name__ == "__main__":
    
    r3 = RobotKinematicsModel()
    r3.numberOfLinks(3)
    r3.numberOfActuators(2)
    q1 , q2, q3 = np.deg2rad(10), np.deg2rad(20), np.deg2rad(30)  # Joint Variables
    r3.set_angle_unit('rad')
    r3.set_a_arr([0,4,3,2])
    r3.set_alpha_arr([0,0,0,0])
    r3.set_d_arr([0,0,0,0])
    r3.set_theta_arr([q1,q2,q3,0])
    print(r3._FK())

