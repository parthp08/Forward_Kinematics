from RobotKinematics import RobotKinematicsModel

class R3robot(RobotKinematicsModel):

    # number of links
    n = 3
    # number of actuators
    m = 3

    # constants
    L1 = 4 # m
    L2 = 3 # m
    L3 = 2 # m

    # DH Parameters
    # for link 1    2    3    H
    a = [      0,  L1,   L2, L3]
    alpha = [  0,   0,    0,  0]
    d = [      0,   0,    0,  0]
    # theta = [ q1,  q2,  q3,   0]


    def __init__(self):
        # super.__init__
        
        self.numberOfLinks(self.n)
        self.numberOfActuators(self.m)
        self.set_a_arr(self.a)
        self.set_alpha_arr(self.alpha)
        self.set_d_arr(self.d)
        # self.set_theta_arr(theta)
        
    def set_q1(self, q1):
        self.q1 = q1

    def set_q2(self, q2):
        self.q2 = q2

    def set_q3(self, q3):
        self.q3 = q3

    def fk(self, q_vec=[], unit=None,plotting=False):
        if len(q_vec) != 0:
            self.set_q1(q_vec[0])
            self.set_q2(q_vec[1])
            self.set_q3(q_vec[2])

        if unit:
            self.set_angle_unit(unit)

        self.set_theta_arr([self.q1,self.q2,self.q3,0])

        return self._FK(plotting=plotting, isPlot2D=True)

    def jacobian(self, q_vec=[], unit=None):
        if (len(q_vec) != 0):
            self.set_q1(q_vec[0])
            self.set_q2(q_vec[1])
            self.set_q3(q_vec[2])
        else:
            q_vec = [self.q1,self.q2,self.q3]

        if unit:
            self.set_angle_unit(unit)
        
        self.set_theta_arr([self.q1,self.q2,self.q3, 0])

        return self._Jacobian_matrix(self.fk, q_vec, DOF=2)
    
    def ik(self, x_desired, y_desired, q_init=[], plotting=False):
        return self._IK(self.fk,x_desired,y_desired,0,DOF=2,q_init=q_init,plotting=plotting)


if __name__ == "__main__":
    import numpy as np
    r3 = R3robot()
    print(r3.fk([10,20,30],unit='deg'))
    print(r3.jacobian())
    print(r3.ik(5,6,q_init=[10,20,30], plotting=True))
    print(r3.fk(q_vec=[22.118,35.0749,43.3346],unit='deg',plotting=True)[:2,3])
