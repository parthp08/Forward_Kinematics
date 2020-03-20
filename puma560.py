from RobotKinematics import RobotKinematicsModel

class Puma560(RobotKinematicsModel):

    # number of links
    n = 6
    # number of actuators
    m = 6

    # constants
    a2 = 0.431  #m
    a3 = 0.020  #m
    d2 = 0.149  #m
    d4 = 0.432  #m
    d6 = 0.056  #m

    # DH Parameters
    # for link 1    2    3      4,      5,      6
    a = [      0,  a2,   a3,    0,      0,      0]
    alpha = [-90,   0,    90, -90,     90,      0]
    d = [      0,  d2,     0,  d4,      0,     d6]
    # theta = [ q1,  q2,  q3,    q4,     q5,     q6]


    def __init__(self):
        # super.__init__
        
        self.numberOfLinks(6)
        self.numberOfActuators(6)
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
    
    def set_q4(self, q4):
        self.q4 = q4

    def set_q5(self, q5):
        self.q5 = q5
    
    def set_q6(self, q6):
        self.q6 = q6
    
    def fk(self, q_vec=[], unit=None, plotting=False):
        if (len(q_vec) != 0):
            self.set_q1(q_vec[0])
            self.set_q2(q_vec[1])
            self.set_q3(q_vec[2])
            self.set_q4(q_vec[3])
            self.set_q5(q_vec[4])
            self.set_q6(q_vec[5])

        if unit:
            self.set_angle_unit(unit)

        self.set_theta_arr([self.q1,self.q2,self.q3,self.q4,self.q5,self.q6])

        return self._FK(plotting=plotting)

    def jacobian(self, q_vec=[], unit=None):
        if (len(q_vec) != 0):
            self.set_q1(q_vec[0])
            self.set_q2(q_vec[1])
            self.set_q3(q_vec[2])
            self.set_q4(q_vec[3])
            self.set_q5(q_vec[4])
            self.set_q6(q_vec[5])
        else:
            q_vec = [self.q1,self.q2,self.q3,self.q4,self.q5,self.q6]

        if unit:
            self.set_angle_unit(unit)
        
        self.set_theta_arr([self.q1,self.q2,self.q3,self.q4,self.q5,self.q6])

        return self._Jacobian_matrix(self.fk, q_vec, DOF=3)
    
    def ik(self, x_desired, y_desired, z_desired, q_init=[]):
        return self._IK(self.fk,x_desired,y_desired,z_desired,DOF=3,q_init=q_init)

if __name__ == "__main__":
    import numpy as np
    robot = Puma560()
    # print(robot.fk([90,0,90,0,0,0],unit='deg'))
    # print(robot.jacobian())
    robot.set_angle_unit('deg')
    print(robot.ik(1,1,1,q_init=[5,10,10,12,15,7]))
    print(robot.fk(q_vec=[ 1129.15323938,1903.0838016,283.34638565,
            -359.94551629,15, 7], unit='deg', plotting=True))

    ## WTF 
    