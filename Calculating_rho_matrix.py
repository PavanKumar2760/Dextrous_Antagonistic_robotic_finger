import numpy as np
from Torque_calculations import system_torque
# th1 = np.pi/6
# th2 = np.pi/3
# th1dot = 3
# th2dot = 1
th1ddot = 0
th2ddot = 0

def rho_matrix(th1, th2,th1dot,th2dot):
     th1ddot = 0
     th2ddot = 0
     a1 = 3
     a2 = 3
     m1 = 3.94781
     m2 = 4.50275
     x1 = -0.00351
     y1 = -0.00160
     z1 = -0.03139
     x2 = -0.00767
     y2 = 0.16669
     z2 = -0.00355
     ixx1 = 0.00455
     iyy1 = 0.00454
     izz1 = 0.00029
     ixy1 = 0.00000
     ixz1 = -0.0000
     iyz1 = 0.00001
     ixx2 = 0.00032
     iyy2 = 0.00010
     izz2 = 0.00042
     ixy2 = 0.00000
     ixz2 = 0.00000
     iyz2 = -0.0000
     rho = np.zeros([2, 1])
     torque_1, torque_2 = system_torque(th1, th2, a1, a2, th1dot, th2dot, m1, m2, x1, y1, z1, x2, y2, z2, th1ddot,
                                        th2ddot, ixx1, iyy1, izz1, ixy1, ixz1, iyz1, ixx2, iyy2, izz2, ixy2, ixz2, iyz2,
                                        g=9.8)
     #from formula T=T(Θ,Θ ̇,Θ ̈)=M(Θ)Θ ̈+ρ(Θ,Θ ̇)

     #calculating rho matrix
     rho[0] = torque_1
     rho[1] = torque_2
     return rho


