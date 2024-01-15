import numpy as np
from Testing_of_rho_matrix import rho_m
from Torque_calculations import system_torque

# th1 = np.pi/6
# th2 = np.pi/3
# th1dot = 0
# th2dot = 1

def M_matrix (th1, th2, th1dot, th2dot):
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
    M = np.zeros((2, 2))
    T = np.zeros((2, 2))
    thddot = np.array([[0, 1], [1, 0]])
    th_1ddot = thddot[0]
    th_2ddot = thddot[1]
    for i in range(2):
       th1ddot = th_1ddot[i]
       th2ddot = th_2ddot[i]
       torque_1, torque_2 = system_torque(th1, th2, a1, a2, th1dot, th2dot, m1, m2, x1, y1, z1, x2, y2, z2, th1ddot,
                                          th2ddot, ixx1, iyy1, izz1, ixy1, ixz1, iyz1, ixx2, iyy2, izz2, ixy2, ixz2, iyz2, g=9.8)
       T[0][i] = torque_1
       T[1][i] = torque_2
       M[0][i] = T[0][i] - rho_m[0]
       M[1][i] = T[1][i] - rho_m[1]
    return M


