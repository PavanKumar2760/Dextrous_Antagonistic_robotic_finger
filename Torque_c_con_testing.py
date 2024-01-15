import numpy as np
from Torque_calculations import system_torque
from Trajectory_Testing import q, qd, qdd
import matplotlib.pyplot as plt

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
g = 9.8
m = 100

time_steps = np.linspace(0, 1, num=len(q[0]))
th_1 = q[0]
th_2 = q[1]
th1_dot = qd[0]
th2_dot = qd[1]
th_1ddot = qdd[0]
th_2ddot = qdd[1]
link_torques = np.zeros((2, len(q[0])))
for i in range(len(q[0])):
    th1 = th_1[i]
    th2 = th_2[i]
    th1ddot = th_1ddot[i]
    th2ddot = th_2ddot[i]
    th1dot = th1_dot[i]
    th2dot = th2_dot[i]
    torque_1, torque_2 = system_torque(th1, th2, a1, a2, th1dot, th2dot, m1, m2, x1, y1, z1, x2, y2, z2, th1ddot, th2ddot,
                                     ixx1, iyy1, izz1, ixy1, ixz1, iyz1, ixx2, iyy2, izz2, ixy2, ixz2, iyz2, g=9.8)
    link_torques[0][i] = torque_1
    link_torques[1][i] = torque_2

# plotting the torques

for i in range(len(q[0])):
    plt.plot(link_torques[0], label ="link_1_torque")
    plt.plot(link_torques[1], label ="link_2_torque")
    plt.title(f'torques_testing in each step{i}')
    plt.legend()
    plt.show()
print(link_torques)
