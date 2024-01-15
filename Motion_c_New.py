import numpy as np
import matplotlib.pyplot as plt
from gekko import GEKKO
from Calculating_M_matrix import M_matrix
from Calculating_rho_matrix import rho_matrix
from M_t_Matrix_F_E_T_S import M_t
from R_t_matrix_F_E_T_S import R_t
from Calculating_M_matrix import M_matrix
from Calculating_rho_matrix import rho_matrix
m = GEKKO()
N = 1000
# Total time = T
T = m.Var(value=1, lb=0)
# tf = T/N
# time steps
m.time = np.linspace(0, 1, N)

# variables & initial conditions
th1 = m.Var(value=np.pi/3)
th2 = m.Var(value=np.pi/2)
th1dot = m.Var(value=0)
th2dot = m.Var(value=0)
th1ddot = m.Var(value=0)
th2ddot = m.Var(value=0)

u1 = m.Var(value=10)
u2 = m.Var(value=10)
u = np.array([[u1], [u2]])
pos = [[th1], [th2], [th1dot], [th2dot]]
# rho = rho_matrix(th1, th2, th1dot, th2dot)
M_t= M_matrix(th1[0], th2[0], th1dot[0], th2dot[0])
rho_t = rho_matrix(th1[0], th2[0], th1dot[0], th2dot[0])
v8 = th1dot
v9 = th2dot
# Equations

thddot = np.array([[th1ddot], [th2ddot]])
cal = np.linalg.inv(M_t) @ (u - R_t)
m.Equation(th1.dt() == th1dot)
m.Equation((th2.dt() == th2dot))
# m.Equation((th1dot.dt() == th1ddot))
# m.Equation((th2dot.dt() == th2ddot))
m.Equation(th1dot.dt() == cal[0][0])
m.Equation(th2dot.dt() == cal[1][0])

# objective
m.Obj(T)

# Options
m.options.IMODE = 6
m.solve()
plt.figure(1)
plt.plot(m.time, th1, "k:", LineWidth=2, label=r"th1")
plt.plot(m.time, th2, "k:", LineWidth=2, label=r"th1")
plt.plot(m.time, th1dot, "k:", LineWidth=2, label=r"th1")
plt.plot(m.time, th2dot, "k:", LineWidth=2, label=r"th1")
plt.plot(m.time, th1ddot, "k:", LineWidth=2, label=r"th1")
plt.plot(m.time, th2ddot, "k:", LineWidth=2, label=r"th1")
plt.xlabel('Time')
plt.ylabel('Value')
plt.show()
