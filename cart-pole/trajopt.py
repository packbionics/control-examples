import numpy as np
import math
import matplotlib.pyplot as plt

from gekko import GEKKO as gk


m = gk(remote=False)
N = 13
T = 1
m.time = np.linspace(0,T,N)

m1 = 2.0
m2 = 0.5
l = 0.5
g = 9.81
x_0 = [0.0,0.1,0.1,0.1]
x_f = [0.5, math.pi, 0.0, 0.0]

pos_lb = -1.0
pos_ub = 1.0
Fmx = 100.0

p = np.zeros(N)
p[-1] = T
final = m.Param(value=p)

x = m.Array(m.Var,(4))

x[0].lower = pos_lb
x[0].upper = pos_ub

for i in range(4):
    x[i].value = x_0[i]

u = m.MV(value=0,lb=-Fmx,ub=Fmx)
u.STATUS = 1


m.Equation(x[0].dt() == x[2])
m.Equation(x[1].dt() == x[3])
m.Equation(x[2].dt() == ((l*m2*m.sin(x[1])*x[3]**2 + g*m2*m.cos(x[1])*m.sin(x[1]) + u)/
                        (m1+m2*(1-m.cos(x[1])**2))))
m.Equation(x[3].dt() == -((l*m2*m.cos(x[1])*m.sin(x[1])*x[3]**2+u*m.cos(x[1])+(m1+m2)*g*m.sin(x[1])) /
                        (l*m1+l*m2*(1-m.cos(x[1])**2))))

m.Minimize(m.integral(u**2)*final)
m.Minimize(1e5*(x[0]*final-x_f[0])**2*final)
m.Minimize(1e5*(x[1]*final-x_f[1])**2*final)
m.Minimize(1e5*(x[2]*final-x_f[2])**2*final)
m.Minimize(1e5*(x[3]*final-x_f[3])**2*final)
m.options.IMODE = 6

try:
    m.solve()
except:
    print('Not successful')
    from gekko.apm import get_file
    print(m._server)
    print(m._model_name)
    f = get_file(m._server,m._model_name,'infeasibilities.txt')
    f = f.decode().replace('\r','')
    with open('infeasibilities.txt', 'w') as fl:
        fl.write(str(f))

fig, axes = plt.subplots(3,1)

axes[0].plot(m.time, x[0].value, 'o')
axes[1].plot(m.time, x[1].value, 'o')
axes[2].plot(m.time, u.value, 'o')

plt.show()