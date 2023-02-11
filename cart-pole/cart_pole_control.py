import math
import time

import matplotlib.pyplot as plt
import numpy as np
from gekko import GEKKO as gk

p, = plt.plot([],[], 'r-')
plt.title('MPC state trajectory')
plt.xlabel('x position')
plt.ylabel('theta')

def update_line(new_data):
    x_data = new_data[0]
    y_data = new_data[1]

    p.set_xdata(x_data)
    p.set_ydata(y_data)
    plt.draw()
    plt.ylim((-5,5))
    plt.xlim((-2,2))
    plt.pause(0.000001)


class CartPoleSwingUpController:

    def __init__(self, gravity=9.81, 
                mass_cart=1, 
                mass_pole=1, 
                length_pole=0.5, 
                k_lqr=np.array([-4.47,80,-6,10.45])):
        self.state = np.zeros((4,1))
        self.k_lqr = k_lqr
        self.gravity = gravity
        self.mass_cart = mass_cart
        self.mass_pole = mass_pole
        self.length_pole = length_pole

    def state_modifier(self, state):
        """
        Changes theta origin to be on the bottom
            to accomodate different derivations
        Keyword arguments:
        state   State vector of system
        """
        return (state[0], state[1], np.pi-state[2], -state[3])

    def state_unpacker(self, state):
        return (state[0], state[1], state[2], state[3])

    def unpack_parameters(self):
        return (self.gravity,
                self.mass_pole,
                self.mass_cart,
                self.length_pole,
                self.k_lqr)

    def state_estimate_callback(self, state):
        state = self.state_unpacker(state)
        self.state[0] = state[0]
        self.state[1] = state[1]
        self.state[2] = state[2]        
        self.state[3] = state[3]
        return state

    def get_action(self):

        # choose LQR if close to the unstable fixed point
        if (abs(self.theta_distance(self.state[2], 0)) < 0.7):
            return self.upright_lqr()
        else: # otherwise choose swingup control
            return self.swingup()

    def theta_distance(self, theta, target):
        """
        Compute the signed angle difference between theta and target
        """
        return math.atan2(math.sin(theta-target), math.cos(theta-target))

    def upright_lqr(self):
        """
        LQR controller
        """
    
        k_lqr = self.k_lqr

        X = self.state
        f = np.dot(k_lqr,X)
        return -f 


class CartPoleMPCController(CartPoleSwingUpController):

    def __init__(self, gravity=9.81, 
                mass_cart=1, 
                mass_pole=1, 
                length_pole=0.5, 
                k_lqr=np.array([-4.47,80,-6,10.45])):

        super().__init__(gravity, 
                mass_cart, 
                mass_pole, 
                length_pole,
                k_lqr)

        self.m = gk(remote=False)

        N = 11
        T = 1
        self.m.time = np.linspace(0,T,N)**2
        p = np.zeros(N)
        p[-1] = T
        final = self.m.Param(value=p)

        m1 = self.mass_cart
        m2 = self.mass_pole
        l = self.length_pole
        g = self.gravity
        x_0 = [0.0,0.0,0.0,0.0]
        x_f = [0.5, math.pi, 0.0, 0.0]

        pos_lb = -1.0
        pos_ub = 1.0
        Fmx = 100.0

        self.x = self.m.Array(self.m.Var,(4))

        self.x[0].lower = pos_lb
        self.x[0].upper = pos_ub

        for i in range(4):
            self.x[i].value = x_0[i]

        self.u = self.m.MV(value=0,lb=-Fmx,ub=Fmx)
        self.u.STATUS = 1

        self.m.Equation(self.x[0].dt() == self.x[2])
        self.m.Equation(self.x[1].dt() == self.x[3])
        self.m.Equation(self.x[2].dt() == ((l*m2*self.m.sin(self.x[1])*self.x[3]**2 + g*m2*self.m.cos(self.x[1])*self.m.sin(self.x[1]) + self.u)/
                                (m1+m2*(1-self.m.cos(self.x[1])**2))))
        self.m.Equation(self.x[3].dt() == -((l*m2*self.m.cos(self.x[1])*self.m.sin(self.x[1])*self.x[3]**2+self.u*self.m.cos(self.x[1])+(m1+m2)*g*self.m.sin(self.x[1])) /
                                (l*m1+l*m2*(1-self.m.cos(self.x[1])**2))))

        #self.m.Equation((self.x[1]*final - x_f[1])**2 - 0.1 <= 0) 

        self.m.Minimize(self.m.integral(self.u**2))
        #self.m.Minimize(1*(self.x[0]*final-x_f[0])**2*final)
        self.m.Minimize(1e3*(self.x[1]-x_f[1])**2*final)
        #self.m.Minimize(1*(self.x[2]*final-x_f[2])**2*final)
        #self.m.Minimize(1e5*(self.x[3]*final-x_f[3])**2*final)

        self.m.options.IMODE = 6
        self.m.options.SOLVER = 3
        self.current_action = 0
        self.fail_count = 0

    def swingup(self):
        try:
            self.x[0].value = self.state[0]
            self.x[1].value = np.pi - self.state[2]
            self.x[2].value = self.state[1]
            self.x[3].value = -self.state[3]
            self.m.solve(disp=False)
        except:
            print('MPC fail')
            self.fail_count += 1
            if self.fail_count > 5:
                raise RuntimeError('MPC failed to solve for trajectory.. falling back')
            return np.array([self.current_action])

        self.fail_count = 0
        update_line(np.array([self.x[0].value,self.x[1].value]))

        self.current_action = self.u.value[1]
        return np.array([self.current_action])


class CartPoleEnergyShapingController(CartPoleSwingUpController): 

    def __init__(self, gravity=9.81, 
                mass_cart=1, 
                mass_pole=1, 
                length_pole=0.5, 
                k_lqr=np.array([-4.47,80,-6,10.45]),
                k_e=2,
                k_x=[5,5]):

        super().__init__(gravity, 
                mass_cart, 
                mass_pole, 
                length_pole,
                k_lqr)
        self.k_e = k_e
        self.k_x = k_x

    def unpack_parameters(self):
        k_e = self.k_e
        k_x = self.k_x

        return super().unpack_parameters() + (k_e, k_x)

    def energy(self):
        """
        Total energy of pendulum
        Assumes the pendulum is a point mass attached by light rod
        """
        (gravity, mass_pole,
            mass_cart,
            length_pole,
            k_lqr,
            k_e,
            k_x) = self.unpack_parameters()

        theta = np.pi - self.state[2]
        theta_dot = -self.state[3]

        U = -mass_pole * gravity * length_pole * math.cos(theta)
        E = 0.5 * (mass_pole * (length_pole ** 2)) * theta_dot ** 2 + U

        return E

    def swingup(self):
        """
        Cart-pole energy shaping control
        """

        (gravity, mass_pole,
            mass_cart,
            length_pole,
            k_lqr, 
            k_e,
            k_x,) = self.unpack_parameters()

        Ed = mass_pole*gravity*length_pole
        Ediff = self.energy() - Ed

        x = self.state[0]
        x_dot = self.state[1]
        theta = np.pi - self.state[2]
        theta_dot = -self.state[3]

        c = math.cos(theta)
        s = math.sin(theta)
        t = math.tan(theta)

        acceleration = k_e * theta_dot * c * Ediff - k_x[0] * x - k_x[1]*x_dot

        f = ((mass_pole+mass_cart)*acceleration + 
                mass_pole*(-acceleration*c-gravity*s)*c - 
                mass_pole*length_pole*theta_dot**2*s)
        return f

