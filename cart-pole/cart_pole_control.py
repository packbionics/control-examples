import gym
import numpy as np
import matplotlib.pyplot as plt

def update_line(new_data):
    """
    Plot real-time data 
    """
    p.set_xdata(np.append(p.get_xdata(), new_data[0]))
    p.set_ydata(np.append(p.get_ydata(), new_data[1]))
    plt.draw()
    plt.ylim((-20,5))
    plt.xlim((0,2000))
    plt.pause(0.000001)

def theta_distance(theta, target):
    """
    Computes X-X*, where X* is the target angle
    Wraps angle every 2pi to get measured difference

    Keyword arguments:
    theta   current angle (-inf,inf)
    target  target angle
    """
    return (theta%(2*np.pi)) - target


def state_modifier(state):
    """
    Changes theta origin to be on the bottom
        to accomodate different derivations

    Keyword arguments:
    state   State vector of system
    """
    return (state[0], state[1], np.pi-state[2], -state[3])

def energy(env, state):
    """
    Total energy of pendulum
    Assumes the pendulum is a point mass attached by light rod

    Keyword arguments:
    env     Environment reference
    state   State vector of system
    """
    g = env.gravity
    masspole = env.masspole
    length = env.length

    x = state[0]
    theta = state[2]
    theta_dot = state[3]

    U = -masspole*g*length*np.cos(theta)
    E = 0.5*(masspole*(length**2))*theta_dot**2 + U

    return E

def swingup(time, env, state, ke=0.5, kx=[5,5], plot=False):
    """
    Cart-pole energy shaping control

    Keyword arguments:
    time    time of system
    env     Environment reference
    state   State vector of system
    ke      Energy gain
    kx      Position PD gain
    plot    Plot energy of pendulum
    """

    g = env.gravity
    masscart = env.masscart
    masspole = env.masspole
    length = env.length

    Ed = masspole*g*length
    E = energy(env, state)
    Ediff = E - Ed
    if plot:
        update_line(np.array([time,Ediff]))
    c = np.cos(state[2])
    s = np.sin(state[2])
    t = np.tan(state[2])
    
    acceleration = ke*state[3]*c*Ediff - kx[0]*state[0] - kx[1]*state[1]

    f = ((masspole+masscart)*acceleration + 
            masspole*(-acceleration*c-g*s)*c - 
            masspole*length*state[3]**2*s)
    return f

def upright_lqr(K,state):
    """
    LQR controller

    Keyword arguments:
    K       LQR controller gains
    state   State vector of system
    """
    
    theta_diff = theta_distance(state[2],np.pi)
    X = np.array([state[0], theta_diff, state[1], state[3]])
    f = np.dot(K,X)

    return -f

def upright(state,kth=[50,20], kx=[0.01,0.01]):
    """
    Non-collocated control of pole in upright position

    Keyword arguments:
    state   State vector of system
    kth     Angle PD gains
    kx      Position PD gains
    """
    c = np.cos(state[2])
    s = np.sin(state[2])
    t = np.tan(state[2])
    theta_diff = theta_distance(state[2],np.pi)

    acceleration = -kth[0]*theta_diff - kth[1]*state[3] - kx[0]*state[0] - kx[1]*state[1]
    f = (c-2/c)*acceleration - 2*t - state[3]**2*s 

    return f

