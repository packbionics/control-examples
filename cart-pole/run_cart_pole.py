import gym
import numpy as np
import matplotlib.pyplot as plt

plt.figure()
p, = plt.plot([],[], 'r-')

def update_line(new_data):
    p.set_xdata(np.append(p.get_xdata(), new_data[0]))
    p.set_ydata(np.append(p.get_ydata(), new_data[1]))
    plt.draw()
    plt.ylim((-20,5))
    plt.xlim((0,2000))
    plt.pause(0.000001)

def theta_distance(theta, target):
    return target - (theta%(2*np.pi))

"""
state_modifier changes theta origin to be on the botton
"""
def state_modifier(state):
    return (state[0], state[1], np.pi-state[2], -state[3])

def energy(env, state):
    g = env.gravity
    masscart = env.masscart
    masspole = env.masspole
    length = env.length

    x = state[0]
    x_dot = state[1]
    theta = state[2]
    theta_dot = state[3]

    U = -masspole*g*length*np.cos(theta)
    E = 0.5*(masspole*(length**2))*theta_dot**2 + U

    return E

def swingup(t, env, state, ke=0.2, kx=[0.9,0.4]):

    g = env.gravity
    masscart = env.masscart
    masspole = env.masspole
    length = env.length

    Ed = masspole*g*length
    E = energy(env, state)
    Ediff = E - Ed
    #update_line(np.array([t,Ediff]))
    c = np.cos(state[2])
    s = np.sin(state[2])
    t = np.tan(state[2])
    
    acceleration = ke*state[3]*c*Ediff - kx[0]*state[0] - kx[1]*state[1]

    f = (masspole+masscart)*acceleration + masspole*length*(-acceleration*c/length-g*s/length)*c - masspole*length*state[3]**2*s

    print('--control--')
    print('Ediff: {}'.format(Ediff))
    print('Theta_dot: {}'.format(state[3]*180/np.pi))
    print('Theta: {}'.format(state[2]*180/np.pi))
    print('Target acceleration: {}'.format(acceleration))
    print('Target force: {}'.format(f))

    return f

def upright(t,env,state,kth=[40,35], kx=[0.1,0.1]):
    c = np.cos(state[2])
    s = np.sin(state[2])
    t = np.tan(state[2])
    theta_diff = theta_distance(state[2],np.pi)
    print('--thetas--')
    print('theta: {}'.format(state[2]*180/np.pi))
    print('diff: {}'.format(theta_diff*180/np.pi))

    acceleration = kth[0]*theta_diff - kth[1]*state[3] - kx[0]*state[0] - kx[1]*state[1]
    f = (c-2/c)*acceleration - 2*t - state[3]**2*s 
    print('Target acceleration: {}'.format(acceleration))
    print('Target force: {}'.format(f))
    return f

env = gym.make('gym_cart_pole:CartPoleSwingUpContinuous-v0')
env.reset()
action = None
state = None
t = 0

for _ in range(100000):
    if state is None:
        action = env.action_space.sample()
    else:
        state = state_modifier(state)
        if (abs(theta_distance(state[2],np.pi)) < 0.1):
            action = upright(t,env,state)
        else:
            action = swingup(t,env,state)
    for _ in range(1):
        env.render()
        state, reward, done, _ = env.step(action) 
        if done:
            env.reset()
            t = 0
            action = np.zeros((1,))
            break
        t += 1
env.close()