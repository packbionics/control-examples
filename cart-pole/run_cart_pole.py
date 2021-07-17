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

def state_modifier(state):
    return (state[0], state[1], np.pi-state[2], -state[3])

def energy(env, state):
    g = env.gravity
    masscart = env.masscart
    masspole = env.masspole
    length = env.length * 2

    x = state[0]
    x_dot = state[1]
    theta = state[2]
    theta_dot = state[3]

    U = -masspole*g*length*np.cos(theta)
    E = 0.5*(masspole*length**2/3)*theta_dot**2 + U

    return E

def swingup(t, env, state, ke=0.05, kx=[0.2,1]):
    Ed = env.masspole*env.gravity*env.length*2
    E = energy(env, state)
    Ediff = E - Ed
    #update_line(np.array([t,Ediff]))
    c = np.cos(state[2])
    s = np.sin(state[2])
    t = np.tan(state[2])
    
    acceleration = ke*state[3]*c*Ediff - kx[0]*state[0] - kx[1]*state[1]
    f = (2-c**2)*acceleration - s*c - state[3]**2*s

    print('--control--')
    print('Ediff: {}'.format(Ediff))
    print('Theta_dot: {}'.format(state[3]*180/np.pi))
    print('Theta: {}'.format(state[2]*180/np.pi))
    print('Target acceleration: {}'.format(acceleration))
    print('Target force: {}'.format(f))
    return f

def upright(t,env,state,kth=[5,100], kx=[10,10]):
    c = np.cos(state[2])
    s = np.sin(state[2])
    t = np.tan(state[2])
    theta_diff = (state[2] % np.pi)
    acceleration = kth[0]*theta_diff - kth[1]*state[3] - kx[0]*state[0] - kx[1]*state[1]
    f = (c-2/c)*acceleration - 2*t - state[3]**2*s
    return f

env = gym.make('gym_cart_pole:CartPoleSwingUpContinuous-v0')
env.reset()
action = None
state = None
t = 0
switch = False
for _ in range(10000):
    if state is None:
        action = env.action_space.sample()
    else:
        if (abs(state[2]%(2*np.pi))<0.01) or switch:
            action = upright(t,env,state_modifier(state))
            switch = True
        else:
            action = swingup(t,env,state_modifier(state))
    for _ in range(1):
        env.render()
        state, reward, done, _ = env.step(action) 
        if done:
            env.reset()
            t = 0
            switch = False
            action = np.zeros((1,))
            break
        t += 1
env.close()