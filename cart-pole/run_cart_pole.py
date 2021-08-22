from cart_pole_control import *

import gym
import numpy as np
import matplotlib.pyplot as plt

plt.figure()
p, = plt.plot([],[], 'r-')

env = gym.make('gym_cart_pole:CartPoleSwingUpContinuous-v0')
env.reset() 
action = None
state = None
t = 0

# LQR gains
K = np.array([-4.47,80,-6,10.45])

"""
Simulation loop
Choose action based on state,
Completes action and returns new state
"""
for _ in range(100000):
    if state is None:
        action = env.action_space.sample()
    else:
        state = state_modifier(state)
        if (abs(theta_distance(state[2],np.pi)) < .4):
            action = upright_lqr(K,state)
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