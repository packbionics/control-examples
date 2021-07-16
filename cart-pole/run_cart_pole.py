import gym
import numpy as np

def energy(env, state):
    g = env.gravity
    masscart = env.masscart
    masspole = env.masspole
    length = env.length * 2

    x = state[0]
    x_dot = state[1]
    theta = state[2]
    theta_dot = state[3]

    masses = [masscart, masspole]
    velocities = [x_dot, theta_dot]

    U = masspole*g*length*np.cos(theta)
    K = np.sum([0.5*m*v**2 for m,v in zip(masses,velocities)])

    return U, K

env = gym.make('gym_cart_pole:CartPoleSwingUpContinuous-v0')
env.reset()
for _ in range(10000):
    env.render()
    action = np.random.uniform(-1,1,(1,))
    state, reward, done, _ = env.step(action) # take a random action
    U, K = energy(env, state)

    if done:
        env.reset()
env.close()