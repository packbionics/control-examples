import gym
env = gym.make('gym_cart_pole:CartPoleSwingUpContinuous-v0')
env.reset()
for _ in range(1000):
    env.render()
    state, reward, done, _ = env.step(env.action_space.sample()) # take a random action
    if done:
        env.reset()
env.close()
