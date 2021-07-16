from gym.envs.registration import register


register(
    id='CartPoleSwingUpContinuous-v0',
    entry_point='gym_cart_pole.envs:CartPoleSwingUpEnv',
    max_episode_steps=1000,
)

register(
    id='CartPoleSwingUpDiscrete-v0',
    entry_point='gym_cart_pole.envs:CartPoleSwingUpDiscreteEnv',
    max_episode_steps=1000,
)
