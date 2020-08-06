import gym, yumi_gym

env = gym.make('yumi-v0')
env.render()
observation = env.reset()

while True:
    env.render()
    
    action = env.action_space.sample()
    
    observation, reward, done, info = env.step(action)
