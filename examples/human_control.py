import gym, yumi_gym
import pybullet as p

env = gym.make('yumi-v0')
env.render()
observation = env.reset()

motorsIds = []
for joint in env.joints:
    motorsIds.append(p.addUserDebugParameter(joint, -1, 1, 0))

while True:
    env.render()

    action = []
    for motorId in motorsIds:
        action.append(p.readUserDebugParameter(motorId))
    
    observation, reward, done, info = env.step(action)
