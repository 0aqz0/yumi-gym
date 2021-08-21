import gym, yumi_gym
import pybullet as p
import numpy as np
import h5py
import time


hf = h5py.File('/home/ddq/inference.h5', 'r')
group1 = hf.get('group1')
l_joint_angle = group1.get('l_joint_angle')
r_joint_angle = group1.get('r_joint_angle')
l_hand_angle = group1.get('l_glove_angle')
r_hand_angle = group1.get('r_glove_angle')

# hf = h5py.File('/home/ddq/projects/MotionTransfer/optimization_result_train.h5', 'r')
# key = 'huozhe'
# arm_traj = hf[key+'/arm_traj_1'][:]
# l_joint_angle = arm_traj[:, :7]
# r_joint_angle = arm_traj[:, 7:14]
# l_glove_angle = arm_traj[:, 14:26]
# r_glove_angle = arm_traj[:, 26:38]

total_frames = l_joint_angle.shape[0]
print(l_joint_angle.shape, r_joint_angle.shape)
env = gym.make('yumi-v0')
env.render()
observation = env.reset()

while True:
    env.render()
    for t in range(total_frames):
        for i in range(5):
            print(t, l_joint_angle.shape, l_joint_angle[t] * 180 / np.pi)
            action = l_joint_angle[t].tolist() + r_joint_angle[t].tolist() + l_hand_angle[t].tolist() + r_hand_angle[t].tolist()
            observation, reward, done, info = env.step(action)
            time.sleep(0.02)
