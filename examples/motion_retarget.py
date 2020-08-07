import gym, yumi_gym
import pybullet as p
import h5py
import numpy as np

class h5Parser(object):
    def __init__(self, h5_file):
        self.data =  h5py.File(h5_file, "r") 

    def parse(self, group_name):
        # @return:x        np.ndarray with shape of 4 * T * 7
        # @return:Q        np.ndarray with shape of T x (7+12+7+12)
        ## Read needed data from mocap file
        l_elbow = self.data[group_name + '/l_elbow'][:]
        r_elbow = self.data[group_name + '/r_elbow'][:]

        l_wrist = self.data[group_name + '/l_wrist'][:] 
        r_wrist = self.data[group_name + '/r_wrist'][:]

        l_joint_angle = self.data[group_name + '/l_joint_angle'][:]
        r_joint_angle = self.data[group_name + '/r_joint_angle'][:]
        l_glove_angle = self.data[group_name + '/l_glove_angle'][:]
        r_glove_angle = self.data[group_name + '/r_glove_angle'][:]

        X = np.stack([l_elbow, r_elbow, l_wrist, r_wrist], axis=0)
        Q = np.concatenate([l_joint_angle, r_joint_angle, l_glove_angle, r_glove_angle], -1)
        return X, Q


parser = h5Parser("mocap_data_YuMi_affine_execute2.h5")
X, Q = parser.parse("fengren.bag")
max_episode = 200
weightVector = np.ones([12])
weightVector[:6] = 0.2     # elbow is less important

def custom_reward(**kwargs):
    # reward for position similarity
    joints = ['yumi_joint_4_l', 'yumi_joint_4_r', 'yumi_joint_7_l', 'yumi_joint_7_r']
    reference_pos = X[:, -1, :3]
    actual_pos = [kwargs['jointStates'][joint][4] for joint in joints]
    actual_pos = np.stack(actual_pos, axis=0)
    pos_error = actual_pos.flatten() - reference_pos.flatten()
    pos_reward = np.exp(-np.sum(pos_error*pos_error*weightVector))
    # reward for avoiding collision
    if kwargs['collision']:
        col_reward = -50
    else:
        col_reward = 0
    # reward for joint angle similarity
    reference_angle = Q[-1]
    joints = ["yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_3_l", "yumi_joint_4_l",
            "yumi_joint_5_l", "yumi_joint_6_l", "yumi_joint_7_l", "yumi_joint_1_r",
            "yumi_joint_2_r", "yumi_joint_3_r", "yumi_joint_4_r", "yumi_joint_5_r",
            "yumi_joint_6_r", "yumi_joint_7_r", "link1", "link11", "link2", "link22",
            "link3", "link33", "link4", "link44", "link5", "link51", "link52", "link53",
            "Link1", "Link11", "Link2", "Link22", "Link3", "Link33", "Link4", "Link44",
            "Link5", "Link51", "Link52", "Link53"]
    actual_angle = np.array([kwargs['jointStates'][joint][0] for joint in joints])
    angle_error = actual_angle - reference_angle
    angle_reward = np.exp(-np.sum(angle_error*angle_error))

    reward = pos_reward + col_reward + angle_reward

    print("current reward is {}".format(reward))

    if kwargs['step_counter'] >= max_episode or kwargs['collision']:
        done = True
    else:
        done = False

    return reward, done


env = gym.make('yumi-v0')
env.render()
observation = env.reset()

motorsIds = []
for joint in env.joints:
    motorsIds.append(p.addUserDebugParameter(joint, -1, 1, 0))

while True:
    env.render()

    # Only for test
    action = []
    for motorId in motorsIds:
        action.append(p.readUserDebugParameter(motorId))
    
    observation, reward, done, info = env.step(action, custom_reward)
