import os
import gym
from gym import spaces
from gym.utils import seeding
import pybullet as p
import pybullet_data
import numpy as np

class YumiEnv(gym.Env):
    """docstring for YumiEnv"""
    def __init__(self):
        super(YumiEnv, self).__init__()
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=[0,0,-0.1])
        self.step_counter = 0
        self.joints = ["yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_3_r", "yumi_joint_4_r",
                    "yumi_joint_5_r", "yumi_joint_6_r", "yumi_joint_7_r", "Link1", "Link11",
                    "Link2", "Link22", "Link3", "Link33", "Link4", "Link44", "Link5", "Link51",
                    "Link52", "Link53", "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_3_l",
                    "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l", "yumi_joint_7_l",
                    "link1", "link11", "link2", "link22", "link3", "link33", "link4", "link44",
                    "link5", "link51", "link52", "link53"]
        self.action_space = spaces.Box(np.array([-1]*len(self.joints)), np.array([1]*len(self.joints)))
        self.observation_space = spaces.Box(np.array([-1]*len(self.joints)), np.array([1]*len(self.joints)))

    def step(self, action, custom_reward=None):
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        p.setJointMotorControlArray(self.yumiUid, [self.joint2Index[joint] for joint in self.joints], p.POSITION_CONTROL, action)
        p.stepSimulation()
        # get joint states
        jointStates = {}
        for joint in self.joints:
            jointStates[joint] = p.getJointState(self.yumiUid, self.joint2Index[joint]) + p.getLinkState(self.yumiUid, self.joint2Index[joint])
        # check collision
        # collision = False
        # for joint in self.joints:
        #     if len(p.getContactPoints(bodyA=self.yumiUid, linkIndexA=self.joint2Index[joint])) > 0:
        #         collision = True
        #         print("Collision Occurred in Joint {}!!!".format(joint))
        #         p.changeVisualShape(self.yumiUid, self.joint2Index[joint], rgbaColor=[1,0,0,1])
        #     else:
        #         # recover color
        #         p.changeVisualShape(self.yumiUid, self.joint2Index[joint], rgbaColor=self.jointColor[joint])
        collision = False
        collisionJoints = set()
        for joint in self.joints:
            for point in p.getContactPoints(bodyA=self.yumiUid, linkIndexA=self.joint2Index[joint]):
                collisionJoints.add(p.getJointInfo(self.yumiUid, point[3])[1].decode('utf-8'))
                collisionJoints.add(p.getJointInfo(self.yumiUid, point[4])[1].decode('utf-8'))
        if len(collisionJoints):
            collision = True
            print("Collision Occurred in Joint {}".format(collisionJoints))
        
        self.step_counter += 1

        if custom_reward is None:
            # default reward
            reward = 0
            done = False
        else:
            # custom reward
            reward, done = custom_reward(jointStates=jointStates, collision=collision, step_counter=self.step_counter)

        info = {'collision': collision}
        observation = [jointStates[joint][0] for joint in self.joints]
        return observation, reward, done, info

    def reset(self):
        p.resetSimulation()
        self.step_counter = 0
        self.yumiUid = p.loadURDF(os.path.join(os.path.dirname(os.path.realpath(__file__)),
            "assets/yumi_with_hands.urdf"), useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
        self.tableUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(),
            "table/table.urdf"), basePosition=[0,0,-0.65])
        p.setGravity(0,0,-10)
        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setTimeStep(1./240.)
        self.joint2Index = {} # jointIndex map to jointName
        for i in range(p.getNumJoints(self.yumiUid)):
            self.joint2Index[p.getJointInfo(self.yumiUid, i)[1].decode('utf-8')] = i
        self.jointColor = {} # jointName map to jointColor
        for data in p.getVisualShapeData(self.yumiUid):
            self.jointColor[p.getJointInfo(self.yumiUid, data[1])[1].decode('utf-8')] = data[7]

    def render(self, mode='human'):
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.5,0,0.5],
                                                          distance=.7,
                                                          yaw=90,
                                                          pitch=0,
                                                          roll=0,
                                                          upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                   aspect=float(960)/720,
                                                   nearVal=0.1,
                                                   farVal=100.0)
        (_, _, px, _, _) = p.getCameraImage(width=960,
                                            height=720,
                                            viewMatrix=view_matrix,
                                            projectionMatrix=proj_matrix,
                                            renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720,960,4))

        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def close(self):
        p.disconnect()