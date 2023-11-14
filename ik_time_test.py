#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 10 14:47:53 2023

@author: lihui.liu
"""

# _*_coding:utf-8-*-
import sys
import gym
from gym import spaces
import pybullet as p
import pybullet_data
import math
import numpy as np
import argparse
# from env_module import random_pos, random_position, get_position_r, \
#     get_position_p, quaternion_revised, admittance

gym.logger.set_level(40)

# %%
class UREnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, gui=False):
        if gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        p.resetDebugVisualizerCamera(cameraDistance=0.3, cameraYaw=60, cameraPitch=-50,
                                     cameraTargetPosition=[0.5, -0.05, 0.1])
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)

    def step(self):
        # p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)  #平滑视觉渲染
        self.posture = np.array(p.getLinkState(self.pandauid, 6)[5],dtype=float).reshape(4,1)
        self.position = np.array(p.getLinkState(self.pandauid, 6)[4],dtype=float).reshape(3,1)
        print(self.position[0])
        self.position[0] = float(self.position[0])+0.001
        # print(self.position[0])
        jointposes = p.calculateInverseKinematics(self.pandauid, 6, self.position, self.posture, maxNumIterations=100)
        p.setJointMotorControlArray(self.pandauid, list([1, 2, 3, 4, 5, 6]), p.POSITION_CONTROL, list(jointposes))
        n=100
        while(n):
            p.stepSimulation()
            n = n-1
        self.position = np.array(p.getLinkState(self.pandauid, 6)[4],dtype=float).reshape(3,1)
        print(self.position[0])
        # print("加了平滑")
        # print("没加平滑")
        # print("2次迭代")
        # print("10次迭代")
        # print("100次迭代")
        # print("1次仿真")
        # print("10次仿真")
        print("100次仿真")
    def reset(self):
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.loadURDF("plane.urdf", basePosition=[0, 0, -0.65])
        self.pandauid = p.loadURDF("ur5_10cm_10mm.urdf", useFixedBase=True)
        p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.65])
        orientation = p.getQuaternionFromEuler([math.pi / 2., -math.pi, 0])
        p.loadURDF("12mm/urdf/hole.urdf", basePosition=[0.355, 0, 0], baseOrientation=orientation,
                               useFixedBase=True)
        rest_poses = [0.019493980630576238, -1.6104386316325356, 2.371476945587111, -2.3317793248031182,
                      -1.57079440431533, 0]
        for j in range(6):
            p.resetJointState(self.pandauid, j + 1, rest_poses[j])
        p.stepSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    def close(self):
        p.disconnect()
if __name__ == '__main__':
    U = UREnv()
    U.reset()
    m = 1
    while(m):
        m = m-1
        U.step()
    U.close()
    