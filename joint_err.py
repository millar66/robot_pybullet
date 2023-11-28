#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 27 19:19:36 2023

@author: lihui.liu
"""
import os
import xml.dom.minidom
import shutil
import numpy as np
import sys
sys.path.append('/home/lihui.liu/mnt/workspace/python/robot/robot_pybullet')
import pybullet as p
import pybullet_data
from time import sleep, time
import timeit
from m_class import SetSimulation, Thread_print, Robot_info, CameraOperate, ParameterInit, DHParameter, robot_control
# import m_class
from queue import Queue
from threading import Event
import math
from pprint import pprint
from pyquaternion import Quaternion
from pybullet_utils import bullet_client as bc
from numpy.linalg import inv
from numpy.linalg import pinv
import random
import copy
import matplotlib  
import matplotlib.pyplot as plt  

ParameterInit.pos_lim()

use_gui = True
if use_gui:
    physicsClientId = p.connect(p.GUI)
else:
    physicsClientId = p.connect(p.DIRECT)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0) # 0:不让CPU上的集成显卡参与渲染工作
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) # 0:不显示GUI上的控件
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1) # 1:打开渲染

p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

p.resetDebugVisualizerCamera(cameraTargetPosition=[0.05,0.02,0.39],\
                             cameraDistance=1.20,\
                             cameraPitch=-30.40,\
                             cameraYaw=24.40) #转变视角

p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
robot_id = p.loadURDF("./ddd/000PSM_10.SLDASM/urdf/modified.urdf",
                      basePosition=[0, 0, 0], useMaximalCoordinates=False, useFixedBase=True)

numJoints = 11
targetPosition_init = [0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.1, 0.5, 0.5, 0.5, 0.5]
point_joint_base1 = DHParameter().DH_compute(targetPosition_init)

joint_pos_err = np.zeros((numJoints, 3))
joint_orn_err = np.zeros((numJoints, 3))
for i in range(11):
    joint_pos_err[i] = [0.01, 0.01, 0.01]
    if i != 0:
        joint_pos_err[i-1] = [0.0, 0.0, 0.0]
    point_joint_base2 = DHParameter().DH_compute(targetPosition_init, joint_pos_err, joint_orn_err)
    print(point_joint_base1,'\n',point_joint_base2,'\n','************ \n','joint',i,'= ',point_joint_base1-point_joint_base2)
# %%
targetPosition_init = [0.3, 0.3, 0, -1.3, 0, 1.0, 0, 0, 0, 0, 0]
robot_control.setJointPosition(robot_id, targetPosition_init, 11)
# p.addUserDebugPoints(pointPositions=[aaaaa[0:3]], pointColorsRGB=[[1,0,1]], pointSize=10)
# sleep(3.)
p.removeAllUserDebugItems()
    
p.disconnect(physicsClientId)

# %%
# numJoints = 11

# targetPosition_init = [0, 0, 0, 0, 0, 0, 0, 0, 0.5, -0.5, 0]
# targetPosition_init = [0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.1, 0.5, 0.5, 0.5, 0.5]
# # targetPosition_init = [1.57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=targetPosition_init)
# sleep(1.)
# joint_positions, joint_velocities, joint_torques = robot_control.getJointStates(robot_id)
# point_joint_base = DHParameter().DH_compute(targetPosition_init, [0,0,0])

# for i in range(240 * 10):
#     targetPosition_init[0] = 1 * np.sin(2 * np.pi * 0.5 * i / 240)
#     if targetPosition_init[0] > 1 :
#         print('******')
#     p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=targetPosition_init)
#     sleep(1./24.)
#     joint_positions, joint_velocities, joint_torques = robot_control.getJointStates(robot_id)
#     point_joint_base = DHParameter().DH_compute(targetPosition_init, [0,0,0])