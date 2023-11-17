#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 09:32:02 2023

@author: lihui.liu
"""

import numpy as np
import sys
sys.path.append('/home/lihui.liu/mnt/workspace/python/robot/robot_pybullet')
import pybullet as p
import pybullet_data
from time import sleep
from m_class import SetSimulation, Thread_print, Robot_info, CameraOperate
# import m_class
from queue import Queue
from threading import Event
import math
from pprint import pprint

event = Event()
print_text_queue = Queue(50)
print_content_queue = Queue(50)
thread = Thread_print(event, run_time=10)
thread.start()
sleep(2)
event.set()
sleep(0.1)
event.clear()
thread.join()
print("init end")
sleep(0.1)

use_gui = True
if use_gui:
    physicsClientId = p.connect(p.GUI)
else:
    physicsClientId = p.connect(p.DIRECT)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0) # 0:不让CPU上的集成显卡参与渲染工作
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1) # 0:不显示GUI上的控件
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1) # 1:打开渲染

p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

p.resetDebugVisualizerCamera(cameraTargetPosition=[0.05,0.02,0.39],\
                             cameraDistance=1.20,\
                             cameraPitch=-30.40,\
                             cameraYaw=24.40) #转变视角

p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
robot_id = p.loadURDF("./uuu/000PSM_10.SLDASM/urdf/000PSM_10.SLDASM.urdf",
                      basePosition=[0, 0, 0], useMaximalCoordinates=False)

# 重置base连杆质心的位置和姿态。
# startPos = [0, 0, 1]
# startOrientation = p.getQuaternionFromEuler([0, 0, 0])
# p.resetBasePositionAndOrientation(robot_id, startPos, startOrientation)  # 没有返回参数

set_robot = SetSimulation(robot_id)
set_robot.Set_init_my_robot()

p.changeDynamics(bodyUniqueId=robot_id, linkIndex=3, restitution=0.5, contactStiffness=10**8, contactDamping=10**5)
# for i in range(numJoints):
#     p.changeDynamics(robot_id, joint_id[i], linearDamping=0, angularDamping=0)

# cube_id = p.loadURDF("cube_small.urdf", 0, 0, 0)

plane_robot = p.createConstraint(plane_id, -1, robot_id, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
# 创建固定约束：将机器人躯干和地面固定
fixed_id = p.createConstraint(parentBodyUniqueId=robot_id, parentLinkIndex=-1, childBodyUniqueId=-1,
                              childLinkIndex=-1, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0],
                              parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0],
                              childFrameOrientation=[0, 0, 0, 1])
p.changeConstraint(plane_robot, maxForce=1000000)  # 更改约束力大小
# 创建固定约束：可变
# robot_constraint = p.createConstraint(cube_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])
# p.changeConstraint(robot_constraint, pivot, jointChildFrameOrientation=orn, maxForce=50) # 更改 约束轴位置和方向、约束力大小
# 创建滑动约束
# one_id = p.createConstraint(robot_id, -1, -1, -1, p.JOINT_PRISMATIC, [0, 0, 1], [0, 0, 0], [0, 0, 1])  # 只能在z轴滑动
# p.changeConstraint(one_id, maxForce=10000000)

# p.removeConstraint(one_id) # 移除约束
# %%
set_robot = SetSimulation(robot_id)
set_robot.Set_init_my_robot()

numJoints = p.getNumJoints(robot_id, physicsClientId)
print("连杆信息: ")
for link_index in range(-1, numJoints, 1):
    info_link = Robot_info(robot_id, link_index)
    info_link.link_info()
    
print("关节信息: ")
for joint_index in range(numJoints):
    info_joint = Robot_info(robot_id, joint_index)
    info_joint.joint_info()

print("连杆信息: ")
info_link = Robot_info(robot_id, 2)
info_link.link_info()
print("关节信息: ")
info_joint = Robot_info(robot_id, 0)
info_joint.joint_info()
print("活动关节: ")
active_joint = Robot_info(robot_id)
active_joint.active_joint_info()
print("关节状态: ")
joint_state = Robot_info(robot_id, 0)
joint_state.joint_state_info()

print("连杆状态: ")
link_state = Robot_info(robot_id, 3)
link_state.link_state_info()

# %%
# 返回从URDF、SDF、MJCF或其他文件中提取的基座(base name)名称和机器人名称(body name)。
base_name, robot_name = p.getBodyInfo(robot_id)
print(base_name)  # b'floor'
print(robot_name)  # b'floor_obj'
base_name = base_name.decode("utf8")  # floor
robot_name = robot_name.decode("utf8")  # floor_obj

# 获取机器人base连杆质心在世界坐标系中的位置和姿态四元数，
# 返回3个浮点数表示的位置列表和4个浮点数表示的姿态列表[x、y、z、w]。单位为米和弧度。
Pos, Orn = p.getBasePositionAndOrientation(robot_id) # 返回两个列表，第一个：base连杆的位置，第二个：base的姿态四元数
print(f"机器人的base位置坐标为:{Pos}\n机器人的base姿态四元数为:{Orn}")

# %%
a = CameraOperate(robot_id)
width, height, rgbImg, depthImg, segImg = a.setCameraPicAndGetPic()









# img = p.getCameraImage(224, 224, renderer=p.ER_BULLET_HARDWARE_OPENGL)





        
# viewMatrix = p.computeViewMatrixFromYawPitchRoll(
#     cameraTargetPosition=[0.05,0.02,0.39],
#     distance=1.2,
#     yaw=-30,
#     pitch=24,
#     roll=0,
#     upAxisIndex=2)
        
# projectionMatrix = p.computeProjectionMatrixFOV(
#     fov=50.0,
#     aspect=1.0,
#     nearVal=0.01,
#     farVal=20,
#     physicsClientId=physicsClientId
# )

# width, height, rgbImg, depthImg, segImg = p.getCameraImage(
#     224, 224,
#     viewMatrix=viewMatrix,
#     projectionMatrix=projectionMatrix,
#     renderer=p.ER_BULLET_HARDWARE_OPENGL,
#     physicsClientId=physicsClientId
# )







# %%
p.disconnect(physicsClientId)


# %%
cylinder_real_robot_start_pos = [0, 0, 0]
cylinder_real_robot_start_orientation = p.getQuaternionFromEuler([0,0,0])










