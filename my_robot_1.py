#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 19:43:50 2023

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
from time import sleep
from m_class import SetSimulation, Thread_print, Robot_info, CameraOperate
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
robot_id = p.loadURDF("./aaa/000PSM_10.SLDASM/urdf/000PSM_10.SLDASM.urdf",
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


a = CameraOperate(robot_id)
width, height, rgbImg, depthImg, segImg = a.setCameraPicAndGetPic()
# img = p.getCameraImage(224, 224, renderer=p.ER_BULLET_HARDWARE_OPENGL)

def setJointPosition(robot, position, ctl_num):
    num_joints = p.getNumJoints(robot)
    p.setJointMotorControlArray(robot,
                                range(ctl_num),
                                p.POSITION_CONTROL,
                                targetPositions=position)
    # if len(position) == num_joints: 
    #     p.setJointMotorControlArray(robot,
    #                                 range(num_joints),
    #                                 p.POSITION_CONTROL,
    #                                 targetPositions=position)
    # else:
    #     print("num_joints is not right")
def getJointStates(robot):
    joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
    joint_positions = [state[0] for state in joint_states]
    joint_velocities = [state[1] for state in joint_states]
    joint_torques = [state[3] for state in joint_states]
    return joint_positions, joint_velocities, joint_torques

def getMotorJointStates(robot):
    joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
    joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
    
    # 可以使用的关节
    available_joints_indexes = [i for i in range(p.getNumJoints(robot)) if joint_infos[2] != p.JOINT_FIXED]
    # print("joint name: ",[joint_infos[i][1] for i in available_joints_indexes])
    # print("can use joint: ",[joint_infos[i][2] for i in available_joints_indexes])

    # info[3] : JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED
    # a = [i[3] for i in joint_infos]
    joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
    # joint_states = [j for j, i in zip(joint_states, joint_infos)]
    joint_positions = [state[0] for state in joint_states]
    joint_velocities = [state[1] for state in joint_states]
    joint_torques = [state[3] for state in joint_states]
    return joint_positions, joint_velocities, joint_torques

# p.resetSimulation()
# robot_id = SetSimulation()
# numJoints = p.getNumJoints(robot_id[0])
set_robot = SetSimulation(robot_id)
set_robot.Set_init_my_robot()
print("numJoints:",numJoints)

RobotEndEffectorIndex = numJoints - 1
print("RobotEndEffectorIndex:",RobotEndEffectorIndex)

robotEndOrientation = p.getQuaternionFromEuler([0,0,-1.57])
# targetPositionsJoints=[0,-1.57/2,1.57/2,0,-1.57,0,0,0,0,0,0,0,0,1.57]
targetPositionsJoints=[0,-1.57/2,1.57/2,-0.5,-1.57,0,0,0,0,0,0]
setJointPosition(robot_id, targetPositionsJoints, 11)
# for i in range(240):
#     p.stepSimulation()
#     sleep(1./240.)

# 刚度系数为50000，阻尼系数为 0.57，摩擦系数为 0.5，恢复系数为 0.6
# p.changeDynamics(bodyUniqueId=robot_id, linkIndex=2, lateralFriction=0.7, restitution=0.6, 
#                  contactStiffness=50000, contactDamping=0.57)
# p.changeDynamics(bodyUniqueId=robot_id, linkIndex=4, lateralFriction=0.7, restitution=0.6, 
#                  contactStiffness=50000, contactDamping=0.57)
# p.changeDynamics(bodyUniqueId=robot_id, linkIndex=3, lateralFriction=0.7, restitution=0.6, 
#                  contactStiffness=50000, contactDamping=0.57, angularDamping=0.5)
# p.changeDynamics(bodyUniqueId=robot_id, linkIndex=1, lateralFriction=0.7, restitution=0.6, 
#                  contactStiffness=50000, contactDamping=0.57, angularDamping=0.5)

# p.setJointMotorControl2(robot_id,1,p.POSITION_CONTROL,targetPosition=0.2,force=80)
# sleep(1./24.)
# p.setJointMotorControl2(robot_id,1,p.POSITION_CONTROL,targetPosition=0,force=80)

# for i in range(240*3):
#     targetPosition = 3 * math.sin(2 * math.pi * 0.4 * i / 240)
#     p.setJointMotorControl2(robot_id,1,p.POSITION_CONTROL,targetPosition=targetPosition,force=80)
#     sleep(1./240.)

set_robot = SetSimulation(robot_id)
set_robot.Set_init_my_robot()

# %%
p.resetDebugVisualizerCamera(cameraTargetPosition=[0.05,0.02,0.39],\
                             cameraDistance=1.20,\
                             cameraPitch=-52.80,\
                             cameraYaw=-34.80) #转变视角

# for i in range(numJoints):
#     p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=1, force=200)
#     sleep(1./24.)

# p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, targetPosition=0, force=200)
# sleep(1./24.)
# p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, targetPosition=0.2, force=200)
# sleep(1./24.)
# p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=3, force=200)
# sleep(1./24.)
# p.setJointMotorControl2(robot_id, 3, p.POSITION_CONTROL, targetPosition=5.5, force=200)
# sleep(1./24.)
# p.setJointMotorControl2(robot_id, 4, p.POSITION_CONTROL, targetPosition=1, force=200)
# sleep(1./24.)
# p.setJointMotorControl2(robot_id, 5, p.POSITION_CONTROL, targetPosition=1, force=200)
# sleep(1./24.)

# p.setJointMotorControl2(robot_id, 6, p.POSITION_CONTROL, targetPosition=0, force=200)
# sleep(1./24.)

targetPosition_init = [3, 0.3, 3, 5.5, 1.6, 0.5, 0, 3, 3, 3, 3]
setJointPosition(robot_id, targetPosition_init, 11)

RobotEndEffectorIndex = 6
result = p.getLinkState(robot_id,
                        RobotEndEffectorIndex,
                        computeLinkVelocity=1,
                        computeForwardKinematics=1)
org_link_trn, org_link_rot, org_com_trn, org_com_rot, org_frame_pos, org_frame_rot, org_link_vt, org_link_vr = result
p.addUserDebugText(text="linktrn", textPosition=org_link_trn, textColorRGB=[0, 1, 0], textSize=1.2)

org_link_rot_new = [org_link_rot[3], org_link_rot[0], org_link_rot[1], org_link_rot[2]]

single_step_duration = 0.03 #seconds
last_link_trn = org_link_trn
targetPositionsJoints = [1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1]
delat_x = org_link_trn[0]
delat_y = org_link_trn[1]
delat_z = org_link_trn[2]
j = 0
i = 0
print(org_link_trn)

for j in range(15):
    step_flag = 0
    org_link_trn_new = last_link_trn
    if j % 2 == 0:
        move_step = 0.0005
    else:
        move_step = -0.0005
    for i in range(2*240):
        step_flag = step_flag + 1
        pos, vel, torq = getJointStates(robot_id)
        mpos, mvel, mtorq = getMotorJointStates(robot_id)
    
        #获取机器人手臂运动过程中的节点位置
        result = p.getLinkState(robot_id,
                                RobotEndEffectorIndex,
                                computeLinkVelocity=1,
                                computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
        
        #画出机器人手臂末端运动轨迹
        debug_line_id = p.addUserDebugLine(last_link_trn,link_trn,lineColorRGB=[0,0,1],lineWidth=2)
        last_link_trn = link_trn
    
    	#机器人手臂末端移动路线的修正
        # delat_x = org_link_trn_new[0] + (step_flag * move_step) - link_trn[0]
        # delat_y = org_link_trn_new[1] + step_flag * 0 - link_trn[1]
        # delat_z = org_link_trn_new[2] + step_flag * 0 - link_trn[2]
        delat_x = org_link_trn_new[0] + (step_flag * move_step)
        # delat_y = org_link_trn_new[1] + step_flag * 0
        # delat_z = org_link_trn_new[2] + step_flag * 0
        movement_vector = [delat_x,delat_y,delat_z]
        # print("movement_vector:",movement_vector)
        # print("org_link_trn_new:",org_link_trn_new)
    
        zero_vec = [0.0] * len(mpos)
        zero_acc = [0.0] * len(mpos)

        #print("movement_vector:",movement_vector)
    	
    	#计算雅可比矩阵
        # jac_t, jac_r = p.calculateJacobian(robot_id, RobotEndEffectorIndex, com_trn, mpos, zero_vec, zero_acc)
        
        joint_ik = p.calculateInverseKinematics(robot_id, RobotEndEffectorIndex, movement_vector)
    
    	#我首先没有考虑保持机器人手臂末端方向一致
    	#所以暂时只用到“jac_t”
        # jac_t_pi = pinv(jac_t)
    
        #计算机器人手臂需要调整的角度
        # expected_delta_q_dot_1 = list(np.dot(jac_t_pi, movement_vector))
        # targetPositionsJoints = list(np.sum([expected_delta_q_dot_1[0:11], targetPositionsJoints], axis = 0))
        # for i in range(len(targetPositionsJoints)):
        #     if targetPositionsJoints[i] < 0 :
        #         print('err******', i, targetPositionsJoints[i])
        for i in range(len(joint_ik)):
            if joint_ik[i] < 0 :
                print('err******', i, joint_ik[i])
            
    	#机器人手臂的移动
        # setJointPosition(robot_id, targetPositionsJoints[0:11])
        setJointPosition(robot_id, joint_ik, 11)
        # location, _ = p.getBasePositionAndOrientation(robot_id[0])
        # p.resetDebugVisualizerCamera(
        #     cameraDistance=3,
        #     cameraYaw=110,
        #     cameraPitch=-30,
        #     cameraTargetPosition=location
        # )
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)  #平滑视觉渲染
        p.stepSimulation()
        sleep(1./240.)

# %%
joint_mass_info = Robot_info(robot_id)
# b = a.getInertial(robot_id, xmlDoc)

shutil.copyfile("/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/aaa/000PSM_10.SLDASM/urdf/000PSM_10.SLDASM.urdf",
                "/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/aaa/000PSM_10.SLDASM/urdf/test.urdf")

if os.path.exists("/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/aaa/000PSM_10.SLDASM/urdf/test.urdf"):
    os.replace("/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/aaa/000PSM_10.SLDASM/urdf/test.urdf",
               "/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/aaa/000PSM_10.SLDASM/urdf/test.xml")

xmlDoc = xml.dom.minidom.parse("/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/aaa/000PSM_10.SLDASM/urdf/test.xml")
link_count = xmlDoc.getElementsByTagName("link").length
joint_count = xmlDoc.getElementsByTagName("joint").length
print(joint_count)
data = joint_mass_info.getInertial(robot_id, xmlDoc)
print(data[0])
print(data[7])
print(data[8])
print(data[9])









# %%
p.removeUserDebugItem(debug_line_id)
p.disconnect(physicsClientId)








