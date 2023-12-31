#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  9 13:31:36 2023

@author: lihui.liu
"""

# %%
from pyquaternion import Quaternion
import pybullet as p
import time
import pybullet_data
from pybullet_utils import bullet_client as bc
import numpy as np
from numpy.linalg import inv
from numpy.linalg import pinv
import math
import random
import copy
import matplotlib  
import matplotlib.pyplot as plt  

# %%
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

# %%
def SetSimulation():
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-9.81)#设置重力

    p.resetDebugVisualizerCamera(cameraTargetPosition=[0.11,0.5,0.25],\
                             cameraDistance=7,\
                             cameraPitch=-35.8,\
                             cameraYaw=-118.05) #转变视角

    #load and set real robot
    cylinder_real_robot_start_pos = [0, 0, 0]
    cylinder_real_robot_start_orientation = p.getQuaternionFromEuler([0,0,0])
    # loadSDF(fileName, useMaximalCoordinates, globalScaling, physicsClientId)
    # loadMJCF(fileName, useMaximalCoordinates, globalScaling, physicsClientId)
    kuka_robot_id = p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")
    p.resetBasePositionAndOrientation(kuka_robot_id[0],cylinder_real_robot_start_pos,cylinder_real_robot_start_orientation)
    plane_id = p.loadURDF("plane.urdf")
    return kuka_robot_id

def setJointPosition(robot, position):
    num_joints = p.getNumJoints(robot)
    if len(position) == num_joints: 
        p.setJointMotorControlArray(robot,
                                    range(num_joints),
                                    p.POSITION_CONTROL,
                                    targetPositions=position)
    else:
        print("num_joints is not right")
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

# %%
p.resetSimulation()
kuka_robot_id = SetSimulation()
numJoints = p.getNumJoints(kuka_robot_id[0])
print("numJoints:",numJoints)

KukaEndEffectorIndex = numJoints - 1
print("KukaEndEffectorIndex:",KukaEndEffectorIndex)

robotEndOrientation = p.getQuaternionFromEuler([0,0,-1.57])
# targetPositionsJoints=[0,-1.57/2,1.57/2,0,-1.57,0,0,0,0,0,0,0,0,1.57]
targetPositionsJoints=[0,-1.57/2,1.57/2,-0.5,-1.57,0,0,0,0,0,0,0,0,0]
setJointPosition(kuka_robot_id[0], targetPositionsJoints)
for i in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

result = p.getLinkState(kuka_robot_id[0],
                        KukaEndEffectorIndex,
                        computeLinkVelocity=1,
                        computeForwardKinematics=1)
org_link_trn, org_link_rot, org_com_trn, org_com_rot, org_frame_pos, org_frame_rot, org_link_vt, org_link_vr = result
print("link_trn:",org_link_trn)
print("link_rot:",org_link_rot)
print("com_trn:",org_com_trn)
print("com_rot:",org_com_rot)
print("frame_pos:",org_frame_pos)
print("frame_rot:",org_frame_rot)
print("link_vt:",org_link_vt)
print("link_vr:",org_link_vr)
p.addUserDebugText(text="linktrn", textPosition=org_link_trn, textColorRGB=[0, 1, 0], textSize=1.2)

org_link_rot_new = [org_link_rot[3], org_link_rot[0], org_link_rot[1], org_link_rot[2]]

single_step_duration = 0.03 #seconds
last_link_trn = org_link_trn

# %%
for j in range(3):
    step_flag = 0
    org_link_trn_new = last_link_trn
    if j % 2 == 0:
        move_step = 0.0005
    else:
        move_step = -0.0005
    for i in range(2*960):
        step_flag = step_flag + 1
        pos, vel, torq = getJointStates(kuka_robot_id[0])
        mpos, mvel, mtorq = getMotorJointStates(kuka_robot_id[0])
    
        #获取机器人手臂运动过程中的节点位置
        result = p.getLinkState(kuka_robot_id[0],
                                KukaEndEffectorIndex,
                                computeLinkVelocity=1,
                                computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
        
        #画出机器人手臂末端运动轨迹
        p.addUserDebugLine(last_link_trn,link_trn,lineColorRGB=[0,0,1],lineWidth=2)
        last_link_trn = link_trn
    
    	#机器人手臂末端移动路线的修正
        delat_x = org_link_trn_new[0] + (step_flag * move_step) - link_trn[0]
        delat_y = org_link_trn_new[1] + step_flag * 0 - link_trn[1]
        delat_z = org_link_trn_new[2] + step_flag * 0 - link_trn[2]
        movement_vector = [delat_x,delat_y,delat_z]
        #print("movement_vector:",movement_vector)
    
        zero_vec = [0.0] * len(mpos)
        zero_acc = [0.0] * len(mpos)
        #print("movement_vector:",movement_vector)
    	
    	#计算雅可比矩阵
        jac_t, jac_r = p.calculateJacobian(kuka_robot_id[0], KukaEndEffectorIndex, com_trn, mpos, zero_vec, zero_acc)
    
    	#我首先没有考虑保持机器人手臂末端方向一致
    	#所以暂时只用到“jac_t”
        jac_t_pi = pinv(jac_t)
    
        #计算机器人手臂需要调整的角度
        expected_delta_q_dot_1 = list(np.dot(jac_t_pi, movement_vector)) + [0] * 3
        targetPositionsJoints = list(np.sum([expected_delta_q_dot_1, targetPositionsJoints], axis = 0))
    
    	#机器人手臂的移动
        setJointPosition(kuka_robot_id[0], targetPositionsJoints)
        # location, _ = p.getBasePositionAndOrientation(kuka_robot_id[0])
        # p.resetDebugVisualizerCamera(
        #     cameraDistance=3,
        #     cameraYaw=110,
        #     cameraPitch=-30,
        #     cameraTargetPosition=location
        # )
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)  #平滑视觉渲染
        p.stepSimulation()
        time.sleep(1./240.)

# %%
p.disconnect()

# %%
# https://python.hotexamples.com/examples/pybullet/-/setJointMotorControlArray/python-setjointmotorcontrolarray-function-examples.html
 # def setJointPosition(self, robot, position, kp=1.0, kv=0.3):
 #     import pybullet as p
 #     num_joints = p.getNumJoints(robot)
 #     zero_vec = [0.0] * num_joints
 #     if len(position) == num_joints:
 #         p.setJointMotorControlArray(robot, range(num_joints), p.POSITION_CONTROL,
 #             targetPositions=position, targetVelocities=zero_vec,
 #             positionGains=[kp] * num_joints, velocityGains=[kv] * num_joints)

  # def applyAction(self, actions):
  #   forces = [0.] * len(self.motors)
  #   for m in range(len(self.motors)):
  #     forces[m] = self.motor_power[m]*actions[m]*0.082
  #   p.setJointMotorControlArray(self.human, self.motors,controlMode=p.TORQUE_CONTROL, forces=forces)

  #   p.stepSimulation()
  #   time.sleep(0.01)
  #   distance=5
  #   yaw = 0
  
# 有的时候，我们希望能够禁用一些关节马达，那么我们可以通过将这个马达的force设为0来达到目的。这也就是官方文档提到的东西：

# maxForce = 0
# mode = p.VELOCITY_CONTROL
# p.setJointMotorControl2(objUid, jointIndex,
#  	controlMode=mode, force=maxForce)

# 在搭建模拟环境测试算法时，有时在调试过程中我们会需要能够知道模拟的过程中发生了什么，
# 毕竟我们不可能每时每刻都在电脑旁，而且训练时一般是不会开渲染的。pybullet中提供了几个保存当前环境状态的函数，
# saveState, saveBullet, restoreState。不同之处在于saveState会将目前模拟器的状态保存到内存中，
# 让这段程序后面可以随时读取内存中的这个模拟器状态，然后载入这个存档，因此saveState只需要指定模拟器环境ID，
# 返回一个状态ID；而saveBullet则是将状态保存到磁盘上，需要接受模拟器ID和路径。

# 除此之外，startStateLogging还能将记录存成json等文件格式。
# 而saveState和saveBullet保存的状态都可以由restoreState读取
# log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "log/robotmove.mp4")

# """
# 能够渲染的代码片段
# """

# p.stopStateLogging(log_id)