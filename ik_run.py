#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec  4 17:02:10 2023

@author: lihui.liu
"""

import os
import sympy
import xml.dom.minidom
import shutil
import numpy as np
import sys
sys.path.append('/home/lihui.liu/mnt/workspace/python/robot/robot_pybullet')
import pybullet as p
import pybullet_data
from time import sleep, time
import timeit
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
from m_class import SetSimulation, Thread_print, Robot_info, CameraOperate
from m_class import ParameterInit, DHParameter, robot_control

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
# robot_id = p.loadURDF("./ddd/000PSM_10.SLDASM/urdf/000PSM_10.SLDASM.urdf",
#                       basePosition=[0, 0, 0], useMaximalCoordinates=False, useFixedBase=True)

# %%
numJoints = 11

Joint_pos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
Joint_pos = [0, 0.5, -0.5, 0.5, -0.5, -0.5, 0.1, 0.5, 0.5, 0.5, 0.5]
# Joint_pos = [0, 0, 0, 0, 0, 0, 0, -1, 0.5, 0, 0]
# Joint_pos = [1.57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos)
sleep(1.)
joint_positions, joint_velocities, joint_torques = robot_control.getJointStates(robot_id)
point_joint_base, point_joint_orn = DHParameter().DH_compute(Joint_pos)
# joint_T = []
end_point = np.array([0., 0., 0])
end_orn = np.array([0, 0, 0.])

for i in range(240 * 10):
    Joint_pos[0] = 1 * np.sin(2 * np.pi * 0.5 * i / 240)
    if Joint_pos[0] > 1 :
        print('******')
    p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos)
    sleep(1./24.)
    joint_positions, joint_velocities, joint_torques = robot_control.getJointStates(robot_id)
    point_joint_pos, point_joint_orn = DHParameter().DH_compute(Joint_pos)
    joint_T = DHParameter().func_dh(Joint_pos, end_point, end_orn)

# %%
numJoints = 11
Joint_pos = [0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.1, 0.5, 0.5, 0.5, 0.5]
point_joint_base1 = DHParameter().DH_compute(Joint_pos)

joint_pos_err = np.zeros((numJoints, 3))
joint_orn_err = np.zeros((numJoints, 3))
for i in range(11):
    joint_orn_err[i] = [0.1, 0.1, 0.1]
    # if i != 0:
    #     joint_orn_err[i-1] = [0.0, 0.0, 0.0]
    point_joint_base2 = DHParameter().DH_compute(Joint_pos, joint_pos_err, joint_orn_err)
    print(point_joint_base1,'\n',point_joint_base2,'\n','************ \n','joint',i,'= ',point_joint_base1-point_joint_base2)

# %%

end_point = DHParameter().DH_compute(Joint_pos)
joint_T = DHParameter().func_dh(Joint_pos)
Joint_pos_d = [-0.5, -0.5, 0.5, 0.5, -0.7, -0.1, -0.1, 0.5, 0.5, 0.5, 0.5]
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos_d)
p.addUserDebugLine([0, 0, 0], [1, 0, 0], lineColorRGB=[1,0,0], lineWidth=5)
p.addUserDebugLine([0, 0, 0], [0, 1, 0], lineColorRGB=[0,1,0], lineWidth=5)
p.addUserDebugLine([0, 0, 0], [0, 0, 1], lineColorRGB=[0,1,0], lineWidth=5)
# %%
# delta = joint_T.inv()
# theta = jacobian_k_-1 * [dx, dy, dz, theta_x, theta_y, theta_z].T
# Joint_pos_d = [0.5, -0.5, 0.5, 0.7, -0.7, -0.1, -0.1, 0.5, 0.5, 0.5, 0.5]
# p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos_d)
# end_point_pos_d, end_point_orn_d = DHParameter().DH_compute(Joint_pos_d)
# p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos_d)
joint_T = DHParameter().func_dh(Joint_pos)
Joint_pos = [0, 0.7, 0.2, 2.2, -0.5, -0.0, 0, 0.5, 0.5, 0.5, 0.5]
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos)
end_point_start = DHParameter().DH_compute(Joint_pos)
sleep(1./240.)
# Joint_pos = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
# end_point_pos, end_point_orn = DHParameter().DH_compute(Joint_pos)

# theta = sympy.symbols('theta1:12')
theta1 = sympy.symbols('theta1')
theta2 = sympy.symbols('theta2')
theta3 = sympy.symbols('theta3')
theta4 = sympy.symbols('theta4')
theta5 = sympy.symbols('theta5')
theta6 = sympy.symbols('theta6')
theta7 = sympy.symbols('theta7')
theta8 = sympy.symbols('theta8')
# f = sympy.symbols("f1:12")
f1 = joint_T[0] - end_point_start[0][0]
f2 = joint_T[1] - end_point_start[0][1]
f3 = joint_T[2] - end_point_start[0][2]
f4 = joint_T[3] - end_point_start[0][3]
f5 = joint_T[4] - end_point_start[1][0]
f6 = joint_T[5] - end_point_start[1][1]
f7 = joint_T[6] - end_point_start[1][2]
f8 = joint_T[7] - end_point_start[1][3]
f9 = joint_T[8] - end_point_start[2][0]
f10 = joint_T[9] - end_point_start[2][1]
f11 = joint_T[10] - end_point_start[2][2]
f12 = joint_T[11] - end_point_start[2][3]
f13 = theta7 - Joint_pos[6]
# funcs = sympy.Matrix([f1, f2, f3, f4, f5, f6, f7, f8])
funcs = sympy.Matrix([f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13])
# funcs = sympy.Matrix(f)
args = sympy.Matrix([theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8])
res = funcs.jacobian(args)
JointJacobian = res.subs([(theta1,Joint_pos[0]), (theta2,Joint_pos[1]), (theta3,Joint_pos[2]), (theta4,Joint_pos[3]), (theta5,Joint_pos[4]), (theta6,Joint_pos[5]), (theta7,Joint_pos[6]), (theta8,Joint_pos[7])])
JointJacobianNp = np.array(JointJacobian)
JointJacobianNp = JointJacobianNp.astype(float)
JointJacobianNpPinv = np.linalg.pinv(JointJacobianNp)
theta_i = np.array(Joint_pos[0:8])
# theta_i[5] = theta_i[5] + np.pi/2
f_jacobian = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), res, "numpy")
f_funcs = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), funcs, "numpy")

base_x = end_point_start[0][3]
base_y = end_point_start[1][3]
base_z = end_point_start[2][3]
# orn_z = end_point_orn_d[2][2]
end_pos_new_1 = end_point_start
end_pos_new = end_point_start
end_point_k = end_point_start

run_x = 0
run_y = 0
run_z = 0.1
end_roll = 0
end_pitch = 0
end_yaw = 0
T_run = np.array([[1, 0, 0, run_x],
                  [0, 1, 0, run_y],
                  [0, 0, 1, run_z],
                  [0, 0, 0, 1]])
end_point_end = np.matmul(end_point_start, T_run)
step_x = run_x / 240.
step_y = run_y / 240.
step_z = run_z / 240.
i=0

T_step = np.array([[1, 0, 0, step_x],
                   [0, 1, 0, step_y],
                   [0, 0, 1, step_z],
                   [0, 0, 0, 1]])

# %%
# log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "/home/lihui.liu//mnt/workspace/python/robot/vedio/end_z.mp4")

for j in range(240):
# for time_sin in range(240):
    
    start_time1 = time()
    end_point_k = np.matmul(end_point_k, T_step)
    # end_point_k[2][3] = end_point_k[2][3] + 0.01
    
    f1 = joint_T[0] - end_point_k[0][0]
    f2 = joint_T[1] - end_point_k[0][1]
    f3 = joint_T[2] - end_point_k[0][2]
    f4 = joint_T[3] - end_point_k[0][3]
    f5 = joint_T[4] - end_point_k[1][0]
    f6 = joint_T[5] - end_point_k[1][1]
    f7 = joint_T[6] - end_point_k[1][2]
    f8 = joint_T[7] - end_point_k[1][3]
    f9 = joint_T[8] - end_point_k[2][0]
    f10 = joint_T[9] - end_point_k[2][1]
    f11 = joint_T[10] - end_point_k[2][2]
    f12 = joint_T[11] - end_point_k[2][3]
    f13 = theta7 - Joint_pos[6]
    funcs = sympy.Matrix([f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13])
    res = funcs.jacobian(args)
    start_time3 = time()
    # JointJacobian = res.subs([(theta1,Joint_pos[0]), (theta2,Joint_pos[1]), (theta3,Joint_pos[2]), (theta4,Joint_pos[3]), (theta5,Joint_pos[4]), (theta6,Joint_pos[5]), (theta7,Joint_pos[6]), (theta8,Joint_pos[7])])
    start_time4 = time()
    # JointJacobianNp = np.array(JointJacobian)
    # JointJacobianNp = JointJacobianNp.astype(float)
    # JointJacobianNpPinv = np.linalg.pinv(JointJacobianNp)
    # theta_i = np.array(Joint_pos[0:8])
    theta_i[5] = theta_i[5] + np.pi/2
    f_jacobian = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), res, "numpy")
    f_funcs = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), funcs, "numpy")
    p.addUserDebugLine(end_pos_new_1[0:3,3], end_pos_new[0:3,3], lineColorRGB=[0.3,0.2,0.6], lineWidth=5)
    end_pos_new_1 = end_pos_new
    start_time2 = time()
    for i in range(10000):
        # print(i)
        # JointJacobian = res.subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]), (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7])])
        JointJacobian = f_jacobian(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
        # JointJacobianNp = np.array(JointJacobian)
        # JointJacobianNp = JointJacobianNp.astype(float)
        JointJacobianNpPinv = np.linalg.pinv(JointJacobian)
        # f_theta_i = funcs.subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]), (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7])])
        f_theta_i = f_funcs(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
        # f_theta_i_Np = np.array(f_theta_i)
        # f_theta_i_Np = f_theta_i_Np.astype(float)
        # f_theta_i_Np_Pinv = np.linalg.pinv(f_theta_i).T
        theta_i = theta_i - np.matmul(JointJacobianNpPinv, f_theta_i).T[0]
        err = np.linalg.norm(f_theta_i,2)
        # print(theta_i)
        # print(err)
        if err < 0.00003 :
            # print('+' * 50)
            # print('i = ',i)
            break
        if i == 500:
            print('500')
        elif i == 1000:
            print('1000')
        elif i == 3000:
            print('3000')
        elif i == 6000:
            print('6000')
    end_time2 = time()
    theta_i[5] = theta_i[5] - np.pi/2
    Joint_pos_new = np.concatenate((theta_i,Joint_pos[8:12]))
    end_pos_new = DHParameter().DH_compute(Joint_pos_new)
    p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos_new)
    sleep(1./240.)
    end_time1 = time()
    # print(start_time1 - start_time3)
    # print(start_time3 - start_time4)
    # print(start_time4 - start_time2)
# p.stopStateLogging(log_id)
theta_i[5] = theta_i[5] - np.pi/2
# print(err)
# print(theta_i)
Joint_pos_new = np.concatenate((theta_i,Joint_pos[8:12]))
end_point_new = DHParameter().DH_compute(Joint_pos_new)
# p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos_new)
# p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos_d)
    
    
# %%
Joint_pos = [0.3, 0.3, 0, -1.3, 0, 1.0, 0, 0, 0, 0, 0]
robot_control.setJointPosition(robot_id, Joint_pos, 11)
# p.addUserDebugPoints(pointPositions=[aaaaa[0:3]], pointColorsRGB=[[1,0,1]], pointSize=10)
# sleep(3.)
p.removeAllUserDebugItems()
    
p.disconnect(physicsClientId)


# %%



aaa = res.subs([(theta1,Joint_pos[0]), (theta2,Joint_pos[1]), (theta3,Joint_pos[2]), (theta4,Joint_pos[3]), (theta5,Joint_pos[4]), (theta6,Joint_pos[5]), (theta7,Joint_pos[6]), (theta8,Joint_pos[7])])
bbb = np.array(aaa)
bbb = bbb.astype(float)


JointJacobianNpDet = np.linalg.det(JointJacobianNp)
if JointJacobianNpDet < 0.0001 :
    print('Singular matrix')
else :
    print('++++++' * 30)
try:
    JointJacobianNpInv = np.linalg.inv(JointJacobianNp)
except np.linalg.LinAlgError as e:
    print(f"Error : {e}")
    print('-' * 50)
else:
    pprint(JointJacobianNpInv)
finally:
    print('*' * 80)




Joint_pos = [0.5, -0.5, 0.5, 1, 0.5, -0.5, 0.05, 0.5, 0.5, 0.5, 0.5]
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos)
end_point_pos, end_point_orn = DHParameter().DH_compute(Joint_pos)
joint_T, point_joint = DHParameter().func_dh(Joint_pos)

theta_i = np.array(Joint_pos)
theta_i[5] = theta_i[5] + np.pi/2
# theta_i[8] = theta_i[8] + np.pi/2
end_point_pos
point_joint
joint_T[3].subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]), (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7])])
joint_T[7].subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]), (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7])])
joint_T[11].subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]), (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7])])
joint_T[12].subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]), (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7])])
joint_T[0].subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]), (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7])])


f4.subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]), (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7])])
f1.subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]), (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7])])




a = joint_T * base_pos

a.subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]), (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7])])


point_joint_aa = np.zeros((numJoints+1, 3))
T_joint = []
for i in range(8):
    T_joint.append(joint_T[i] * base_pos)
    theta_i = np.array(Joint_pos)
    np_joint = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6'
                              ,'theta7','theta8','theta9','theta10','theta11'),T_joint[i],"numpy")
    point_joint_aa[i+1] = np_joint(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4]
                                ,theta_i[5] + np.pi/2,theta_i[6],theta_i[7],theta_i[8] + np.pi/2,theta_i[9]
                                ,theta_i[10])[0:3].T
print(point_joint_aa)

aaaa = np.array(res)


bbbb = aaaa.subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]), (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7])])


f = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), res, "numpy")
f(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])


aa = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), funcs, "numpy")

aa(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])

# f1 = joint_T[3] - end_point_pos_d[0]
joint_T[3].subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]), (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7])])

joint_T[7].subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]), (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7])])

joint_T[11].subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]), (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7])])

for i in range(6):
    exec('var{} = {}'.format(i, i*6))
print(var0, var1, var2, var3 ,var4 ,var5)

class Test_class(object):
    def __init__(self):
        names = self.__dict__
        for i in range(6):
            names['n' + str(i)] = i
            #names['x%s' % i] = i

t = Test_class()
print(t.n0, t.n1, t.n2, t.n3, t.n4, t.n5)












start_time = time()
f_jacobian = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), res, "numpy")
f_funcs = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), funcs, "numpy")
# p.addUserDebugLine([0, 0, 0], [1, 1, 1], lineColorRGB=[0.3,0.2,0.6], lineWidth=5)
end_time = time()
print(end_time - start_time)



p.addUserDebugLine(end_point_end[0:3,3], end_point_start[0:3,3], lineColorRGB=[0.3,0.2,0.6], lineWidth=5)






