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
import tensorflow as tf
from pyquaternion import Quaternion
from pybullet_utils import bullet_client as bc
from numpy.linalg import inv
from numpy.linalg import pinv
import random
import copy
import matplotlib  
import matplotlib.pyplot as plt
from m_class import SetSimulation, Thread_print, Robot_info, CameraOperate
from m_class import PoE_func as PF
from m_class import ParameterInit, DHParameter, robot_control
import quaternion
import scipy

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

p.resetDebugVisualizerCamera(cameraTargetPosition=[0.36,-0.37,-0.17],\
                             cameraDistance=1.60,\
                             cameraPitch=-33.2,\
                             cameraYaw=224.40) #转变视角

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
# point_joint_base1 = DHParameter().DH_compute(Joint_pos)

joint_pos_err = np.zeros((numJoints, 3))
joint_orn_err = np.zeros((numJoints, 3))
# for i in range(11):
#     joint_orn_err[i] = [0.1, 0.1, 0.1]
#     # if i != 0:
#     #     joint_orn_err[i-1] = [0.0, 0.0, 0.0]
#     point_joint_base2 = DHParameter().DH_compute(Joint_pos, joint_pos_err, joint_orn_err)
#     print(point_joint_base1,'\n',point_joint_base2,'\n','************ \n','joint',i,'= ',point_joint_base1-point_joint_base2)
# pos = np.loadtxt("/home/lihui.liu/mnt/workspace/python/robot/model/2023_12_13_10_02_53.txt",usecols=(range(0,67)),delimiter=',',skiprows=27708,unpack=False,ndmin=0,encoding='bytes')

# %%

# end_point = DHParameter().DH_compute(Joint_pos)
joint_T = DHParameter().func_dh(Joint_pos)
Joint_pos_d = [-0.5, -0.5, 0.5, 0.5, -0.7, -0.1, -0.1, 0.5, 0.5, 0.5, 0.5]
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos_d)
p.addUserDebugLine([0, 0, 0], [1, 0, 0], lineColorRGB=[1,0,0], lineWidth=5)
p.addUserDebugLine([0, 0, 0], [0, 1, 0], lineColorRGB=[0,1,0], lineWidth=5)
p.addUserDebugLine([0, 0, 0], [0, 0, 1], lineColorRGB=[0,1,0], lineWidth=5)

# %%
# x = []
# for i in range(0,192394):
#     x.append(i)

# j1 = pos[:,21]
# j2 = pos[:,27]
# j3 = pos[:,33]
# j4 = pos[:,39]
# j5 = pos[:,45]
# j6 = pos[:,51]
# j7 = pos[:,57]
# j8 = pos[:,62]
# j1[0],j2[0],j3[0],j4[0],j5[0],j6[0],j7[0],j8[0]

# endx = pos[:,0] + end_point_start[0,3]
# endy = pos[:,1] + end_point_start[1,3]
# endz = pos[:,2] + end_point_start[2,3]
# thetax = pos[:,3]
# thetay = pos[:,4]
# thetaz = pos[:,5]
# endxd = pos[:,14] + end_point_start[0,3]
# endyd = pos[:,15] + end_point_start[1,3]
# endzd = pos[:,16] + end_point_start[2,3]
# thetaxd = pos[:,17]
# thetayd = pos[:,18]
# thetazd = pos[:,19]

# %%
# plt.xlabel('x')
# plt.ylabel('y')
# plt.title("pos")
# plt.plot(x, thetaxd - thetax)
# plt.show()

# # end_point_j = np.vstack([endx,endy,endz])
# # end_point_d = np.vstack([endxd,endyd,endzd])
# end_point = []
# end_pointd = []

# T_basez = np.array([[np.cos(-np.pi/2),-np.sin(-np.pi/2),0,0],
#                     [np.sin(-np.pi/2), np.cos(-np.pi/2),0,0],
#                     [0,0,1,0],
#                     [0,0,0,1]])
# %%
# for i in range(192397):
#     T_end = (np.array([
#         [np.cos(thetax[i])*np.cos(thetay[i]), np.cos(thetax[i])*np.sin(thetay[i])*np.sin(thetaz[i])-np.cos(thetaz[i])*np.sin(thetax[i]),np.sin(thetax[i])*np.sin(thetaz[i])+np.cos(thetax[i])*np.cos(thetaz[i])*np.sin(thetay[i]),endx[i]],
#         [np.cos(thetay[i])*np.cos(thetax[i]), np.cos(thetax[i])*np.cos(thetaz[i])+np.sin(thetax[i])*np.sin(thetay[i])*np.sin(thetaz[i]),np.cos(thetaz[i])*np.sin(thetax[i])*np.sin(thetay[i])-np.cos(thetax[i])*np.sin(thetaz[i]),endy[i]],
#         [-np.cos(thetay[i])*np.sin(thetaz[i]),np.sin(thetay[i]),                                                                        np.cos(thetay[i])*np.cos(thetaz[i]),                                          endz[i]],
#         [0,                             0,                                                                         0,                                                                         1]])
#         )
#     end_point.append(np.dot(T_end))
#     T_endd = (np.array([
#         [np.cos(thetaxd[i])*np.cos(thetayd[i]), np.cos(thetaxd[i])*np.sin(thetayd[i])*np.sin(thetazd[i])-np.cos(thetazd[i])*np.sin(thetaxd[i]),np.sin(thetaxd[i])*np.sin(thetazd[i])+np.cos(thetaxd[i])*np.cos(thetazd[i])*np.sin(thetayd[i]),endxd[i]],
#         [np.cos(thetayd[i])*np.cos(thetaxd[i]), np.cos(thetaxd[i])*np.cos(thetazd[i])+np.sin(thetaxd[i])*np.sin(thetayd[i])*np.sin(thetazd[i]),np.cos(thetazd[i])*np.sin(thetaxd[i])*np.sin(thetayd[i])-np.cos(thetaxd[i])*np.sin(thetazd[i]),endyd[i]],
#         [-np.cos(thetayd[i])*np.sin(thetazd[i]),np.sin(thetayd[i]),                                                                        np.cos(thetayd[i])*np.cos(thetazd[i]),                                          endzd[i]],
#         [0,                             0,                                                                         0,                                                                         1]])
#         )
#     end_pointd.append(np.dot(T_basez,T_endd))
# p.addUserDebugPoints(pointPositions=[end_point[0][0:3,3]], pointColorsRGB=[[0.3,1,0]], pointSize=5)
# p.addUserDebugPoints(pointPositions=[end_point_start[0:3,3]], pointColorsRGB=[[0.3,1,0]], pointSize=5)
# %%
# delta = joint_T.inv()
# theta = jacobian_k_-1 * [dx, dy, dz, theta_x, theta_y, theta_z].T
# Joint_pos_d = [0.5, -0.5, 0.5, 0.7, -0.7, -0.1, -0.1, 0.5, 0.5, 0.5, 0.5]
# p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos_d)
# end_point_pos_d, end_point_orn_d = DHParameter().DH_compute(Joint_pos_d)
# p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos_d)
joint_T = DHParameter().func_dh(Joint_pos)
joint_T_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), joint_T, "numpy")
Joint_pos = [0, 0.7, 0.8, 2.3, -0.5, 1.0, 0, 0.5, 0.5, 0.5, 0.5]
Joint_pos = [0, 0.7, 0.8, 2.3, -0.5, 0.3, -0.06, 0.5, 0.5, 0.5, 0.5]
Joint_pos = [0, 0.7, 0.6, 2.3, -0.3, 0.6, -0.06, 0.5, 0.5, 0.5, 0.5]
# Joint_pos = [j1[0],j2[0],j3[0]/2,j4[0],j5[0],j6[0]-3.14/2,j7[0],j8[0],0,0,0]
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos)
end_point_start = DHParameter().DH_compute(Joint_pos)
TrocarPoint = end_point_start[0:3,3]
sleep(1./240.)
# Joint_pos = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
# end_point_pos, end_point_orn = DHParameter().DH_compute(Joint_pos)

w_a = 5.78397543858225
w_b = 4.24657731
w_b = 4246.57731
# w_a7 = 5.78397543858225
w_b7 = 14246577.31
w_b7 = 142465773.1
theta_i = np.array(Joint_pos[0:8])
theta_ik = np.array(Joint_pos[0:8])

theta1 = sympy.symbols('theta1')
theta2 = sympy.symbols('theta2')
theta3 = sympy.symbols('theta3')
theta4 = sympy.symbols('theta4')
theta5 = sympy.symbols('theta5')
theta6 = sympy.symbols('theta6')
theta7 = sympy.symbols('theta7')
theta8 = sympy.symbols('theta8')
thetak1 = sympy.symbols('thetak1')
thetak2 = sympy.symbols('thetak2')
thetak3 = sympy.symbols('thetak3')
thetak4 = sympy.symbols('thetak4')
thetak5 = sympy.symbols('thetak5')
thetak6 = sympy.symbols('thetak6')
thetak7 = sympy.symbols('thetak7')
thetak8 = sympy.symbols('thetak8')
end1 = sympy.symbols('end1')
end2 = sympy.symbols('end2')
end3 = sympy.symbols('end3')
end4 = sympy.symbols('end4')
end5 = sympy.symbols('end5')
end6 = sympy.symbols('end6')
end7 = sympy.symbols('end7')
end8 = sympy.symbols('end8')
end9 = sympy.symbols('end9')
end10 = sympy.symbols('end10')
end11 = sympy.symbols('end11')
end12 = sympy.symbols('end12')

args = sympy.Matrix([theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8])
base_x = end_point_start[0][3]
base_y = end_point_start[1][3]
base_z = end_point_start[2][3]
# orn_z = end_point_orn_d[2][2]
end_pos_new_1 = end_point_start
end_pos_new = end_point_start
end_point_k = end_point_start

DRV1_LOW = -2.85
DRV1_HIGH = 2.85
DRV2_LOW  = -2.181
DRV2_HIGH = 2.181
# DRV2_LOW  = -1.57
# DRV2_HIGH = 1.57
DRV3_LOW  = -4.4
DRV3_HIGH = 2.25
# DRV3_HIGH = 1.25
DRV4_LOW  = 0.3
DRV4_HIGH = 2.8
DRV5_LOW  = -2.70526
DRV5_HIGH = 2.70526
# DRV6_LOW  = 0.61 + 1.5707
# DRV6_HIGH = 2.53 + 1.5707
DRV6_LOW  = -0.83 + 1.5707
DRV6_HIGH = 0.61 + 1.5707
DRV7_LOW  = -0.07
DRV7_HIGH = 0.07
DRV8_LOW  = -5.75
DRV8_HIGH = 5.75

f1 = joint_T[0] - end1
f2 = joint_T[1] - end2
f3 = joint_T[2] - end3
f4 = joint_T[3] - end4
f5 = joint_T[4] - end5
f6 = joint_T[5] - end6
f7 = joint_T[6] - end7
f8 = joint_T[7] - end8
f9 = joint_T[8] - end9
f10 = joint_T[9] - end10
f11 = joint_T[10] - end11
f12 = joint_T[11] - end12
# f13 = (sympy.Piecewise((1e10,thetak1 < DRV1_LOW), (2/(DRV1_HIGH-DRV1_LOW)*(thetak1-(DRV1_HIGH+DRV1_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV1_HIGH-DRV1_LOW)*(thetak1-(DRV1_HIGH+DRV1_LOW)/2)))-1) * (theta1-thetak1)**2, thetak1 < (DRV1_HIGH + DRV1_LOW)/2), (2/(DRV1_HIGH-DRV1_LOW)*(thetak1-(DRV1_HIGH+DRV1_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV1_HIGH-DRV1_LOW)*(thetak1-(DRV1_HIGH+DRV1_LOW)/2)))-1) * (theta1-thetak1)**2, thetak1 < DRV1_HIGH), (1e10, True)) + \
#        sympy.Piecewise((1e10,thetak2 < DRV2_LOW), (2/(DRV2_HIGH-DRV2_LOW)*(thetak2-(DRV2_HIGH+DRV2_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV2_HIGH-DRV2_LOW)*(thetak2-(DRV2_HIGH+DRV2_LOW)/2)))-1) * (theta2-thetak2)**2, thetak2 < (DRV2_HIGH + DRV2_LOW)/2), (2/(DRV2_HIGH-DRV2_LOW)*(thetak2-(DRV2_HIGH+DRV2_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV2_HIGH-DRV2_LOW)*(thetak2-(DRV2_HIGH+DRV2_LOW)/2)))-1) * (theta2-thetak2)**2, thetak2 < DRV2_HIGH), (1e10, True)) + \
#        sympy.Piecewise((1e10,thetak3 < DRV3_LOW), (2/(DRV3_HIGH-DRV3_LOW)*(thetak3-(DRV3_HIGH+DRV3_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV3_HIGH-DRV3_LOW)*(thetak3-(DRV3_HIGH+DRV3_LOW)/2)))-1) * (theta3-thetak3)**2, thetak3 < (DRV3_HIGH + DRV3_LOW)/2), (2/(DRV3_HIGH-DRV3_LOW)*(thetak3-(DRV3_HIGH+DRV3_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV3_HIGH-DRV3_LOW)*(thetak3-(DRV3_HIGH+DRV3_LOW)/2)))-1) * (theta3-thetak3)**2, thetak3 < DRV3_HIGH), (1e10, True)) + \
#        sympy.Piecewise((1e10,thetak4 < DRV4_LOW), (2/(DRV4_HIGH-DRV4_LOW)*(thetak4-(DRV4_HIGH+DRV4_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV4_HIGH-DRV4_LOW)*(thetak4-(DRV4_HIGH+DRV4_LOW)/2)))-1) * (theta4-thetak4)**2, thetak4 < (DRV4_HIGH + DRV4_LOW)/2), (2/(DRV4_HIGH-DRV4_LOW)*(thetak4-(DRV4_HIGH+DRV4_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV4_HIGH-DRV4_LOW)*(thetak4-(DRV4_HIGH+DRV4_LOW)/2)))-1) * (theta4-thetak4)**2, thetak4 < DRV4_HIGH), (1e10, True)) + \
#        sympy.Piecewise((1e10,thetak5 < DRV5_LOW), (2/(DRV5_HIGH-DRV5_LOW)*(thetak5-(DRV5_HIGH+DRV5_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV5_HIGH-DRV5_LOW)*(thetak5-(DRV5_HIGH+DRV5_LOW)/2)))-1) * (theta5-thetak5)**2, thetak5 < (DRV5_HIGH + DRV5_LOW)/2), (2/(DRV5_HIGH-DRV5_LOW)*(thetak5-(DRV5_HIGH+DRV5_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV5_HIGH-DRV5_LOW)*(thetak5-(DRV5_HIGH+DRV5_LOW)/2)))-1) * (theta5-thetak5)**2, thetak5 < DRV5_HIGH), (1e10, True)) + \
#        sympy.Piecewise((1e10,thetak6 < DRV6_LOW), (2/(DRV6_HIGH-DRV6_LOW)*(thetak6-(DRV6_HIGH+DRV6_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV6_HIGH-DRV6_LOW)*(thetak6-(DRV6_HIGH+DRV6_LOW)/2)))-1) * (theta6-thetak6)**2, thetak6 < (DRV6_HIGH + DRV6_LOW)/2), (2/(DRV6_HIGH-DRV6_LOW)*(thetak6-(DRV6_HIGH+DRV6_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV6_HIGH-DRV6_LOW)*(thetak6-(DRV6_HIGH+DRV6_LOW)/2)))-1) * (theta6-thetak6)**2, thetak6 < DRV6_HIGH), (1e10, True)) + \
#        sympy.Piecewise((1e10,thetak7 < DRV7_LOW), (2/(DRV7_HIGH-DRV7_LOW)*(thetak7-(DRV7_HIGH+DRV7_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV7_HIGH-DRV7_LOW)*(thetak7-(DRV7_HIGH+DRV7_LOW)/2)))-1) * (theta7-thetak7)**2, thetak7 < (DRV7_HIGH + DRV7_LOW)/2), (2/(DRV7_HIGH-DRV7_LOW)*(thetak7-(DRV7_HIGH+DRV7_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV7_HIGH-DRV7_LOW)*(thetak7-(DRV7_HIGH+DRV7_LOW)/2)))-1) * (theta7-thetak7)**2, thetak7 < DRV7_HIGH), (1e10, True)) + \
#        sympy.Piecewise((1e10,thetak8 < DRV8_LOW), (2/(DRV8_HIGH-DRV8_LOW)*(thetak8-(DRV8_HIGH+DRV8_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV8_HIGH-DRV8_LOW)*(thetak8-(DRV8_HIGH+DRV8_LOW)/2)))-1) * (theta8-thetak8)**2, thetak8 < (DRV8_HIGH + DRV8_LOW)/2), (2/(DRV8_HIGH-DRV8_LOW)*(thetak8-(DRV8_HIGH+DRV8_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV8_HIGH-DRV8_LOW)*(thetak8-(DRV8_HIGH+DRV8_LOW)/2)))-1) * (theta8-thetak8)**2, thetak8 < DRV8_HIGH), (1e10, True)))

f13 = sympy.Piecewise((1e10*theta1,thetak1 < DRV1_LOW), (-2/(DRV1_HIGH-DRV1_LOW)*(thetak1-(DRV1_HIGH+DRV1_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV1_HIGH-DRV1_LOW)*(thetak1-(DRV1_HIGH+DRV1_LOW)/2)))-1) * (theta1-thetak1)**2, thetak1 < (DRV1_HIGH + DRV1_LOW)/2), (2/(DRV1_HIGH-DRV1_LOW)*(thetak1-(DRV1_HIGH+DRV1_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV1_HIGH-DRV1_LOW)*(thetak1-(DRV1_HIGH+DRV1_LOW)/2)))-1) * (theta1-thetak1)**2, thetak1 < DRV1_HIGH), (1e10*theta1, True))
f14 = sympy.Piecewise((1e10*theta2,thetak2 < DRV2_LOW), (-2/(DRV2_HIGH-DRV2_LOW)*(thetak2-(DRV2_HIGH+DRV2_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV2_HIGH-DRV2_LOW)*(thetak2-(DRV2_HIGH+DRV2_LOW)/2)))-1) * (theta2-thetak2)**2, thetak2 < (DRV2_HIGH + DRV2_LOW)/2), (2/(DRV2_HIGH-DRV2_LOW)*(thetak2-(DRV2_HIGH+DRV2_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV2_HIGH-DRV2_LOW)*(thetak2-(DRV2_HIGH+DRV2_LOW)/2)))-1) * (theta2-thetak2)**2, thetak2 < DRV2_HIGH), (1e10*theta2, True))
f15 = sympy.Piecewise((1e10*theta3,thetak3 < DRV3_LOW), (-2/(DRV3_HIGH-DRV3_LOW)*(thetak3-(DRV3_HIGH+DRV3_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV3_HIGH-DRV3_LOW)*(thetak3-(DRV3_HIGH+DRV3_LOW)/2)))-1) * (theta3-thetak3)**2, thetak3 < (DRV3_HIGH + DRV3_LOW)/2), (2/(DRV3_HIGH-DRV3_LOW)*(thetak3-(DRV3_HIGH+DRV3_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV3_HIGH-DRV3_LOW)*(thetak3-(DRV3_HIGH+DRV3_LOW)/2)))-1) * (theta3-thetak3)**2, thetak3 < DRV3_HIGH), (1e10*theta3, True))
f16 = sympy.Piecewise((1e10*theta4,thetak4 < DRV4_LOW), (-2/(DRV4_HIGH-DRV4_LOW)*(thetak4-(DRV4_HIGH+DRV4_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV4_HIGH-DRV4_LOW)*(thetak4-(DRV4_HIGH+DRV4_LOW)/2)))-1) * (theta4-thetak4)**2, thetak4 < (DRV4_HIGH + DRV4_LOW)/2), (2/(DRV4_HIGH-DRV4_LOW)*(thetak4-(DRV4_HIGH+DRV4_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV4_HIGH-DRV4_LOW)*(thetak4-(DRV4_HIGH+DRV4_LOW)/2)))-1) * (theta4-thetak4)**2, thetak4 < DRV4_HIGH), (1e10*theta4, True))
f17 = sympy.Piecewise((1e10*theta5,thetak5 < DRV5_LOW), (-2/(DRV5_HIGH-DRV5_LOW)*(thetak5-(DRV5_HIGH+DRV5_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV5_HIGH-DRV5_LOW)*(thetak5-(DRV5_HIGH+DRV5_LOW)/2)))-1) * (theta5-thetak5)**2, thetak5 < (DRV5_HIGH + DRV5_LOW)/2), (2/(DRV5_HIGH-DRV5_LOW)*(thetak5-(DRV5_HIGH+DRV5_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV5_HIGH-DRV5_LOW)*(thetak5-(DRV5_HIGH+DRV5_LOW)/2)))-1) * (theta5-thetak5)**2, thetak5 < DRV5_HIGH), (1e10*theta5, True))
f18 = sympy.Piecewise((1e10*theta6,thetak6 < DRV6_LOW), (-2/(DRV6_HIGH-DRV6_LOW)*(thetak6-(DRV6_HIGH+DRV6_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV6_HIGH-DRV6_LOW)*(thetak6-(DRV6_HIGH+DRV6_LOW)/2)))-1) * (theta6-thetak6)**2, thetak6 < (DRV6_HIGH + DRV6_LOW)/2), (2/(DRV6_HIGH-DRV6_LOW)*(thetak6-(DRV6_HIGH+DRV6_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV6_HIGH-DRV6_LOW)*(thetak6-(DRV6_HIGH+DRV6_LOW)/2)))-1) * (theta6-thetak6)**2, thetak6 < DRV6_HIGH), (1e10*theta6, True))
f18 = thetak6
f19 = sympy.Piecewise((1e10*theta7,thetak7 < DRV7_LOW), (-2/(DRV7_HIGH-DRV7_LOW)*(thetak7-(DRV7_HIGH+DRV7_LOW)/2)*w_b7/(sympy.exp(w_a*(1+2/(DRV7_HIGH-DRV7_LOW)*(thetak7-(DRV7_HIGH+DRV7_LOW)/2)))-1) * (theta7-thetak7)**2, thetak7 < (DRV7_HIGH + DRV7_LOW)/2), (2/(DRV7_HIGH-DRV7_LOW)*(thetak7-(DRV7_HIGH+DRV7_LOW)/2)*w_b7/(sympy.exp(w_a*(1-2/(DRV7_HIGH-DRV7_LOW)*(thetak7-(DRV7_HIGH+DRV7_LOW)/2)))-1) * (theta7-thetak7)**2, thetak7 < DRV7_HIGH), (1e10*theta7, True))
f20 = sympy.Piecewise((1e10*theta8,thetak8 < DRV8_LOW), (-2/(DRV8_HIGH-DRV8_LOW)*(thetak8-(DRV8_HIGH+DRV8_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV8_HIGH-DRV8_LOW)*(thetak8-(DRV8_HIGH+DRV8_LOW)/2)))-1) * (theta8-thetak8)**2, thetak8 < (DRV8_HIGH + DRV8_LOW)/2), (2/(DRV8_HIGH-DRV8_LOW)*(thetak8-(DRV8_HIGH+DRV8_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV8_HIGH-DRV8_LOW)*(thetak8-(DRV8_HIGH+DRV8_LOW)/2)))-1) * (theta8-thetak8)**2, thetak8 < DRV8_HIGH), (1e10*theta8, True))
f21 = sympy.sqrt(theta1**2+theta2**2+theta3**2+theta4**2+theta5**2+theta6**2+theta7**2+theta8**2) \
     -sympy.sqrt(thetak1**2+thetak2**2+thetak3**2+thetak4**2+thetak5**2+thetak6**2+thetak7**2+thetak8**2)
# f18 = theta6 - 0.3 - 1.5707
# f19 = theta7 - 0.06
# funcs = sympy.Matrix([f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12])
funcs = sympy.Matrix([f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14, f15, f16, f17, f18, f19, f20])
res = funcs.jacobian(args)
# resh = sympy.hessian(funcs,args)
start_time4 = time()
f_jacobian = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8',\
        'end1','end2','end3','end4','end5','end6','end7','end8','end9','end10','end11','end12',\
        'thetak1','thetak2','thetak3','thetak4','thetak5','thetak6','thetak7','thetak8'), res, "numpy")
f_funcs = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8',\
        'end1','end2','end3','end4','end5','end6','end7','end8','end9','end10','end11','end12',\
        'thetak1','thetak2','thetak3','thetak4','thetak5','thetak6','thetak7','thetak8'), funcs, "numpy")
f_1 = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8',\
        'end1','end2','end3','end4','end5','end6','end7','end8','end9','end10','end11','end12',\
        'thetak1','thetak2','thetak3','thetak4','thetak5','thetak6','thetak7','thetak8'), f1, "numpy")
f_18 = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8',\
        'end1','end2','end3','end4','end5','end6','end7','end8','end9','end10','end11','end12',\
        'thetak1','thetak2','thetak3','thetak4','thetak5','thetak6','thetak7','thetak8'), f18, "numpy")
f_19 = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8',\
        'end1','end2','end3','end4','end5','end6','end7','end8','end9','end10','end11','end12',\
        'thetak1','thetak2','thetak3','thetak4','thetak5','thetak6','thetak7','thetak8'), f19, "numpy")

sign = 1
signj = 1
run_x = 0.0002
run_y = -0.0002
run_z = 0.0002
end_roll = 0
end_pitch = 0
end_yaw = 0
derr = 0.5
T_run = np.array([[1, 0, 0, run_x],
                  [0, np.cos(3.14/180), -np.sin(3.14/180), run_y],
                  [0, np.sin(3.14/180),  np.cos(3.14/180), run_z],
                  [0, 0, 0, 1]])
end_point_end = np.matmul(end_point_start, T_run)

step_x = 0.00
step_y = 0.00
step_z = 0.00
step_x = 0.0002
step_y = -0.0002
step_z = 0.0002
# step_y = -0.0001
# step_z = 0.0001
thetax = 0
thetay = 0
thetaz = 0
thetax = 0.002
# thetay = 0.002
# thetaz = 0.002
i=0

T_step = np.array([[1, 0, 0, step_x],
                   [0, 1, 0, step_y],
                   [0, 0, 1, step_z],
                   [0, 0, 0, 1]])
err_list = []
err_limlist = []
theta_i_list = [theta_i]
end_point_k = end_point_start.copy()
num_i = [np.array([])]

# theta_i = np.array([ 0.5  ,  0.7 ,  1.0 ,  2.6 , -0.8 ,  0.1 , -0.06,  0.5 ])
# theta_ik = theta_i
# p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=[ 0.5  ,  0.7 ,  1.0 ,  2.6 , -0.8 ,  0.3 , -0.06,  0.5 ,0,0,0])

# %%
# log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "/home/lihui.liu//mnt/workspace/python/robot/vedio/jump6_ok6.mp4")

for j in range(1000):
# for time_sin in range(240):
    if j % 2000 == 0:
        sign = -sign
    if j % 2000 == 0:
        signj = -signj
    # if j % 40 == 0:
    #     sign = -sign
    # sign = 0
    
    T_x = np.array([[1, 0, 0, step_x*sign],
                    [0, np.cos(thetax*derr*signj), -np.sin(thetax*derr*signj), step_y*sign],
                    [0, np.sin(thetax*derr*signj),  np.cos(thetax*derr*signj), step_z*sign],
                    [0, 0, 0, 1]])
    T_y = np.array([[np.cos(thetay*derr*signj), 0, np.sin(thetay*derr*signj), 0],
                    [0,               1, 0,            0],
                    [-np.sin(thetay*derr*signj), 0, np.cos(thetay*derr*signj), 0],
                    [0,               0, 0,            1]])
    T_z = np.array([[np.cos(thetaz*derr*signj), -np.sin(thetaz*derr*signj), 0, 0],
                    [np.sin(thetaz*derr*signj), np.cos(thetaz*derr*signj),    0, 0],
                    [0,            0,               1, 0],
                    [0,            0,               0, 1]])

    # end_point_k = np.matmul(end_point_k, T_step)
    end_point_k = np.matmul(end_point_k, T_z)
    end_point_k = np.matmul(end_point_k, T_y)
    end_point_k = np.matmul(end_point_k, T_x)
    # end_point_k = end_point[i]
    
    theta_i[5] = theta_i[5] + np.pi/2
    # joint_T_v = joint_T_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
    # test_end_point_k = np.dot(joint_T_v,[0,0,-0.5,1])
    # np.random.normal(loc=0.0, scale=0.002, size=(3,4))
    # noise = np.random.normal(loc=0.0, scale=0.002, size=8)
    # noise = np.random.uniform(-0.002, 0.002, 8)
    # theta_ik = theta_i + noise
    theta_ik = theta_i
    # theta_ik[0:3,] = theta_ik[0:3,] + noise
    p.addUserDebugLine(end_pos_new_1[0:3,3], end_pos_new[0:3,3], lineColorRGB=[0.3,0.2,0.6], lineWidth=1)
    # p.addUserDebugPoints(pointPositions=[test_end_point_k[0:3]], pointColorsRGB=[[0.3,1,0]], pointSize=5)
    end_pos_new_1 = end_pos_new
    # start_time2 = time()
    f_theta_i = f_funcs(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7],\
        end_point_k[0][0],end_point_k[0][1],end_point_k[0][2],end_point_k[0][3],\
        end_point_k[1][0],end_point_k[1][1],end_point_k[1][2],end_point_k[1][3],\
        end_point_k[2][0],end_point_k[2][1],end_point_k[2][2],end_point_k[2][3],\
        theta_ik[0],theta_ik[1],theta_ik[2],theta_ik[3],theta_ik[4],theta_ik[5],theta_ik[6],theta_ik[7])
    k = 0
    err_robot = 0
    for i in range(300):
        err_robot_old = err_robot
        JointJacobian = f_jacobian(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7],\
            end_point_k[0][0],end_point_k[0][1],end_point_k[0][2],end_point_k[0][3],\
            end_point_k[1][0],end_point_k[1][1],end_point_k[1][2],end_point_k[1][3],\
            end_point_k[2][0],end_point_k[2][1],end_point_k[2][2],end_point_k[2][3],\
            theta_ik[0],theta_ik[1],theta_ik[2],theta_ik[3],theta_ik[4],theta_ik[5],theta_ik[6],theta_ik[7])
        JointJacobianNpPinv = np.linalg.pinv(JointJacobian)
        theta_i = theta_i - 0.1*np.matmul(JointJacobianNpPinv, f_theta_i).T[0]
        f_theta_i = f_funcs(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7],\
            end_point_k[0][0],end_point_k[0][1],end_point_k[0][2],end_point_k[0][3],\
            end_point_k[1][0],end_point_k[1][1],end_point_k[1][2],end_point_k[1][3],\
            end_point_k[2][0],end_point_k[2][1],end_point_k[2][2],end_point_k[2][3],\
            theta_ik[0],theta_ik[1],theta_ik[2],theta_ik[3],theta_ik[4],theta_ik[5],theta_ik[6],theta_ik[7])

        err_robot = np.linalg.norm(f_theta_i[0:12],2)
        # if -1e-8 < err_robot_old - err_robot < 1e-8:
            # print(err_robot_old - err_robot)
        #     k = k+1
        # else:
        #     k = 0
        # if k == 5:
            # noise = np.random.uniform(-0.002, 0.002, 8)
            # theta_i = theta_i + noise
            # theta_i = theta_i + [0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01]
            # print(err_robot)
        # err_robot = np.linalg.norm(f_theta_i,2)
        err_lim = 0
        err_d = 0
        err_lim = np.linalg.norm(f_theta_i[12:-1],2)
        # err_d = f_theta_i[-1]
        err_list.append(err_robot)
        err_limlist.append(err_lim)
        theta_i_list.append(theta_i)
        # print(theta_i)
        # print(err_robot)
        if err_robot < 5e-3 and err_lim < 1e-2 and err_d < 1:
        # if err_robot < 3e-3 :
            # print('+' * 50)
            # print('i = ',i)
            break
        # if i == 1:
        #     print(j,'\n')
        #     print('1')
        if i == 500:
            print(j,'\n')
            print('500')
        elif i == 1000:
            print('1000')
        elif i == 3000:
            print('3000')
        elif i == 6000:
            print('6000')
    num_i = np.append(num_i,values=i)
    # np.concatenate(num_i,i)
    theta_i[5] = theta_i[5] - np.pi/2
    # print(theta_i)
    # theta_ik = theta_i
    # noise = np.random.normal(loc=0.0, scale=0.002, size=8)
    noise = np.random.uniform(-0.005, 0.005, 8)
    # theta_i = theta_i + noise
    # theta_ik = theta_i
    # theta_ik = theta_i
    # theta_ik[5] = theta_ik[5] - np.pi/2
    Joint_pos_new = np.concatenate((theta_i,Joint_pos[8:12]))
    end_pos_new = DHParameter().DH_compute(Joint_pos_new)
    p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=Joint_pos_new)
    sleep(1./240.)
    # j = j+1
# p.stopStateLogging(log_id)
theta_i[5] = theta_i[5] - np.pi/2
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

end_point_poe = PF().fk6(theta_i)
# %%
for i in range(11):
    Robot_info(robot_id,i).joint_state_info()

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

ff = f_1(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7],\
    end_point_k[0][0],end_point_k[0][1],end_point_k[0][2],end_point_k[0][3],\
    end_point_k[1][0],end_point_k[1][1],end_point_k[1][2],end_point_k[1][3],\
    end_point_k[2][0],end_point_k[2][1],end_point_k[2][2],end_point_k[2][3],\
    theta_ik[0],theta_ik[1],theta_ik[2],theta_ik[3],theta_ik[4],theta_ik[5],theta_ik[6],theta_ik[7])
f_18(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7],\
    end_point_k[0][0],end_point_k[0][1],end_point_k[0][2],end_point_k[0][3],\
    end_point_k[1][0],end_point_k[1][1],end_point_k[1][2],end_point_k[1][3],\
    end_point_k[2][0],end_point_k[2][1],end_point_k[2][2],end_point_k[2][3],\
    theta_ik[0],theta_ik[1],theta_ik[2],theta_ik[3],theta_ik[4],theta_ik[5],theta_ik[6],theta_ik[7])
    
    
sympy.Piecewise((1e10,theta_i[5] < DRV6_LOW), (2/(DRV6_HIGH-DRV6_LOW)*(theta_i[5]-(DRV6_HIGH+DRV6_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV6_HIGH-DRV6_LOW)*(theta_i[5]-(DRV6_HIGH+DRV6_LOW)/2)))-1) * (theta_ik[5]-theta_i[5])**2, theta_i[5] < (DRV6_HIGH + DRV6_LOW)/2), (2/(DRV6_HIGH-DRV6_LOW)*(theta_i[5]-(DRV6_HIGH+DRV6_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV6_HIGH-DRV6_LOW)*(theta_i[5]-(DRV6_HIGH+DRV6_LOW)/2)))-1) * (theta_ik[5]-theta_i[5])**2, theta_i[5] < DRV6_HIGH), (1e10, True))


f19.subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]),
          (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7]),
          (end1,end_point_k[0][0]),(end1,end_point_k[0][1]),(end1,end_point_k[0][2]),(end1,end_point_k[0][3]),
          (end1,end_point_k[1][0]),(end1,end_point_k[1][1]),(end1,end_point_k[1][2]),(end1,end_point_k[1][3]),
          (end1,end_point_k[2][0]),(end1,end_point_k[2][1]),(end1,end_point_k[2][2]),(end1,end_point_k[2][3]),
          (thetak1,theta_ik[0]),(thetak2,theta_ik[1]),(thetak3,theta_ik[2]),(thetak4,theta_ik[3]),
          (thetak5,theta_ik[4]),(thetak6,theta_ik[5]),(thetak7,theta_ik[6]),(thetak8,theta_ik[7])])

def func2(y):
    return y*100

# func2 = y*100
x = np.arange(0,10)
print(x)
xx=np.piecewise(x, [x < 4, x >= 6], [func2, 1])

xxxx=np.piecewise(x, [x < 4, x >= 6], [lambda x:x**2, lambda x:x*100, 157])
print(xx)

print(xxxx)

x = sympy.Symbol('x')
f = x**2
g = x - 2
h = sympy.log(x)
p = sympy.Piecewise((f, x<0), (g, x<2), (h, True))

print(p.subs(x, -2))
print(p.subs(x, 1))
print(p.subs(x, 10))

a, b, c = sympy.symbols('a b c')
eqs = [sympy.Eq(0.5*b/(sympy.exp(a*(1-0.5))-1), 8000),
       sympy.Eq(0.8*b/(sympy.exp(a*(1-0.8))-1), 100000)]
       # sympy.Eq(-x - y + 5 * z, 42)]
print(sympy.solve(eqs, [a, b, c]))

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei'] #用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False #用来正常显示负号

wa = 5.78397543858225
wb = 2.724657731
# 一元一次函数图像
x = np.arange(-1, 1, 0.1)
y = np.piecewise(x, [x<0, 0<x], [lambda x:-wb*x/(np.exp(wa*(1+x))-1), lambda x:wb*x/(np.exp(wa*(1-x))-1)])
plt.xlabel('x')
plt.ylabel('y')
plt.title("一元一次函数")
plt.plot(x, y)
plt.show()

38923*theta_ik[6]*2*(theta_i[6]-theta_ik[6])/(325*np.exp(-82*theta_ik[6])-1)
-38935*theta_ik[6]*(theta_i[6]-theta_ik[6])**2/(323.8/np.exp(-82.596*theta_ik[6])-1)

sympy.Piecewise((1e10,theta_ik[6] < DRV7_LOW), (-2/(DRV7_HIGH-DRV7_LOW)*(theta_ik[6]-(DRV7_HIGH+DRV7_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV7_HIGH-DRV7_LOW)*(theta_ik[6]-(DRV7_HIGH+DRV7_LOW)/2)))-1) * (theta_i[6]-theta_ik[6])**2, theta_ik[6] < (DRV7_HIGH + DRV7_LOW)/2), (2/(DRV7_HIGH-DRV7_LOW)*(theta_ik[6]-(DRV7_HIGH+DRV7_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV7_HIGH-DRV7_LOW)*(theta_ik[6]-(DRV7_HIGH+DRV7_LOW)/2)))-1) * (theta_i[6]-theta_ik[6])**2, theta_ik[6] < DRV7_HIGH), (1e10, theta_ik[6] >= DRV7_HIGH))


f_19 = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8',\
        # 'end1','end2','end3','end4','end5','end6','end7','end8','end9','end10','end11','end12',\
        'thetak1','thetak2','thetak3','thetak4','thetak5','thetak6','thetak7','thetak8'), f19, "numpy")
f_19(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7],\
    0,0,0,0,0,0,0.076,0)

sampling_rate = 1024
tf = np.arange(0, 1.0, 1.0 / sampling_rate)
ff1 = 100
ff2 = 20
ff3 = 50
dataf = np.piecewise(tf, [tf < 1, tf < 0.8, 0 < tf < 0.3],
                    [lambda tf: np.sin(2 * np.pi * ff1 * tf),
                     lambda tf: np.sin(2 * np.pi * ff2 * tf),
                     lambda tf: np.sin(2 * np.pi * ff3 * tf)])



start_time = time()
f_jacobian = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), res, "numpy")
f_funcs = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), funcs, "numpy")
# p.addUserDebugLine([0, 0, 0], [1, 1, 1], lineColorRGB=[0.3,0.2,0.6], lineWidth=5)
end_time = time()
print(end_time - start_time)



p.addUserDebugLine(end_point_end[0:3,3], end_point_start[0:3,3], lineColorRGB=[0.3,0.2,0.6], lineWidth=5)

matrix = tf.constant([[6.,4.,7.,1.],[4.,1.,3.,2.],[2.,6.,8.,4.],[1.,9.,2.,4.]])
 
with tf.compat.v1.Session() as sess:
    print(tf.compat.v1.matrix_solve(matrix,[[3.],[2.],[7.],[8.]]).eval())

n = sympy.Symbol('n')
f= (1/2)**n
sympy.summation(f,(n,0,3)) #如下左图所示
 
#2、如果求和最终没有确定值，则会返回求和表达式
f= 1/sympy.log(n) +3
sympy.summation(f,(n,0,sympy.oo)) #如下右图所示

sympy.Piecewise((1e10,theta_ik[0] < DRV1_LOW), (2/(DRV1_HIGH-DRV1_LOW)*(theta_ik[0]-(DRV1_HIGH+DRV1_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV1_HIGH-DRV1_LOW)*(theta_ik[0]-(DRV1_HIGH+DRV1_LOW)/2)))-1) * (theta_i[0]-theta_ik[0])**2, theta_ik[0] < (DRV1_HIGH + DRV1_LOW)/2), (2/(DRV1_HIGH-DRV1_LOW)*(theta_ik[0]-(DRV1_HIGH+DRV1_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV1_HIGH-DRV1_LOW)*(theta_ik[0]-(DRV1_HIGH+DRV1_LOW)/2)))-1) * (theta_i[0]-theta_ik[0])**2, theta_ik[0] < DRV1_HIGH), (1e10, True))
sympy.Piecewise((1e10,theta_ik[1] < DRV2_LOW), (2/(DRV2_HIGH-DRV2_LOW)*(theta_ik[1]-(DRV2_HIGH+DRV2_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV2_HIGH-DRV2_LOW)*(theta_ik[1]-(DRV2_HIGH+DRV2_LOW)/2)))-1) * (theta_i[1]-theta_ik[1])**2, theta_ik[1] < (DRV2_HIGH + DRV2_LOW)/2), (2/(DRV2_HIGH-DRV2_LOW)*(theta_ik[1]-(DRV2_HIGH+DRV2_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV2_HIGH-DRV2_LOW)*(theta_ik[1]-(DRV2_HIGH+DRV2_LOW)/2)))-1) * (theta_i[1]-theta_ik[1])**2, theta_ik[1] < DRV2_HIGH), (1e10, True))
sympy.Piecewise((1e10,theta_ik[2] < DRV3_LOW), (2/(DRV3_HIGH-DRV3_LOW)*(theta_ik[2]-(DRV3_HIGH+DRV3_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV3_HIGH-DRV3_LOW)*(theta_ik[2]-(DRV3_HIGH+DRV3_LOW)/2)))-1) * (theta_i[2]-theta_ik[2])**2, theta_ik[2] < (DRV3_HIGH + DRV3_LOW)/2), (2/(DRV3_HIGH-DRV3_LOW)*(theta_ik[2]-(DRV3_HIGH+DRV3_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV3_HIGH-DRV3_LOW)*(theta_ik[2]-(DRV3_HIGH+DRV3_LOW)/2)))-1) * (theta_i[2]-theta_ik[2])**2, theta_ik[2] < DRV3_HIGH), (1e10, True))
sympy.Piecewise((1e10,theta_ik[3] < DRV4_LOW), (2/(DRV4_HIGH-DRV4_LOW)*(theta_ik[3]-(DRV4_HIGH+DRV4_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV4_HIGH-DRV4_LOW)*(theta_ik[3]-(DRV4_HIGH+DRV4_LOW)/2)))-1) * (theta_i[3]-theta_ik[3])**2, theta_ik[3] < (DRV4_HIGH + DRV4_LOW)/2), (2/(DRV4_HIGH-DRV4_LOW)*(theta_ik[3]-(DRV4_HIGH+DRV4_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV4_HIGH-DRV4_LOW)*(theta_ik[3]-(DRV4_HIGH+DRV4_LOW)/2)))-1) * (theta_i[3]-theta_ik[3])**2, theta_ik[3] < DRV4_HIGH), (1e10, True))
sympy.Piecewise((1e10,theta_ik[4] < DRV5_LOW), (2/(DRV5_HIGH-DRV5_LOW)*(theta_ik[4]-(DRV5_HIGH+DRV5_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV5_HIGH-DRV5_LOW)*(theta_ik[4]-(DRV5_HIGH+DRV5_LOW)/2)))-1) * (theta_i[4]-theta_ik[4])**2, theta_ik[4] < (DRV5_HIGH + DRV5_LOW)/2), (2/(DRV5_HIGH-DRV5_LOW)*(theta_ik[4]-(DRV5_HIGH+DRV5_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV5_HIGH-DRV5_LOW)*(theta_ik[4]-(DRV5_HIGH+DRV5_LOW)/2)))-1) * (theta_i[4]-theta_ik[4])**2, theta_ik[4] < DRV5_HIGH), (1e10, True))
sympy.Piecewise((1e10,theta_ik[5] < DRV6_LOW), (2/(DRV6_HIGH-DRV6_LOW)*(theta_ik[5]-(DRV6_HIGH+DRV6_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV6_HIGH-DRV6_LOW)*(theta_ik[5]-(DRV6_HIGH+DRV6_LOW)/2)))-1) * (theta_i[5]-theta_ik[5])**2, theta_ik[5] < (DRV6_HIGH + DRV6_LOW)/2), (2/(DRV6_HIGH-DRV6_LOW)*(theta_ik[5]-(DRV6_HIGH+DRV6_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV6_HIGH-DRV6_LOW)*(theta_ik[5]-(DRV6_HIGH+DRV6_LOW)/2)))-1) * (theta_i[5]-theta_ik[5])**2, theta_ik[5] < DRV6_HIGH), (1e10, True))
sympy.Piecewise((1e10,theta_ik[6] < DRV7_LOW), (2/(DRV7_HIGH-DRV7_LOW)*(theta_ik[6]-(DRV7_HIGH+DRV7_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV7_HIGH-DRV7_LOW)*(theta_ik[6]-(DRV7_HIGH+DRV7_LOW)/2)))-1) * (theta_i[6]-theta_ik[6])**2, theta_ik[6] < (DRV7_HIGH + DRV7_LOW)/2), (2/(DRV7_HIGH-DRV7_LOW)*(theta_ik[6]-(DRV7_HIGH+DRV7_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV7_HIGH-DRV7_LOW)*(theta_ik[6]-(DRV7_HIGH+DRV7_LOW)/2)))-1) * (theta_i[6]-theta_ik[6])**2, theta_ik[6] < DRV7_HIGH), (1e10, True))
sympy.Piecewise((1e10,theta_ik[7] < DRV8_LOW), (2/(DRV8_HIGH-DRV8_LOW)*(theta_ik[7]-(DRV8_HIGH+DRV8_LOW)/2)*w_b/(sympy.exp(w_a*(1+2/(DRV8_HIGH-DRV8_LOW)*(theta_ik[7]-(DRV8_HIGH+DRV8_LOW)/2)))-1) * (theta_i[7]-theta_ik[7])**2, theta_ik[7] < (DRV8_HIGH + DRV8_LOW)/2), (2/(DRV8_HIGH-DRV8_LOW)*(theta_ik[7]-(DRV8_HIGH+DRV8_LOW)/2)*w_b/(sympy.exp(w_a*(1-2/(DRV8_HIGH-DRV8_LOW)*(theta_ik[7]-(DRV8_HIGH+DRV8_LOW)/2)))-1) * (theta_i[7]-theta_ik[7])**2, theta_ik[7] < DRV8_HIGH), (1e10, True))

f13.subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2]), (theta4,theta_i[3]),\
          (theta5,theta_i[4]), (theta6,theta_i[5]), (theta7,theta_i[6]), (theta8,theta_i[7]),
          (thetak1,theta_ik[0]),(thetak2,theta_ik[1]),(thetak3,theta_ik[2]),(thetak4,theta_ik[3]),\
          (thetak5,theta_ik[4]),(thetak6,theta_ik[5]),(thetak7,theta_ik[6]),(thetak8,theta_ik[7])])


2 / 0.4 * theta_i[6] * w_b / (np.exp(w_a*(1-2/0.4*theta_i[6]))-1) * (theta_i[6]-theta_ik[6])**2

2/(DRV7_HIGH-DRV7_LOW)*(theta_i[6]-(DRV7_HIGH+DRV7_LOW)/2)*w_b \
    /(np.exp(w_a*(1-2/(DRV7_HIGH-DRV7_LOW)*(theta_i[6]-(DRV7_HIGH+DRV7_LOW)/2)))-1)
    
2/(DRV7_HIGH-DRV7_LOW)*(theta_i[6]-(DRV7_HIGH+DRV7_LOW)/2)

2/(DRV7_HIGH-DRV7_LOW)*(0.07-(DRV7_HIGH+DRV7_LOW)/2)

2 / 0.4 * 0.01 * w_b / (np.exp(w_a*(1-2/0.4*0.01))-1)

2/(DRV7_HIGH-DRV7_LOW)*(0.06999-(DRV7_HIGH+DRV7_LOW)/2)*w_b \
    /(np.exp(w_a*(1-2/(DRV7_HIGH-DRV7_LOW)*(0.06999-(DRV7_HIGH+DRV7_LOW)/2)))-1)

#define DRV1_POSITION_LIMIT_LOW  (-2.85)
#define DRV1_POSITION_LIMIT_HIGH (2.85)
#define DRV2_POSITION_LIMIT_LOW  (-2.181)
#define DRV2_POSITION_LIMIT_HIGH (2.181)
#define DRV3_POSITION_LIMIT_LOW  (-4.4)
#define DRV3_POSITION_LIMIT_HIGH (1.25)
#define DRV4_POSITION_LIMIT_LOW  (0.3)
#define DRV4_POSITION_LIMIT_HIGH (2.8)
#define DRV5_POSITION_LIMIT_LOW  (-2.70526)
#define DRV5_POSITION_LIMIT_HIGH (2.70526)
#define DRV6_POSITION_LIMIT_LOW  (0.61)
#define DRV6_POSITION_LIMIT_HIGH (2.53)
#define DRV7_POSITION_LIMIT_LOW  (-0.07)
#define DRV7_POSITION_LIMIT_HIGH (0.07)
#define DRV8_POSITION_LIMIT_LOW  (-5.75)
#define DRV8_POSITION_LIMIT_HIGH (5.75)

#define DRV1_VELOCITY_LIMIT_N  (-2.2)
#define DRV1_VELOCITY_LIMIT_P  (2.2)
#define DRV2_VELOCITY_LIMIT_N  (-2.2)
#define DRV2_VELOCITY_LIMIT_P  (2.2)
#define DRV3_VELOCITY_LIMIT_N  (-2.5)
#define DRV3_VELOCITY_LIMIT_P  (2.5)
#define DRV4_VELOCITY_LIMIT_N  (-2.5)
#define DRV4_VELOCITY_LIMIT_P  (2.5)
#define DRV5_VELOCITY_LIMIT_N  (-2.5)
#define DRV5_VELOCITY_LIMIT_P  (2.5)
#define DRV6_VELOCITY_LIMIT_N  (-2.5)
#define DRV6_VELOCITY_LIMIT_P  (2.5)
#define DRV7_VELOCITY_LIMIT_N  (-0.5)
#define DRV7_VELOCITY_LIMIT_P  (0.5)
#define DRV8_VELOCITY_LIMIT_N  (-10.0)
#define DRV8_VELOCITY_LIMIT_P  (10.0)
#define DRV9_VELOCITY_LIMIT_N  (-40.0)
#define DRV9_VELOCITY_LIMIT_P  (40.0)
#define DRV10_VELOCITY_LIMIT_N (-40.0)
#define DRV10_VELOCITY_LIMIT_P (40.0)
#define DRV11_VELOCITY_LIMIT_N (-40.0)
#define DRV11_VELOCITY_LIMIT_P (40.0)

#define DRV1_TORQUE_D_LIMIT_N  (-14.0)
#define DRV1_TORQUE_D_LIMIT_P  (14.0)
#define DRV2_TORQUE_D_LIMIT_N  (-14.0)
#define DRV2_TORQUE_D_LIMIT_P  (14.0)
#define DRV3_TORQUE_D_LIMIT_N  (-7.5)
#define DRV3_TORQUE_D_LIMIT_P  (7.5)
#define DRV4_TORQUE_D_LIMIT_N  (-6.0)
#define DRV4_TORQUE_D_LIMIT_P  (6.0)
#define DRV5_TORQUE_D_LIMIT_N  (-3.0)
#define DRV5_TORQUE_D_LIMIT_P  (3.0)
#define DRV6_TORQUE_D_LIMIT_N  (-24000)
#define DRV6_TORQUE_D_LIMIT_P  (24000)
#define DRV7_TORQUE_D_LIMIT_N  (-24000)
#define DRV7_TORQUE_D_LIMIT_P  (24000)
#define DRV8_TORQUE_D_LIMIT_N  (-14000)
#define DRV8_TORQUE_D_LIMIT_P  (14000)
#define DRV9_TORQUE_D_LIMIT_N  (-3000)
#define DRV9_TORQUE_D_LIMIT_P  (3000)
#define DRV10_TORQUE_D_LIMIT_N (-3000)
#define DRV10_TORQUE_D_LIMIT_P (3000)
#define DRV11_TORQUE_D_LIMIT_N (-3000)
#define DRV11_TORQUE_D_LIMIT_P (3000)
