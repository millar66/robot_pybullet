#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 28 13:49:26 2023

@author: lihui.liu
"""
import sympy
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

class numerical_calculation:
    
    def JacobianBody():
        
        
        
    def IKinBody():
        a,b,x,y = sympy.symbols("a b x y")
        a = 2*x**3 - y**2 - 1
        b = x*y**3 - y - 4
        funcs = sympy.Matrix([a,b])
        args = sympy.Matrix([x,y])
        res = funcs.jacobian(args)
        pprint(res)
        

    def FKinBody(self, joint_positions=[0]*11, joint_pos_err=np.zeros((11, 3)), joint_orn_err=np.zeros((11, 3)), end_pos=[0,0,0]):
        base_pos = np.array(end_pos+[1])
        numJoints = 11
        # pos, orn = self.getPosOrn()
        pos = [[0.00000, 0.00000, 0.27985],
               [0.00000, 0.00000, 0.00000],
               [0.00000, -0.36330, 0.00000],
               [0.04951, 0.00000, 0.00000],
               [0.04951, 0.36665, 0.00000],
               [0.00000, 0.00000, 0.00000],
               [0.00000, 0.00000, 0.00000],
               [0.04050, 0.00000, 0.55443-0.16],
               [0.00000, 0.00000, 0.16],
               [0.00000, 0.01125, 0.00000],
               [0.00000, 0.00000, 0.00000]]

        orn = [[0.00000, 0.00000, 0.00000],
               [-1.57078, 0.00000, 0.00000],
               [-1.57079, 0.00000, -3.14159],
               [1.57079, 0.00000, 3.14159],
               [1.57078, 0.00000, 1.57078],
               [-3.1416, 1.5708, 3.14159],
               [-1.5708, 0.00000, 0.00000],
               [0.00000, 0.00000, 0.00000],
               [1.5706, 0.00000, 0.00000],
               [1.57080, 3.14159, 0.00000],
               [1.57080, 3.14159, 0.00000]]

        Td = np.zeros((numJoints, 4, 4))
        Tx = np.zeros((numJoints, 4, 4))
        Ty = np.zeros((numJoints, 4, 4))
        Tz = np.zeros((numJoints, 4, 4))
        T_dot = np.eye(4)
        T_joint = np.array([0., 0., 0.,1.]*(numJoints+1))
        T_joint.resize(12,4)
        point_joint = np.zeros((numJoints+1, 3))
        orn_cos_x = np.zeros(numJoints)
        orn_sin_x = np.zeros(numJoints)
        orn_cos_y = np.zeros(numJoints)
        orn_sin_y = np.zeros(numJoints)
        orn_cos_z = np.zeros(numJoints)
        orn_sin_z = np.zeros(numJoints)
        for i in range(numJoints):
            orn_cos_x[i] = np.cos(orn[i][0] + float(joint_orn_err[i][0])/1000)
            orn_sin_x[i] = np.sin(orn[i][0] + float(joint_orn_err[i][0])/1000)
            orn_cos_y[i] = np.cos(orn[i][1] + float(joint_orn_err[i][1])/1000)
            orn_sin_y[i] = np.sin(orn[i][1] + float(joint_orn_err[i][1])/1000)
            if i == 6:
                orn_cos_z[i] = np.cos(orn[i][1])
                orn_sin_z[i] = np.sin(orn[i][1])
                Td[i] = np.array([[1, 0, 0, pos[i][0] + joint_pos_err[i][0]/1000],
                                  [0, 1, 0, pos[i][1] + joint_pos_err[i][1]/1000 + joint_positions[i]],
                                  [0, 0, 1, pos[i][2] + joint_pos_err[i][2]/1000],
                                  [0, 0, 0, 1        ]])
            else:
                orn_cos_z[i] = np.cos(orn[i][2] + joint_orn_err[i][2]/1000 + joint_positions[i])
                orn_sin_z[i] = np.sin(orn[i][2] + joint_orn_err[i][2]/1000 + joint_positions[i])
                # Ta[i] = np.array([[1, 0, 0, pos[i][0]],
                #                   [0, 1, 0, 0        ],
                #                   [0, 0, 1, 0        ],
                #                   [0, 0, 0, 1        ]])
                # Td[i] = np.array([[1, 0, 0, 0        ],
                #                   [0, 1, 0, 0        ],
                #                   [0, 0, 1, pos[i][2]],
                #                   [0, 0, 0, 1        ]])
                Td[i] = np.array([[1, 0, 0, pos[i][0] + joint_pos_err[i][0]/1000],
                                  [0, 1, 0, pos[i][1] + joint_pos_err[i][1]/1000],
                                  [0, 0, 1, pos[i][2] + joint_pos_err[i][2]/1000],
                                  [0, 0, 0, 1        ]])
            # Talpha = np.array([[1, 0,            0,               0],
            #                   [0, orn_cos_x[i], orn_sin_x[i]*-1, 0],
            #                   [0, orn_sin_x[i], orn_cos_x[i],    0],
            #                   [0, 0,            0,               1]])
            # Tzeta = np.array([[1, 0,            0,               0],
            #                   [0, orn_cos_x[i], orn_sin_x[i]*-1, 0],
            #                   [0, orn_sin_x[i], orn_cos_x[i],    0],
            #                   [0, 0,            0,               1]])
            Tx[i] = np.array([[1, 0,            0,               0],
                              [0, orn_cos_x[i], orn_sin_x[i]*-1, 0],
                              [0, orn_sin_x[i], orn_cos_x[i],    0],
                              [0, 0,            0,               1]])
            Ty[i] = np.array([[orn_cos_y[i],    0, orn_sin_y[i], 0],
                              [0,               1, 0,            0],
                              [orn_sin_y[i]*-1, 0, orn_cos_y[i], 0],
                              [0,               0, 0,            1]])
            Tz[i] = np.array([[orn_cos_z[i], orn_sin_z[i]*-1, 0, 0],
                              [orn_sin_z[i], orn_cos_z[i],    0, 0],
                              [0,            0,               1, 0],
                              [0,            0,               0, 1]])
            
            T_dot = T_dot@Td[i]@Tx[i]@Ty[i]@Tz[i]
            T_joint[i+1] = T_dot@base_pos
            point_joint[i+1] = T_joint[i+1][0:3]
            p.addUserDebugLine(point_joint[i], point_joint[i+1], lineColorRGB=[0,0,1], lineWidth=5)
        # p.addUserDebugPoints(pointPositions=[point_joint[numJoints]], pointColorsRGB=[[1,0,1]], pointSize=6)
        # p.addUserDebugPoints(pointPositions=point_joint, pointColorsRGB=[[1,0,1]]*12, pointSize=6)
        return point_joint[numJoints]*1000
        
        