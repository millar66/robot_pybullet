# -*- coding: utf-8 -*-
"""
Created on Wed Nov 15 09:43:21 2023

@author: LH
"""

from threading import Thread
from time import sleep, ctime, time
from queue import Queue
from threading import Event
import pybullet as p
import pybullet_data
import math
from pprint import pprint
import numpy as np

import os
import xml.etree.ElementTree as ET
import shutil
import xml.dom.minidom

import re
import sympy

# 创建 Thread 的子类
class Thread_print(Thread):
#    def __init__(self, func, args):
    def __init__(self, event, print_text_queue=Queue(50), print_content_queue=Queue(50),  run_time=300):
        '''
        by m.
        event
        print_content
        print_text
        run_time
        '''
        Thread.__init__(self)
#        self.func = func
        self.print_text_queue = print_text_queue
        self.print_content_queue = print_content_queue
        self.run_time = run_time
        self.event = event
#        self.result = None

    def run(self):
#       event = Event()
#       self.result = self.func(*self.args)
#       print_content = self.print_content_queue.get()
        print_text = 'Hello World :'
        print_content = 'm.'
        time_start = time()
        time_end = time()
        time_terminate_start = time()
        time_terminate_end = time()
        while True:
            time_terminate_end = time()
            if self.event.is_set():
                # 在此添加退出前要做的工作，如保存文件等
                print('Worker is ended')
                break
            elif ((time_terminate_end - time_terminate_start) > self.run_time):
                print('Worker is abnormal')
                break
            if not self.print_content_queue.empty():
                print_content = self.print_content_queue.get()
            if not self.print_text_queue.empty():
                print_text = self.print_text_queue.get()
            if self.print_content_queue.empty() and self.print_text_queue.empty():
                sleep(1./200.)
#        for i in range(100):
            # block for a moment
#            print_text = self.print_text_queue.get()
            # check for stop
#            print_content = self.print_content_queue.get_nowait()
            # report a message
            if (time_end - time_start) > 1:
                time_start = time()
                time_end = time()
                print(print_text, print_content)
            else:
                time_end = time()
        print('Thread is ended')

#    def getResult(self):
#        return self.result

class SetSimulation:
    def __init__(self, robot_id, numJoints=11):
#         '''
#         by m.
#         event
#         print_content
#         print_text
#         run_time
#         '''
#         Thread.__init__(self)
# #        self.func = func
#         self.print_text_queue = print_text_queue
#         self.print_content_queue = print_content_queue
        self.robot_id = robot_id
        self.numJoints = numJoints
        # self.r = r
        # self.result = None

    def Set_init_my_robot(self):
        # p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)
        # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        # p.setGravity(0, 0, -9.81)
        # p.setRealTimeSimulation(1)

        p.resetDebugVisualizerCamera(cameraTargetPosition=[0.05,0.02,0.39],\
                                      cameraDistance=1.20,\
                                      cameraPitch=-30.40,\
                                      cameraYaw=24.40) #转变视角
        
        # p.setDebugObjectColor(objectUniqueId=self.robot_id, linkIndex=-1, objectDebugColorRGB=[1, 0, 0])
        # p.setDebugObjectColor(objectUniqueId=self.robot_id, linkIndex=1, objectDebugColorRGB=[0, 1, 0])
        # p.setDebugObjectColor(objectUniqueId=self.robot_id, linkIndex=2, objectDebugColorRGB=[0, 0, 1])
        # p.setDebugObjectColor(objectUniqueId=self.robot_id, linkIndex=3, objectDebugColorRGB=[1, 0, 1])
        # p.setDebugObjectColor(objectUniqueId=self.robot_id, linkIndex=4, objectDebugColorRGB=[0, 1, 0])
        # p.setDebugObjectColor(objectUniqueId=self.robot_id, linkIndex=5, objectDebugColorRGB=[0, 0, 1])
        # p.setDebugObjectColor(objectUniqueId=self.robot_id, linkIndex=6, objectDebugColorRGB=[1, 0, 1])
        # p.setDebugObjectColor(objectUniqueId=self.robot_id, linkIndex=7, objectDebugColorRGB=[0, 1, 1])
        # p.setDebugObjectColor(objectUniqueId=self.robot_id, linkIndex=8, objectDebugColorRGB=[0, 0, 1])
        # p.setDebugObjectColor(objectUniqueId=self.robot_id, linkIndex=9, objectDebugColorRGB=[1, 0, 1])
        # p.setDebugObjectColor(objectUniqueId=self.robot_id, linkIndex=10, objectDebugColorRGB=[0, 1, 1])
        # p.setDebugObjectColor(objectUniqueId=self.robot_id, linkIndex=11, objectDebugColorRGB=[0, 0, 1])
        
        # changeVisualShape(objectUniqueId, linkIndex, shapeIndex, textureUniqueId, rgbaColor, specularColor, physicsClientId)
        p.changeVisualShape(self.robot_id, -1, rgbaColor=[255/255, 255/255, 255/255, 1])
        p.changeVisualShape(self.robot_id, 0, rgbaColor=[245/255, 40/255, 145/255, 0.8])
        p.changeVisualShape(self.robot_id, 1, rgbaColor=[15/255, 29/255, 248/255, 0.7])
        p.changeVisualShape(self.robot_id, 2, rgbaColor=[16/255, 239/255, 146/255, 0.43])
        p.changeVisualShape(self.robot_id, 3, rgbaColor=[244/255, 244/255, 9/255, 0.64])
        p.changeVisualShape(self.robot_id, 4, rgbaColor=[16/255, 246/255, 217/255, 0.94])
        p.changeVisualShape(self.robot_id, 5, rgbaColor=[240/255, 12/255, 15/255, 0.94])
        p.changeVisualShape(self.robot_id, 6, rgbaColor=[62/255, 5/255, 250/255, 0.94])
        p.changeVisualShape(self.robot_id, 7, rgbaColor=[224/255, 5/255, 250/255, 0.94])
        p.changeVisualShape(self.robot_id, 8, rgbaColor=[10/255, 247/255, 26/255, 0.94])
        p.changeVisualShape(self.robot_id, 9, rgbaColor=[200/255, 100/255, 10/255, 0.6])
        p.changeVisualShape(self.robot_id, 10, rgbaColor=[50/255, 1/255, 200/255, 0.6])
        
        for i in range(self.numJoints):
            p.resetJointState(self.robot_id, jointIndex=i, targetValue=0, targetVelocity=0)

        #load and set real robot
        # loadSDF(fileName, useMaximalCoordinates, globalScaling, physicsClientId)
        # loadMJCF(fileName, useMaximalCoordinates, globalScaling, physicsClientId)
        # robot_id = p.loadURDF("./uuu/000PSM_10.SLDASM/urdf/000PSM_10.SLDASM.urdf",
        #                       basePosition=[0, 0, 0], useMaximalCoordinates=False)
        # p.resetBasePositionAndOrientation(self.robot_id, self.pos, self.r)
        # plane_id = p.loadURDF("plane.urdf")
        # return robot_id
        # return
        
class Robot_info :
    
    def __init__(self, robot_id, index=0):
        self.robot_id = robot_id
        self.index = index
    
    def link_info(self):
        link_info = p.getDynamicsInfo(self.robot_id, self.index)
        print(f"\
                [0]质量: {link_info[0]}\n\
                [1]横向摩擦系数(lateral friction): {link_info[1]}\n\
                [2]主惯性矩: {link_info[2]}\n\
                [3]惯性坐标系的位置: {link_info[3]}\n\
                [4]惯性坐标系的姿态: {link_info[4]}\n\
                [5]恢复系数: {link_info[5]}\n\
                [6]滚动摩擦系数: {link_info[6]}\n\
                [7]扭转摩擦系数: {link_info[7]}\n\
                [8]接触阻尼: {link_info[8]}\n\
                [9]接触刚度: {link_info[9]}\n\
                [10]物体属性(1=刚体，2=多刚体，3=软体): {link_info[10]}\n\
                [11]碰撞边界: {link_info[11]}\n\n")
                
    def joint_info(self):
        joint_info = p.getJointInfo(self.robot_id, self.index)
        print(f"\
                [0]Index: {joint_info[0]}\n\
                [1]Name: {joint_info[1]}\n\
                [2]Type(REVOLUTE=0;PRISMATIC=1;PLANAR=3;FIXED=4): {joint_info[2]}\n\
                [3]此主体的位置状态变量中的第一个位置索引: {joint_info[3]}\n\
                [4]在这个物体的速度状态变量中的第一个速度索引: {joint_info[4]}\n\
                [5]Reserve: {joint_info[5]}\n\
                [6]Joint_Damping: {joint_info[6]}\n\
                [7]Frictional coefficient: {joint_info[7]}\n\
                [8]Min_rad: {joint_info[8]}\n\
                [9]Max_rad: {joint_info[9]}\n\
                [10]maxJointMoment: {joint_info[10]}\n\
                [11]maxJointVelocity: {joint_info[11]}\n\
                [12]Link_name: {joint_info[12]}\n\
                [13]Rotation axis: {joint_info[13]}\n\
                [14]pos in parent: {joint_info[14]}\n\
                [15]ore in parent: {joint_info[15]}\n\
                [16]parent index(base is -1): {joint_info[16]}\n\n")
                
    def active_joint_info(self):
        active_joint_id = [i for i in range(p.getNumJoints(self.robot_id)) if p.getJointInfo(self.robot_id, i)[2] != p.JOINT_FIXED]
        print("活动关节ID: ", active_joint_id)
        active_joint_name = [p.getJointInfo(self.robot_id, i)[1] for i in active_joint_id]
        print("活动关节名称: ")
        # pprint("活动关节名称: ", active_joint_name)  # pprint()函数打印出来的数据结构更加完整，每行为一个数据结构
        pprint(active_joint_name)
        wheel_joints_indexes = [i for i in active_joint_id if "wheel" in str(p.getJointInfo(self.robot_id, i)[1])]
        print("轮子关节索引", wheel_joints_indexes)
        
    def joint_state_info(self):
        # joint_list = [self.index]
        joint_state = p.getJointStates(self.robot_id, [self.index])
        # joint_state = p.getJointStates(self.robot_id, [self.index], physicsClient)
        print(f"\
                [0]jointPosition: {joint_state[0][0]}\n\
                [1]jointVelocity: {joint_state[0][1]}\n\
                [2]jointReactionForces: {joint_state[0][2]}\n\
                [3]appliedJointMotorTorque: {joint_state[0][3]}\n\n")
                
    def link_state_info(self):
#         computeLinkVelocity: 如果设置为1，则将计算笛卡尔世界坐标系下的速度并返回。
#         computeForwardKinematics: 如果设置为1(或True)，笛卡尔世界位置/方向将使用正向运动学重新计算。
        link_info = p.getLinkState(self.robot_id, self.index, computeLinkVelocity=1, computeForwardKinematics=1)
        print(f"\
                [0]linkWorldPosition: {link_info[0]}\n\
                [1]linkWorldOrientation: {link_info[1]}\n\
                [2]localInertialFramePosition: {link_info[2]}\n\
                [3]localInertialFrameOrientation: {link_info[3]}\n\
                [4]worldLinkFramePosition: {link_info[4]}\n\
                [5]worldLinkFrameOrientation: {link_info[5]}\n\
                [6]worldLinkLinearVelocity: {link_info[6]}\n\
                [7]worldLinkAngularVelocity: {link_info[7]}\n\n")
                
    def getInertial(self, joint_id):
        """ get inertial data. """
        xmlDoc = xml.dom.minidom.parse("/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/ddd/000PSM_10.SLDASM/urdf/test.xml")

        mass = xmlDoc.getElementsByTagName("mass")[joint_id].getAttribute("value")
        ixx  = xmlDoc.getElementsByTagName("inertia")[joint_id].getAttribute("ixx")
        ixy  = xmlDoc.getElementsByTagName("inertia")[joint_id].getAttribute("ixy")
        ixz  = xmlDoc.getElementsByTagName("inertia")[joint_id].getAttribute("ixz")
        iyy  = xmlDoc.getElementsByTagName("inertia")[joint_id].getAttribute("iyy")
        iyz  = xmlDoc.getElementsByTagName("inertia")[joint_id].getAttribute("iyz")
        izz  = xmlDoc.getElementsByTagName("inertia")[joint_id].getAttribute("izz")
        xyz  = xmlDoc.getElementsByTagName("inertial")[joint_id].getElementsByTagName("origin")[0].getAttribute("xyz")
        xyz  = xyz.split(" ")
        xyz  = np.array(xyz,dtype=np.float32)
        data = [float(mass),float(ixx),float(iyy),float(izz),float(ixy),float(ixz),float(iyz),xyz[0],xyz[1],xyz[2]]
        link_count = xmlDoc.getElementsByTagName("link").length
        joint_count = xmlDoc.getElementsByTagName("joint").length
        print("joint_count= ", joint_count)
        print("mass= ", data[0])
        print("质心X：", data[7])
        print("质心y：", data[8])
        print("质心z：", data[9])
        return data

class ParameterInit :
    
    def pos_lim():
        shutil.copyfile("/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/ddd/000PSM_10.SLDASM/urdf/000PSM_10.SLDASM.urdf",
                        "/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/ddd/000PSM_10.SLDASM/urdf/test.urdf")

        if os.path.exists("/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/ddd/000PSM_10.SLDASM/urdf/test.urdf"):
            os.replace("/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/ddd/000PSM_10.SLDASM/urdf/test.urdf",
                       "/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/ddd/000PSM_10.SLDASM/urdf/test.xml")

        tree = ET.parse('/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/ddd/000PSM_10.SLDASM/urdf/test.xml')
        numJoints = 11
        root = tree.getroot()
        elements_joint = root.findall('joint')
        elements_link = root.findall('link')
        joint_limit = [elements_joint[i].findall('limit') for i in range(numJoints)]
        joint_parent = [elements_joint[i].findall('parent') for i in range(numJoints)]
        joint_axis = [elements_joint[i].findall('axis') for i in range(numJoints)]
        joint_origin_list = [elements_joint[i].findall('origin') for i in range(11)]
        # link_2_visual = elements_link[2].findall('visual')
        # link_2_origin = link_2_visual[0].findall('origin')
        # link_2_origin[0].set('rpy',  '0, 0, -0.13')
        # link_3_visual = elements_link[3].findall('visual')
        # link_3_origin = link_3_visual[0].findall('origin')
        # link_3_origin[0].set('rpy',  '0, 3.14, 3.14')
        # link_4_inertial = elements_link[4].findall('inertial')
        # link_4_inertial_origin = link_4_inertial[0].findall('origin')
        # link_4_inertial_origin[0].set('xyz',  '0, 0, 0')
        # link_5_visual = elements_link[5].findall('visual')
        # link_5_inertial = elements_link[5].findall('inertial')
        # link_5_origin = link_5_visual[0].findall('origin')
        # link_5_inertial_origin = link_5_inertial[0].findall('origin')
        # link_5_origin[0].set('rpy',  '0, 3.14, 0')
        # link_5_inertial_origin[0].set('xyz',  '0, 0, 0')
        # link_7_visual = elements_link[7].findall('visual')
        # link_7_origin = link_7_visual[0].findall('origin')
        # link_7_origin[0].set('rpy',  '0 0 1.57')

        joint_lower = [0] * numJoints
        joint_upper = [0] * numJoints
        joint_effort = [0] * numJoints
        joint_velocity = [0] * numJoints
        for i in range(numJoints):
            joint_lower[i] = joint_limit[i][0].get('lower')
            joint_upper[i] = joint_limit[i][0].get('upper')
            joint_effort[i] = joint_limit[i][0].get('effort')
            joint_velocity[i] = joint_limit[i][0].get('velocity')
        joint_xyz = [0] * numJoints
        for i in range(numJoints):
            joint_xyz[i] = joint_axis[i][0].get('xyz')
        joint_pos = [0] * numJoints
        joint_orn = [0] * numJoints
        for i in range(numJoints):
            joint_pos[i] = joint_origin_list[i][0].get('xyz')
            joint_orn[i] = joint_origin_list[i][0].get('rpy')
        print('*'*20)
        # pprint(joint_xyz)
        pprint(joint_pos)
        pprint(joint_orn)
        # pprint(joint_lower)
        # pprint(joint_upper)
        # pprint(joint_effort)
        # pprint(joint_velocity)

        # joint_origin_list[0][0].set('xyz', '0.00000 0.00000 0.27985')
        # joint_origin_list[1][0].set('xyz', '0.00000 0.00000 0.00000')
        joint_origin_list[2][0].set('xyz', '0.00000 -0.36330 0.00000')
        joint_origin_list[3][0].set('xyz', '0.04951 0.00000 0.00000')
        joint_origin_list[4][0].set('xyz', '0.04951 0.36665 0.00000')
        # joint_origin_list[5][0].set('xyz', '0.00000 0.00000 0.00000')
        # joint_origin_list[6][0].set('xyz', '0.00000 0.00000 0.00000')
        # joint_origin_list[7][0].set('xyz', '-0.04050 0.00000 0.55443')
        # joint_origin_list[8][0].set('xyz', '0.00000 0.00000 0.00000')
        joint_origin_list[9][0].set('xyz', '0.01125 0.00000 0.00000')
        joint_origin_list[10][0].set('xyz','0.01125 0.00000 0.00000')

        # joint_origin_list[0][0].set('rpy', '0.00000 0.00000 0.00000')
        # joint_origin_list[1][0].set('rpy', '-1.57078 0.00000 0.00000')
        joint_origin_list[2][0].set('rpy', '-1.57079 0.00000 -3.14159')
        joint_origin_list[3][0].set('rpy', '1.57079 0.00000 3.14159')
        joint_origin_list[4][0].set('rpy', '1.57078 0.00000 -3.14159')
        # joint_origin_list[5][0].set('rpy', '-3.1416 1.5708 1.5708')
        # joint_origin_list[6][0].set('rpy', '-1.5708 0.00000 0.00000')
        # joint_origin_list[7][0].set('rpy', '0.00000 0.00000 0.00000')
        # joint_origin_list[8][0].set('rpy', '1.5706 -1.5708 0.00000')
        joint_origin_list[9][0].set('rpy', '-1.57080 0.00000 3.14159')
        joint_origin_list[10][0].set('rpy','-1.57080 0.00000 3.14159')
        joint_parent[9][0].set('link', 'Link9')
        joint_parent[10][0].set('link', 'Link9')
        joint_axis[0][0].set('xyz', '0 0 1')
        joint_axis[1][0].set('xyz', '0 0 1')
        joint_axis[2][0].set('xyz', '0 0 1')
        joint_axis[3][0].set('xyz', '0 0 1')
        joint_axis[4][0].set('xyz', '0 0 1')
        joint_axis[5][0].set('xyz', '0 0 1')
        joint_axis[6][0].set('xyz', '0 0 1')
        joint_axis[7][0].set('xyz', '0 0 1')
        joint_axis[8][0].set('xyz', '0 0 1')
        joint_axis[9][0].set('xyz', '0 0 1')
        joint_axis[10][0].set('xyz', '0 0 1')
        joint_limit[0][0].set('lower', '-3.14')
        joint_limit[1][0].set('lower', '-3.14')
        joint_limit[2][0].set('lower', '-3.14')
        joint_limit[3][0].set('lower', '-3.14')
        joint_limit[4][0].set('lower', '-3.14')
        joint_limit[5][0].set('lower', '-3.14')
        joint_limit[6][0].set('lower', '-0.2')
        joint_limit[7][0].set('lower', '-3.14')
        joint_limit[8][0].set('lower', '-3.14')
        joint_limit[9][0].set('lower', '-3.14')
        joint_limit[10][0].set('lower', '-3.14')
        joint_limit[0][0].set('upper', '3.14')
        joint_limit[1][0].set('upper', '3.14')
        joint_limit[2][0].set('upper', '3.14')
        joint_limit[3][0].set('upper', '3.14')
        joint_limit[4][0].set('upper', '3.14')
        joint_limit[5][0].set('upper', '3.14')
        joint_limit[6][0].set('upper', '0.2')
        joint_limit[7][0].set('upper', '3.14')
        joint_limit[8][0].set('upper', '3.14')
        joint_limit[9][0].set('upper', '3.14')
        joint_limit[10][0].set('upper', '3.14')
        joint_limit[0][0].set('effort', '100')
        joint_limit[1][0].set('effort', '100')
        joint_limit[2][0].set('effort', '100')
        joint_limit[3][0].set('effort', '100')
        joint_limit[4][0].set('effort', '100')
        joint_limit[5][0].set('effort', '100')
        joint_limit[6][0].set('effort', '100')
        joint_limit[7][0].set('effort', '100')
        joint_limit[8][0].set('effort', '100')
        joint_limit[9][0].set('effort', '100')
        joint_limit[10][0].set('effort', '100')
        joint_limit[0][0].set('velocity', '100')
        joint_limit[1][0].set('velocity', '100')
        joint_limit[2][0].set('velocity', '100')
        joint_limit[3][0].set('velocity', '100')
        joint_limit[4][0].set('velocity', '100')
        joint_limit[5][0].set('velocity', '100')
        joint_limit[6][0].set('velocity', '100')
        joint_limit[7][0].set('velocity', '100')
        joint_limit[8][0].set('velocity', '100')
        joint_limit[9][0].set('velocity', '100')
        joint_limit[10][0].set('velocity', '100')

        tree.write('/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/ddd/000PSM_10.SLDASM/urdf/modified.urdf')
        tree.write('/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/ddd/000PSM_10.SLDASM/urdf/test.xml')

class DHParameter :
    
    # def __init__(self, getPosOrn):
    #     self.getPosOrn = getPosOrn
    
    def getPosOrn(self):
        tree = ET.parse('/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/ddd/000PSM_10.SLDASM/urdf/test.xml')
        root = tree.getroot()
        elements = root.findall('joint')
        joint_origin_list = [elements[i].findall('origin') for i in range(11)]
        
        numJoints = 11
        joint_pos = [0] * numJoints
        joint_orn = [0] * numJoints
        for i in range(numJoints):
            joint_pos[i] = joint_origin_list[i][0].get('xyz')
            joint_orn[i] = joint_origin_list[i][0].get('rpy')

        joint_pos_str = [''] * 3
        joint_pos_float = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
                           [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
                           [0, 0, 0], [0, 0, 0], [0, 0, 0]]

        for i in range(numJoints):
            for j in range(3):
                for item in joint_pos[i]:
                    if item.isspace():
                        joint_pos[i] = joint_pos[i][1:]
                        break
                    else:
                        joint_pos_str[j] = joint_pos_str[j] + item
                        joint_pos[i] = joint_pos[i][1:]
                joint_pos_float[i][j] = float(joint_pos_str[j])
                joint_pos_str = [''] * 3
        joint_pos_float_np = np.array(joint_pos_float)
        print('*' *20)
        print('joint_pos_float= ', )
        pprint(joint_pos_float_np)

        joint_orn_str = [''] * 3
        joint_orn_float = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
                           [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
                           [0, 0, 0], [0, 0, 0], [0, 0, 0]]

        for i in range(numJoints):
            for j in range(3):
                for item in joint_orn[i]:
                    if item.isspace():
                        joint_orn[i] = joint_orn[i][1:]
                        break
                    else:
                        joint_orn_str[j] = joint_orn_str[j] + item
                        joint_orn[i] = joint_orn[i][1:]
                joint_orn_float[i][j] = float(joint_orn_str[j])
                joint_orn_str = [''] * 3
        joint_orn_float_np = np.array(joint_orn_float)
        print('*' *20)
        print('joint_orn_float= ')
        pprint(joint_orn_float_np)
        return joint_pos_float_np, joint_orn_float_np

    def DH_compute(self, joint_positions=np.zeros((11)), end_pos=np.zeros((3)), end_orn=np.zeros((3)), joint_pos_err=np.zeros((11, 3)), joint_orn_err=np.zeros((11, 3))):
        base_pos = np.append(end_pos,1)
        base_orn = np.append(end_orn,1)
        cr = np.cos(end_orn[0]);
        cp = np.cos(end_orn[1]);
        cy = np.cos(end_orn[2]);
        sr = np.sin(end_orn[0]);
        sp = np.sin(end_orn[1]);
        sy = np.sin(end_orn[2]);
        base_pos_orn = np.array([[cp*cy, cy*sr*sp - cr*sy, sr*sy + cr*cy*sp, end_pos[0]],
                                 [cp*sy, cr*cy + sr*sp*sy, cr*sp*sy - cy*sr, end_pos[1]],
                                 [-sp,   cp*sr,            cr*cp,            end_pos[2]],
                                 [0,     0,                0,                1         ]])
        # numJoints = 11
        numJoints = 8
        pos = [[0.00000, 0.00000, 0.27985],
               [0.00000, 0.00000, 0.00000],
               [0.00000, -0.36330, 0.00000],
               [-0.04951, 0.00000, 0.00000],
               [0.04951, 0.36665, 0.00000],
               [0.00000, 0.00000, 0.00000],
               [0.00000, 0.00000, 0.00000],
               [-0.04050, 0.00000, 0.55443-0.16],
               [0.00000, 0.00000, 0.16],
               [0.01125, 0.00000, 0.00000],
               [0.00000, 0.00000, 0.00000]]

        orn = [[ 0.00000, 0.00000, 0.00000],
               [-1.57078, 0.00000, 0.00000],
               [ 1.57079, 0.00000, 0.00000],
               [ 1.57079, 0.00000, 0.00000],
               [-1.57078, 0.00000, 0.00000],
               [-1.57079, 0.00000, 1.57079],
               [-1.57080, 0.00000, 0.00000],
               [ 0.00000, 0.00000, 0.00000],
               [ 1.57080, 0.00000, 1.57079],
               [ 1.57080, 0.00000, 0.00000],
               [ 0.00000, 0.00000, 0.00000]]

        Td = np.zeros((numJoints, 4, 4))
        Tx = np.zeros((numJoints, 4, 4))
        Ty = np.zeros((numJoints, 4, 4))
        Tz = np.zeros((numJoints, 4, 4))
        T_dot = np.eye(4)
        origin_point = np.eye(4, 4)
        # T_joint = np.array([0., 0., 0.,1.]*(numJoints+1))
        # T_joint.resize(12,4)
        T_joint = np.zeros((numJoints+1, 4, 4))
        point_joint_pos = np.zeros((numJoints+1, 3))
        point_joint_orn = np.zeros((numJoints+1, 3, 3))
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
                # Td[i] = np.array([[1, 0, 0, pos[i][0] + joint_pos_err[i][0]/1000],
                #                   [0, 1, 0, pos[i][1] + joint_pos_err[i][1]/1000],
                #                   [0, 0, 1, pos[i][2] + joint_pos_err[i][2]/1000],
                #                   [0, 0, 0, 1        ]])
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
            if i < numJoints-1:
                T_joint[i+1] = T_dot@origin_point
            else:
              T_joint[i+1] = T_dot@base_pos_orn
            point_joint_pos[i+1] = T_joint[i+1][0:3, 3]
            point_joint_orn[i+1] = T_joint[i+1][0:3, 0:3]
            # p.addUserDebugLine(point_joint_pos[i], point_joint_pos[i+1], lineColorRGB=[0,0,1], lineWidth=5)
        # p.addUserDebugPoints(pointPositions=[point_joint_pos[numJoints]], pointColorsRGB=[[1,0,1]], pointSize=6)
        # p.addUserDebugPoints(pointPositions=point_joint_pos, pointColorsRGB=[[1,0,1]]*12, pointSize=6)
        # return point_joint_pos[numJoints], point_joint_orn[numJoints]
        return T_joint[i+1]

    def func_dh(self, joint_positions=np.zeros((11)), end_pos=np.zeros((3)), end_orn=np.zeros((3)), joint_pos_err=np.zeros((11, 3)), joint_orn_err=np.zeros((11, 3))):
        theta_rol = joint_positions.copy()
        base_pos = np.append(end_pos,1).reshape(4,1)
        numJoints = 8
        alpha = [0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0, 0, np.pi/2, np.pi/2, 0]
        A = [0, 0, 0, -0.04951, 0.04951, 0, 0, -0.04050, 0, 0.01125, 0]
        # theta_rol = np.array([0., 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        # theta_rol = np.array([0, 0.5, -0.5, 0.5, -0.5, -0.5, 0.1, 0.5, 0.5, 0.5, 0.5])
        theta_rol[5] = theta_rol[5] + np.pi/2
        theta_rol[8] = theta_rol[8] + np.pi/2
        L = [0.27985, 0, 0.36330, 0, 0.36665, 0, 0, 0.55443-0.16, 0.16, 0, 0]
        T_dot = []
        T_simplify = []
        # T_dot = np.zeros((numJoints, 4, 4))
        T_joint = []
        # T_joint.resize(12,4)
        point_joint = np.zeros((numJoints+1, 3))
        theta = sympy.symbols("theta1:12")
        # theta1 = sympy.symbols("theta1")
        # theta2 = sympy.symbols("theta2")
        # theta3 = sympy.symbols("theta3")
        # theta4 = sympy.symbols("theta4")
        # theta5 = sympy.symbols("theta5")
        # theta6 = sympy.symbols("theta6")
        # theta7 = sympy.symbols("theta7")
        # theta8 = sympy.symbols("theta8")
        # theta9 = sympy.symbols("theta9")
        # theta10 = sympy.symbols("theta10")
        # theta11 = sympy.symbols("theta11")
        T = []

        for i in range(numJoints):
            if i != 6:
                T_NumJoint = sympy.Matrix([
                    [sympy.cos(theta[i]),             -sympy.sin(theta[i]),               0,             A[i]],
                    [np.cos(alpha[i])*sympy.sin(theta[i]), sympy.cos(theta[i])*np.cos(alpha[i]), -np.sin(alpha[i]), -np.sin(alpha[i])*L[i]],
                    [sympy.sin(theta[i])*np.sin(alpha[i]), sympy.cos(theta[i])*np.sin(alpha[i]),  np.cos(alpha[i]),  np.cos(alpha[i])*L[i]],
                    [0,                                0,                                 0,             1]])
            else:
                T_NumJoint = sympy.Matrix([
                    [1.,  0., 0., 0.],
                    [0.,  0., 1., (theta[i])],
                    [0., -1., 0., 0.],
                    [0.,  0., 0., 1.]])
            T.append(T_NumJoint)
            if i ==0 :
                T_dot.append(T_NumJoint)
            else:
                T_dot.append(T_dot[i-1] * T_NumJoint)
            T_simplify.append(T_dot[i].xreplace({n : round(n, 6) for n in T_dot[i].atoms(sympy.Number)}))
            # T_simplify.append(T_dot[i])
            T_joint.append(T_simplify[i] * base_pos)
            # T_joint.append(T_simplify[i][ : ,0:3])
            # np_joint = T_joint[i].subs([(theta1,0),(theta2,0),(theta3,3)])[0:3].T
            np_joint = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6'
                                      ,'theta7','theta8','theta9','theta10','theta11'),T_joint[i],"numpy")
            point_joint[i+1] = np_joint(theta_rol[0],theta_rol[1],theta_rol[2],theta_rol[3],theta_rol[4]
                                        ,theta_rol[5],theta_rol[6],theta_rol[7],theta_rol[8],theta_rol[9]
                                        ,theta_rol[10])[0:3].T
            # p.addUserDebugLine(point_joint[i], point_joint[i+1], lineColorRGB=[0,0,1], lineWidth=5)
        # p.addUserDebugPoints(pointPositions=[point_joint[numJoints]], pointColorsRGB=[[1,0.5,0.3]], pointSize=12)
        
        return T_simplify[numJoints-1]
    
    def func_dh_all(self, joint_positions=np.zeros((11)), end_pos=np.zeros((3)), end_orn=np.zeros((3)), joint_pos_err=np.zeros((11, 3)), joint_orn_err=np.zeros((11, 3))):
        theta_rol = joint_positions.copy()
        base_pos = np.append(end_pos,1).reshape(4,1)
        numJoints = 8
        alpha = [0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0, 0, np.pi/2, np.pi/2, 0]
        A = [0, 0, 0, -0.04951, 0.04951, 0, 0, -0.04050, 0, 0.01125, 0]
        # theta_rol = np.array([0., 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        # theta_rol = np.array([0, 0.5, -0.5, 0.5, -0.5, -0.5, 0.1, 0.5, 0.5, 0.5, 0.5])
        theta_rol[5] = theta_rol[5] + np.pi/2
        theta_rol[8] = theta_rol[8] + np.pi/2
        L = [0.27985, 0, 0.36330, 0, 0.36665, 0, 0, 0.55443-0.16, 0.16, 0, 0]
        T_dot = []
        T_simplify = []
        # T_dot = np.zeros((numJoints, 4, 4))
        T_joint = []
        # T_joint.resize(12,4)
        point_joint = np.zeros((numJoints+1, 3))
        theta = sympy.symbols("theta1:12")
        # theta1 = sympy.symbols("theta1")
        # theta2 = sympy.symbols("theta2")
        # theta3 = sympy.symbols("theta3")
        # theta4 = sympy.symbols("theta4")
        # theta5 = sympy.symbols("theta5")
        # theta6 = sympy.symbols("theta6")
        # theta7 = sympy.symbols("theta7")
        # theta8 = sympy.symbols("theta8")
        # theta9 = sympy.symbols("theta9")
        # theta10 = sympy.symbols("theta10")
        # theta11 = sympy.symbols("theta11")
        T = []

        for i in range(numJoints):
            if i != 6:
                T_NumJoint = sympy.Matrix([
                    [sympy.cos(theta[i]),             -sympy.sin(theta[i]),               0,             A[i]],
                    [np.cos(alpha[i])*sympy.sin(theta[i]), sympy.cos(theta[i])*np.cos(alpha[i]), -np.sin(alpha[i]), -np.sin(alpha[i])*L[i]],
                    [sympy.sin(theta[i])*np.sin(alpha[i]), sympy.cos(theta[i])*np.sin(alpha[i]),  np.cos(alpha[i]),  np.cos(alpha[i])*L[i]],
                    [0,                                0,                                 0,             1]])
            else:
                T_NumJoint = sympy.Matrix([
                    [1.,  0., 0., 0.],
                    [0.,  0., 1., (theta[i])],
                    [0., -1., 0., 0.],
                    [0.,  0., 0., 1.]])
            T.append(T_NumJoint)
            if i ==0 :
                T_dot.append(T_NumJoint)
            else:
                T_dot.append(T_dot[i-1] * T_NumJoint)
            T_simplify.append(T_dot[i].xreplace({n : round(n, 6) for n in T_dot[i].atoms(sympy.Number)}))
            # T_simplify.append(T_dot[i])
            T_joint.append(T_simplify[i] * base_pos)
            # T_joint.append(T_simplify[i][ : ,0:3])
            # np_joint = T_joint[i].subs([(theta1,0),(theta2,0),(theta3,3)])[0:3].T
            np_joint = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6'
                                      ,'theta7','theta8','theta9','theta10','theta11'),T_joint[i],"numpy")
            point_joint[i+1] = np_joint(theta_rol[0],theta_rol[1],theta_rol[2],theta_rol[3],theta_rol[4]
                                        ,theta_rol[5],theta_rol[6],theta_rol[7],theta_rol[8],theta_rol[9]
                                        ,theta_rol[10])[0:3].T
            # p.addUserDebugLine(point_joint[i], point_joint[i+1], lineColorRGB=[0,0,1], lineWidth=5)
        # p.addUserDebugPoints(pointPositions=[point_joint[numJoints]], pointColorsRGB=[[1,0.5,0.3]], pointSize=12)
        
        return T_simplify
    
    def func_dh_i(self, joint_positions=np.zeros((11)), end_pos=np.zeros((3)), end_orn=np.zeros((3)), joint_pos_err=np.zeros((11, 3)), joint_orn_err=np.zeros((11, 3))):
        theta_rol = joint_positions.copy()
        base_pos = np.append(end_pos,1).reshape(4,1)
        numJoints = 8
        alpha = [0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0, 0, np.pi/2, np.pi/2, 0]
        A = [0, 0, 0, -0.04951, 0.04951, 0, 0, -0.04050, 0, 0.01125, 0]
        # theta_rol = np.array([0., 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        # theta_rol = np.array([0, 0.5, -0.5, 0.5, -0.5, -0.5, 0.1, 0.5, 0.5, 0.5, 0.5])
        theta_rol[5] = theta_rol[5] + np.pi/2
        theta_rol[8] = theta_rol[8] + np.pi/2
        L = [0.27985, 0, 0.36330, 0, 0.36665, 0, 0, 0.55443-0.16, 0.16, 0, 0]
        T_dot = []
        T_simplify = []
        # T_dot = np.zeros((numJoints, 4, 4))
        T_joint = []
        T_i = []
        # T_joint.resize(12,4)
        point_joint = np.zeros((numJoints+1, 3))
        theta = sympy.symbols("theta1:12")
        # theta1 = sympy.symbols("theta1")
        # theta2 = sympy.symbols("theta2")
        # theta3 = sympy.symbols("theta3")
        # theta4 = sympy.symbols("theta4")
        # theta5 = sympy.symbols("theta5")
        # theta6 = sympy.symbols("theta6")
        # theta7 = sympy.symbols("theta7")
        # theta8 = sympy.symbols("theta8")
        # theta9 = sympy.symbols("theta9")
        # theta10 = sympy.symbols("theta10")
        # theta11 = sympy.symbols("theta11")
        T = []

        for i in range(numJoints):
            if i != 6:
                T_NumJoint = sympy.Matrix([
                    [sympy.cos(theta[i]),             -sympy.sin(theta[i]),               0,             A[i]],
                    [np.cos(alpha[i])*sympy.sin(theta[i]), sympy.cos(theta[i])*np.cos(alpha[i]), -np.sin(alpha[i]), -np.sin(alpha[i])*L[i]],
                    [sympy.sin(theta[i])*np.sin(alpha[i]), sympy.cos(theta[i])*np.sin(alpha[i]),  np.cos(alpha[i]),  np.cos(alpha[i])*L[i]],
                    [0,                                0,                                 0,             1]])
            else:
                T_NumJoint = sympy.Matrix([
                    [1.,  0., 0., 0.],
                    [0.,  0., 1., (theta[i])],
                    [0., -1., 0., 0.],
                    [0.,  0., 0., 1.]])
            T.append(T_NumJoint)
            T_i.append(T[i].xreplace({n : round(n, 6) for n in T[i].atoms(sympy.Number)}))
            if i ==0 :
                T_dot.append(T_NumJoint)
            else:
                T_dot.append(T_dot[i-1] * T_NumJoint)
            T_simplify.append(T_dot[i].xreplace({n : round(n, 6) for n in T_dot[i].atoms(sympy.Number)}))
            T_joint.append(T_simplify[i] * base_pos)
            # T_joint.append(T_simplify[i][ : ,0:3])
            # np_joint = T_joint[i].subs([(theta1,0),(theta2,0),(theta3,3)])[0:3].T
            np_joint = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6'
                                      ,'theta7','theta8','theta9','theta10','theta11'),T_joint[i],"numpy")
            point_joint[i+1] = np_joint(theta_rol[0],theta_rol[1],theta_rol[2],theta_rol[3],theta_rol[4]
                                        ,theta_rol[5],theta_rol[6],theta_rol[7],theta_rol[8],theta_rol[9]
                                        ,theta_rol[10])[0:3].T
            # p.addUserDebugLine(point_joint[i], point_joint[i+1], lineColorRGB=[0,0,1], lineWidth=5)
        # p.addUserDebugPoints(pointPositions=[point_joint[numJoints]], pointColorsRGB=[[1,0.5,0.3]], pointSize=12)
        
        return T_i
    
    # def T_joint(self, theta=np.zeros(11)):
    #     T_joint[0] = np.array((1.0*(((-1.0*sin(theta1)*sin(theta3) + cos(theta1)*cos(theta2)*cos(theta3))*cos(theta4) + 1.0*sin(theta2)*sin(theta4)*cos(theta1))*cos(theta5) - 1.0*(1.0*sin(theta1)*cos(theta3) + 1.0*sin(theta3)*cos(theta1)*cos(theta2))*sin(theta5))*cos(theta6) - 1.0*(-1.0*(-1.0*sin(theta1)*sin(theta3) + cos(theta1)*cos(theta2)*cos(theta3))*sin(theta4) + 1.0*sin(theta2)*cos(theta1)*cos(theta4))*sin(theta6))*cos(theta8) + 1.0*(1.0*((-1.0*sin(theta1)*sin(theta3) + cos(theta1)*cos(theta2)*cos(theta3))*cos(theta4) + 1.0*sin(theta2)*sin(theta4)*cos(theta1))*sin(theta5) + 1.0*(1.0*sin(theta1)*cos(theta3) + 1.0*sin(theta3)*cos(theta1)*cos(theta2))*cos(theta5))*sin(theta8))
    #     T_joint[1] = np.array()
    #     T_joint[2] = np.array()
    #     T_joint[3] = np.array()
    #     T_joint[4] = np.array()
    #     T_joint[5] = np.array()
    #     T_joint[6] = np.array()
    #     T_joint[7] = np.array()
    #     T_joint[8] = np.array()
    #     T_joint[9] = np.array()
    #     T_joint[10] = np.array()
    #     T_joint[11] = np.array()
    #     T_joint[12] = np.array(0)
    #     T_joint[13] = np.array(0)
    #     T_joint[14] = np.array(0)
    #     T_joint[15] = np.array(1)
    


class robot_control :
    
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
class CameraOperate :
    
    def __init__(self, robot_id : int, width : int = 224, height : int = 224, physicsClientId : int = 0):
        self.robot_id = robot_id
        self.width = width
        self.height = height
        self.physicsClientId = physicsClientId
        
    # def log_video(self, task_name):
    #     """
    #     Logs video of each task being executed
    #     """
    #     # if not os.path.exists("video_logs/"):
    #     #     os.makedirs("video_logs")
    #     try:
    #         p.stopStateLogging(self.curr_recording)
    #         self.video_log_key += 1
    #     except Exception:
    #         print("No Video Currently Being Logged")
    #     self.curr_recording = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,
    #                                               "video_logs/task_vid_" +
    #                                               str(task_name) + "_" + str(self.video_log_key) + ".mp4") 
        
    def setCameraPicAndGetPic(self):
        """
        给合成摄像头设置图像并返回robot_id对应的图像
        摄像头的位置为miniBox前头的位置
        """
        basePos, baseOrientation = p.getBasePositionAndOrientation(self.robot_id, physicsClientId=self.physicsClientId)
        # 从四元数中获取变换矩阵，从中获知指向(左乘(1,0,0)，因为在原本的坐标系内，摄像机的朝向为(1,0,0))
        matrix = p.getMatrixFromQuaternion(baseOrientation, physicsClientId=self.physicsClientId)
        tx_vec = np.array([matrix[0], matrix[3], matrix[6]])
        tz_vec = np.array([matrix[2], matrix[5], matrix[8]])
        
        basePos = np.array(basePos)
        # 摄像头的位置
        # BASE_RADIUS 为 0.5，是机器人底盘的半径。BASE_THICKNESS 为 0.2 是机器人底盘的厚度。
        BASE_RADIUS = 0.5
        BASE_THICKNESS = 0.1
        cameraPos = basePos + BASE_RADIUS * tx_vec + 0.5 * BASE_THICKNESS * tz_vec
        targetPos = cameraPos + 1 * tx_vec
        
        viewMatrix = p.computeViewMatrix(
            cameraEyePosition=cameraPos,
            cameraTargetPosition=targetPos,
            cameraUpVector=tz_vec,
            physicsClientId=self.physicsClientId
        )
        
        # viewMatrix = self._p.computeViewMatrixFromYawPitchRoll(
        #     cameraTargetPosition=base_pos,
        #     distance=self._cam_dist,
        #     yaw=self._cam_yaw,
        #     pitch=self._cam_pitch,
        #     roll=0,
        #     upAxisIndex=2)
        
        # viewMatrix = p.computeViewMatrix(
        #     [0, 0, -1], [0, 0, 0], [0, -1, 0], physicsClientId=self.physicsClientId
        # )
        # projectionMatrix = p.computeProjectionMatrixFOV(
        #     20, 1, 0.05, 2, physicsClientId=self.physicsClientId
        # )
        
        projectionMatrix = p.computeProjectionMatrixFOV(
            fov=50.0,
            aspect=1.0,
            nearVal=0.01,
            farVal=20,
            physicsClientId=self.physicsClientId
        )
        
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
            width=self.width, height=self.height,
            viewMatrix=viewMatrix,
            # projectionMatrix=projectionMatrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
            physicsClientId=self.physicsClientId
        )
        
        return width, height, rgbImg, depthImg, segImg

def main():
    # 创建 Thread 实例
    event = Event()
    print_text_queue = Queue(50)
    print_content_queue = Queue(50)
    # create a thread 
    a = 1700
#    thread = Thread(target=task, args=(event,b))
    thread = Thread_print(event, print_text_queue, print_content_queue, run_time=50)
    # thread = Thread_print(event, print_text_queue=print_text_queue, run_time=10)
    # start the new thread
    thread.start()
    # print(time())
    sleep(3)
    for i in range(5):
        a += 100
        if not print_content_queue.full():
            print_content_queue.put(a)
        else:
            print('queue is full')
#        print_text_queue.put(b)
        sleep(2)
        print_text_queue.put('test')
    print(time())
    # block for a while
    #sleep(5)
    # stop the worker thread
    print('Main stopping thread')
    event.set()
    # 这里是为了演示，实际开发时，主进程有事件循环，耗时函数不需要调用join()方法
    thread.join()
    print("end")

    # numJoints = 11
    # joint_positions = [0]*11
    # T_dot = np.eye(4)
    # T_joint = np.array([0., 0., 0.,1.]*(numJoints+1))
    # T_joint.resize(12,4)
    # for num in range(240*2):
    #     T_dot = np.eye(4)
    #     for i in range(numJoints):
    #         joint_positions[10] = 8 * np.sin(2 * np.pi * 1 * num / 240)
    #         # joint_positions[0] = math.pi/3
    #         orn_cos_z[i] = np.cos(orn[i][2]+joint_positions[i])
    #         orn_sin_z[i] = np.sin(orn[i][2]+joint_positions[i])
    #         Tz[i] = np.array([[orn_cos_z[i], orn_sin_z[i]*-1, 0, 0],
    #                           [orn_sin_z[i], orn_cos_z[i],    0, 0],
    #                           [0,            0,               1, 0],
    #                           [0,            0,               0, 1]])
    #         T_dot = T_dot@Td[i]@Tx[i]@Tz[i]
    #         T_joint[i+1] = T_dot@base_pos
    #         point_joint[i+1] = T_joint[i+1][0:3]
    #         # p.addUserDebugLine(point_joint[i], point_joint[i+1], lineColorRGB=[0,0,1], lineWidth=2)
    #     p.addUserDebugPoints(pointPositions=[point_joint[i+1]], pointColorsRGB=[[1,0,1]], pointSize=6)
    #     sleep(1./240.)

if __name__ == '__main__':
    main()













