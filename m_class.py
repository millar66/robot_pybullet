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
                
class CameraOperate :
    
    def __init__(self, robot_id : int, width : int = 224, height : int = 224, physicsClientId : int = 0):
        self.robot_id = robot_id
        self.width = width
        self.height = height
        self.physicsClientId = physicsClientId
        
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

if __name__ == '__main__':
    main()













