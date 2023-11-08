#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov  6 20:23:20 2023

@author: lihui.liu
"""
# %%
import pybullet as p
import time
import pybullet_data
import numpy as np
import datetime
import math

# %%
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,-9.81)
# p.setGravity(0,0,0)

cylinder_Obstacle_1_StartPos = [-0.4, 0.3, 0.2 ]#圆柱体障碍物的初始位置
cylinder_Obstacle_2_StartPos = [ 0.0,-0.6, 0.15]
cylinder_Obstacle_3_StartPos = [ 0.6,-0.6, 0.1 ]
cylinder_Obstacle_4_StartPos = [ 0.8, 0.2, 0.1]
cylinder_target_StartPos = [ 0, 0.0, 0.3]
cylinder_robot_StartPos = [ -0.8, 0.0, 0.3]

#（圆柱体障碍物）这里的参数会转换成一个四元数，可以理解成能够控制模型在空间中绕x，y，z轴旋转的参数。
#（参数是角度。e.g. [3.14,0,0] == [pai,0,0];[1.57,0,0] == [pai/2,0,0]。参数的正负表示旋转的不同方向。）
cylinder_Obstacle_1_StartOrientation = p.getQuaternionFromEuler([0,0,0])
cylinder_Obstacle_2_StartOrientation = p.getQuaternionFromEuler([0,0,0])
cylinder_Obstacle_3_StartOrientation = p.getQuaternionFromEuler([0,0,0])
cylinder_Obstacle_4_StartOrientation = p.getQuaternionFromEuler([0,0,0])
cylinder_target_StartOrientation = p.getQuaternionFromEuler([0,0,0])
cylinder_robot_StartOrientation = p.getQuaternionFromEuler([0,0,0])

cylinder_Obstacle_1_Id = p.loadURDF("mymodel/cylinder_obstacle1.urdf",cylinder_Obstacle_1_StartPos,cylinder_Obstacle_1_StartOrientation)
cylinder_Obstacle_2_Id = p.loadURDF("mymodel/cylinder_obstacle2.urdf",cylinder_Obstacle_2_StartPos,cylinder_Obstacle_2_StartOrientation)
cylinder_Obstacle_3_Id = p.loadURDF("mymodel/cylinder_obstacle3.urdf",cylinder_Obstacle_3_StartPos,cylinder_Obstacle_3_StartOrientation)
cylinder_Obstacle_4_Id = p.loadURDF("mymodel/cylinder_obstacle4.urdf",cylinder_Obstacle_4_StartPos,cylinder_Obstacle_4_StartOrientation)
cylinder_target_Id = p.loadURDF("mymodel/cylinder_target.urdf",cylinder_target_StartPos,cylinder_target_StartOrientation)
cylinder_robot_Id = p.loadURDF("mymodel/cylinder_robot.urdf",cylinder_robot_StartPos,cylinder_robot_StartOrientation)

robot_shape = p.getVisualShapeData(cylinder_robot_Id)#得到有关模型形状的一些列信息
target_shape = p.getVisualShapeData(cylinder_target_Id)
robot_R = robot_shape[0][3][1]#半径
robot_H = robot_shape[0][3][0]#高度
target_R = target_shape[0][3][1]
target_H = target_shape[0][3][0]

path_string = input("Please input the path:")
path_string = path_string.replace('[','')
path_string = path_string.replace(']','')
path_string = path_string.split(',')
length = len(path_string)
little_length = length/3
path = []
little_length = int(little_length)
for i in range(little_length):
    path.append([])
    for j in range(3):
        path[i].append(0.4)
path_flag = 0
for i in range(little_length):
    for j in range(3):
        path[i][j] = float(path_string[path_flag])
        path_flag = path_flag + 1
#path = [[0.4,0.0,0.3],[0.6,0.0,0.3],[0.8,0.0,0.3],[1.0,0.0,0.3],[1.2,0.0,0.3]]

# %%
# 主函数
# [[0.4,0.0,0.3],[0.6,0.0,0.3],[0.8,0.0,0.3],[1.0,0.0,0.3],[1.2,0.0,0.3]]
# [[0.4,1.0,0.3],[0.6,-1.0,0.3],[0.8,2.0,0.3],[-1.0,0.0,0.3],[-1.2,-2.0,0.3]]
cylinder_Obstacle_text = p.addUserDebugText(text="p1", textPosition=path[0], textColorRGB=[0, 1, 0], textSize=1.2)
cylinder_Obstacle_text = p.addUserDebugText(text="p2", textPosition=path[1], textColorRGB=[0, 1, 0], textSize=1.2)
cylinder_Obstacle_text = p.addUserDebugText(text="p3", textPosition=path[2], textColorRGB=[0, 1, 0], textSize=1.2)
cylinder_Obstacle_text = p.addUserDebugText(text="p4", textPosition=path[3], textColorRGB=[0, 1, 0], textSize=1.2)
cylinder_Obstacle_text = p.addUserDebugText(text="p5", textPosition=path[4], textColorRGB=[0, 1, 0], textSize=1.2)
for i in range(len(path)):
    print("robot go to the",i+1,"point")
    cylinder_target_data = p.getBasePositionAndOrientation(cylinder_target_Id)
    cylinder_robot_data = p.getBasePositionAndOrientation(cylinder_robot_Id)
    robot_x = cylinder_robot_data[0][0]
    robot_y = cylinder_robot_data[0][1]
    path_x = path[i][0]
    path_y = path[i][1]
    target_x = cylinder_target_data[0][0]
    target_y = cylinder_target_data[0][1]
    
    # cylinder_Obstacle_1_data = p.getBasePositionAndOrientation(cylinder_Obstacle_1_Id)
    # cylinder_Obstacle_2_data = p.getBasePositionAndOrientation(cylinder_Obstacle_2_Id)
    # cylinder_Obstacle_3_data = p.getBasePositionAndOrientation(cylinder_Obstacle_3_Id)
    # cylinder_Obstacle_4_data = p.getBasePositionAndOrientation(cylinder_Obstacle_4_Id)
    # cylinder_Obstacle_1_x = cylinder_Obstacle_1_data[0][0]
    # cylinder_Obstacle_1_y = cylinder_Obstacle_1_data[0][1]
    # cylinder_Obstacle_2_x = cylinder_Obstacle_2_data[0][0]
    # cylinder_Obstacle_2_y = cylinder_Obstacle_2_data[0][1]
    # cylinder_Obstacle_3_x = cylinder_Obstacle_3_data[0][0]
    # cylinder_Obstacle_3_y = cylinder_Obstacle_3_data[0][1]
    # cylinder_Obstacle_4_x = cylinder_Obstacle_4_data[0][0]
    # cylinder_Obstacle_4_y = cylinder_Obstacle_4_data[0][1]
    # cylinder_Obstacle_text = p.addUserDebugText(text="1", textPosition=[cylinder_Obstacle_1_x, cylinder_Obstacle_1_y, 0], textColorRGB=[0, 1, 0], textSize=1.2)
    # cylinder_Obstacle_text = p.addUserDebugText(text="2", textPosition=[cylinder_Obstacle_2_x, cylinder_Obstacle_2_y, 0], textColorRGB=[0, 1, 0], textSize=1.2)
    # cylinder_Obstacle_text = p.addUserDebugText(text="3", textPosition=[cylinder_Obstacle_3_x, cylinder_Obstacle_3_y, 0], textColorRGB=[0, 1, 0], textSize=1.2)
    # cylinder_Obstacle_text = p.addUserDebugText(text="4", textPosition=[cylinder_Obstacle_4_x, cylinder_Obstacle_4_y, 0], textColorRGB=[0, 1, 0], textSize=1.2)
    
    Scalar_Product_1 = (path_x - robot_x)*(target_x - robot_x) + (path_y - robot_y)*(target_y - robot_y)#向量的点乘
    Scalar_Product_2 = (target_x - path_x)*(robot_x - path_x) + (target_y - path_y)*(robot_y - path_y)
    print("*********************************************")
    #点乘是锐角或者直角
    if Scalar_Product_1 >= 0 and Scalar_Product_2 > 0:
        the_first_situation(cylinder_target_Id,cylinder_robot_Id,robot_x,robot_y,path_x,path_y,target_x,target_y)
        
    #点乘是钝角
    elif Scalar_Product_1 < 0 and Scalar_Product_2 > 0:
        the_second_situation(cylinder_target_Id,cylinder_robot_Id,robot_x,robot_y,path_x,path_y,target_x,target_y)
        
        
    elif Scalar_Product_2 == 0:
        the_third_situation(cylinder_target_Id,cylinder_robot_Id,robot_x,robot_y,path_x,path_y,target_x,target_y)
        
    else:
        the_fourth_situation(cylinder_target_Id,cylinder_robot_Id,robot_x,robot_y,path_x,path_y,target_x,target_y)

# %%
p.disconnect()

# %%
# 求两直线交点
def cross_point(line1,line2):#计算交点函数
    x1=line1[0]#取四点坐标
    y1=line1[1]
    x2=line1[2]
    y2=line1[3]
    x3=line2[0]
    y3=line2[1]
    x4=line2[2]
    y4=line2[3]
    if (x4-x3)==0:#L2直线斜率不存在操作
        k2=None
        b2=0
        x=x3
        k1=(y2-y1)*1.0/(x2-x1)#计算k1,由于点均为整数，需要进行浮点数转化
        b1=y1*1.0-x1*k1*1.0#整型转浮点型是关键
        y=k1*x*1.0+b1*1.0
    elif (x2-x1)==0:#L1直线斜率不存在操作
        k1=None
        b1=0
        x=x1
        k2=(y4-y3)*1.0/(x4-x3)
        b2=y3*1.0-x3*k2*1.0
        y=k2*x*1.0+b2*1.0
    else:
        k1=(y2-y1)*1.0/(x2-x1)#计算k1,由于点均为整数，需要进行浮点数转化
        k2=(y4-y3)*1.0/(x4-x3)#斜率存在操作
        b1=y1*1.0-x1*k1*1.0#整型转浮点型是关键
        b2=y3*1.0-x3*k2*1.0
        x=(b2-b1)*1.0/(k1-k2)
        y=k1*x*1.0+b1*1.0    
    return [x,y]

# 计算垂心
def orthocenter_point(robot_x,robot_y,path_x,path_y,target_x,target_y):
    if (path_y - robot_y) == 0 and (path_x - robot_x) != 0: #水平
        flag = 1
        orthocenter_x = target_x
        orthocenter_y = robot_y
    elif (path_y - robot_y) != 0 and (path_x - robot_x) == 0: #竖直
        flag = 2
        orthocenter_x = robot_x
        orthocenter_y = target_y
    else:
        flag = 3
        k1 = (path_y - robot_y)/(path_x - robot_x)
        k4 = -(1/k1)
        b4 = target_y - k4 * target_x
        point_fake_x = target_x + 1
        point_fake_y = k4 * point_fake_x + b4
        line1 = [path_x,path_y,robot_x,robot_y]
        line4 = [target_x,target_y,point_fake_x,point_fake_y]
        orthocenter = cross_point(line1,line4)
        orthocenter_x = orthocenter[0]
        orthocenter_y = orthocenter[1]
    return [orthocenter_x,orthocenter_y,flag]

# 计算移动的方向
def move_direction(flag,robot_x,robot_y,path_x,path_y):
    if flag == 1:#水平
        direction_x = (robot_x - path_x)/abs((robot_x - path_x))
        direction_y = 0
        cosx = 1
        sinx = 0
    elif flag == 2:#竖直
        direction_x = 0
        direction_y = (robot_y - path_y)/abs((robot_y - path_y))
        cosx = 0
        sinx = 1
    else:
        dir_y = path_y - robot_y 
        dir_x = path_x - robot_x
        cosx = abs(dir_x)/(math.sqrt(np.square(dir_x)+np.square(dir_y)))
        sinx = abs(dir_y)/(math.sqrt(np.square(dir_x)+np.square(dir_y)))
        direction_x = (robot_x - path_x)/abs((robot_x - path_x))
        direction_y = (robot_y - path_y)/abs((robot_y - path_y))
    return [direction_x,direction_y,cosx,sinx]

# 计算向“上”移动的方向
def move_vertically_direction(flag,robot_x,robot_y,target_x,target_y,path_x,path_y):
    if flag == 1:#水平
        fake_point_x = robot_x
        fake_point_y = target_y#只要不是robot_y,其他任意数字都可以
        line2 = [robot_x,robot_y,fake_point_x,fake_point_y]
        line3 = [path_x,path_y,target_x,target_y]
        cross_point_2 = cross_point(line2,line3)
        cross_point_2_x = cross_point_2[0]
        cross_point_2_y = cross_point_2[1]
        dir_x2 = (cross_point_2_x - robot_x)/abs(cross_point_2_x - robot_x)
        dir_y2 = (cross_point_2_y - robot_y)/abs(cross_point_2_y - robot_y)
        cosx2 = 0
        sinx2 = 1*dir_y2    
    elif flag == 2:#竖直
        fake_point_x = target_x#只要不是robot_x,其他任意数字都可以
        fake_point_y = robot_y
        line2 = [robot_x,robot_y,fake_point_x,fake_point_y]
        line3 = [path_x,path_y,target_x,target_y]
        cross_point_2 = cross_point(line2,line3)
        cross_point_2_x = cross_point_2[0]
        cross_point_2_y = cross_point_2[1]
        dir_x2 = (cross_point_2_x - robot_x)/abs(cross_point_2_x - robot_x)
        dir_y2 = (cross_point_2_y - robot_y)/abs(cross_point_2_y - robot_y)
        cosx2 = 1*dir_x2
        sinx2 = 0
    else:
        k1 = (path_y - robot_y)/(path_x - robot_x)
        k2 = -(1/k1) 
        b2 = robot_y - k2 * robot_x
        fake_point_x = robot_x + 1
        fake_point_y = k2 * fake_point_x + b2
        line2 = [robot_x,robot_y,fake_point_x,fake_point_y]
        line3 = [path_x,path_y,target_x,target_y]
        cross_point_2 = cross_point(line2,line3)
        cross_point_2_x = cross_point_2[0]
        cross_point_2_y = cross_point_2[1]
        dir_x2 = cross_point_2_x - robot_x
        dir_y2 = cross_point_2_y - robot_y
        cosx2 = dir_x2/(math.sqrt(np.square(dir_x2)+np.square(dir_y2)))
        sinx2 = dir_y2/(math.sqrt(np.square(dir_x2)+np.square(dir_y2)))
        
    return [cosx2,sinx2,cross_point_2_x,cross_point_2_y]

# first move
def first_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y):
    #print("the first move")
    #垂点距离机器人的距离
    #target and robot are too close together, robot moves separate distance from target
    while 1:
        cylinder_target_data = p.getBasePositionAndOrientation(cylinder_target_Id)
        cylinder_robot_data = p.getBasePositionAndOrientation(cylinder_robot_Id)
        robot_x = cylinder_robot_data[0][0]
        robot_y = cylinder_robot_data[0][1]
        target_x = cylinder_target_data[0][0]
        target_y = cylinder_target_data[0][1]
        orthocenter = orthocenter_point(robot_x,robot_y,path_x,path_y,target_x,target_y)
        orthocenter_x = orthocenter[0]
        orthocenter_y = orthocenter[1]
        flag = orthocenter[2]
        orthocenter_2_robot = math.sqrt(np.square(orthocenter_x - robot_x)+np.square(orthocenter_y - robot_y))
        if orthocenter_2_robot - (robot_R + target_R) > 0.2 and orthocenter_2_robot - (robot_R + target_R) <= 0.3:
            break
        if (orthocenter_2_robot - (robot_R + target_R)) <= 0.2:
            direction_move = move_direction(flag,robot_x,robot_y,path_x,path_y)
            direction_x = direction_move[0]
            direction_y = direction_move[1]
            cosx = direction_move[2]
            sinx = direction_move[3]
        #距离太远
        elif orthocenter_2_robot - (robot_R + target_R) > 0.3:
            direction_move = move_direction(flag,robot_x,robot_y,path_x,path_y)
            direction_x = -direction_move[0]
            direction_y = -direction_move[1]
            cosx = direction_move[2]
            sinx = direction_move[3]
        else:
            direction_x = 0.0
            direction_y = 0.0
            cosx = 0.0
            sinx = 0.0      
        p.resetBaseVelocity(cylinder_robot_Id,[direction_x*cosx/5,direction_y*sinx/5,0])#give the robot a velocity
        # p.resetBaseVelocity(cylinder_robot_Id,[direction_x*cosx*10,direction_y*sinx*10,0])#give the robot a velocity
        p.stepSimulation()
        time.sleep(1./240.)
    orthocenter = orthocenter_point(robot_x,robot_y,path_x,path_y,target_x,target_y)
    flag = orthocenter[2]
    return flag

# second move
def second_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y,flag):
    #print("the second move")
    while 1:
        cylinder_target_data = p.getBasePositionAndOrientation(cylinder_target_Id)
        cylinder_robot_data = p.getBasePositionAndOrientation(cylinder_robot_Id)
        robot_x = cylinder_robot_data[0][0]
        robot_y = cylinder_robot_data[0][1]
        target_x = cylinder_target_data[0][0]
        target_y = cylinder_target_data[0][1]
        vertically_move_direction = move_vertically_direction(flag,robot_x,robot_y,target_x,target_y,path_x,path_y)
        cosx2 = vertically_move_direction[0]
        sinx2 = vertically_move_direction[1]
        cross_point_2_x = vertically_move_direction[2]
        cross_point_2_y = vertically_move_direction[3]
    
        if (math.sqrt(np.square(robot_x - cross_point_2_x)+np.square(robot_y - cross_point_2_y))) <= 0.1:
            break
        
        p.resetBaseVelocity(cylinder_robot_Id,[cosx2/5,sinx2/5,0])
        #p.applyExternalForce(cylinder_robot_Id,-1,[cosx2 * 2,sinx2 * 2,0],[0,0,robot_H/2],p.WORLD_FRAME) #施加力
        p.stepSimulation()
        time.sleep(1./240.)
        flag = first_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y)

    return 1
    
# third move
def third_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y,flag):
    #print("the third move")
    while 1:
        cylinder_target_data = p.getBasePositionAndOrientation(cylinder_target_Id)
        cylinder_robot_data = p.getBasePositionAndOrientation(cylinder_robot_Id)
        robot_x = cylinder_robot_data[0][0]
        robot_y = cylinder_robot_data[0][1]
        target_x = cylinder_target_data[0][0]
        target_y = cylinder_target_data[0][1]
        if flag == 1:
            cosx3 = (path_x - robot_x)/abs((path_x - robot_x))
            sinx3 = 0
        elif flag == 2:
            cosx3 = 0
            sinx3 = (path_y - robot_y)/abs((path_y - robot_y))
        else:
            dir_x = path_x - robot_x
            dir_y = path_y - robot_y
            cosx3 = dir_x/(math.sqrt(np.square(dir_x)+np.square(dir_y)))
            sinx3 = dir_y/(math.sqrt(np.square(dir_x)+np.square(dir_y)))
        if (math.sqrt(np.square(target_x - path_x)+np.square(target_y - path_y))) <= 0.1:
            break
        start_time = datetime.datetime.now()
        while 1:
            end_time = datetime.datetime.now()
            interval = (end_time-start_time).seconds
            if interval == 3:
                break
            p.resetBaseVelocity(cylinder_robot_Id,[cosx3/5,sinx3/5,0])
            p.stepSimulation()
            time.sleep(1./240.)
        flag = first_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y)
        second_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y,flag)

# 向“下”移动
def second_time_first_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y,flag):
    #print("the second time first move")
    while 1:
        cylinder_target_data = p.getBasePositionAndOrientation(cylinder_target_Id)
        cylinder_robot_data = p.getBasePositionAndOrientation(cylinder_robot_Id)
        robot_x = cylinder_robot_data[0][0]
        robot_y = cylinder_robot_data[0][1]
        target_x = cylinder_target_data[0][0]
        target_y = cylinder_target_data[0][1]
        Scalar_Product_1 = (path_x - robot_x)*(target_x - robot_x) + (path_y - robot_y)*(target_y - robot_y)#向量的点乘
        if Scalar_Product_1 > 0:
            break
        if flag == 1:
            if target_y >= path_y:
                cosx21 = 0
                sinx21 = -1
                p.resetBaseVelocity(cylinder_robot_Id,[cosx21/5,sinx21/5,0])
            else:
                cosx21 = 0
                sinx21 = 1
                p.resetBaseVelocity(cylinder_robot_Id,[cosx21/5,sinx21/5,0])
        elif flag ==3:
            if target_x >= path_x:
                cosx21 = -1
                sinx21 = 0
                p.resetBaseVelocity(cylinder_robot_Id,[cosx21/5,sinx21/5,0])
            else:
                cosx21 = 1
                sinx21 = 0
                p.resetBaseVelocity(cylinder_robot_Id,[cosx21/5,sinx21/5,0])
        else:
            orthocenter = orthocenter_point(robot_x,robot_y,path_x,path_y,target_x,target_y)
            orthocenter_x = orthocenter[0]
            orthocenter_y = orthocenter[1]
            flag = orthocenter[2]
            dir_x = orthocenter_x - target_x
            dir_y = orthocenter_y - target_y
            cosx21 = dir_x/(math.sqrt(np.square(dir_x)+np.square(dir_y)))
            sinx21 = dir_y/(math.sqrt(np.square(dir_x)+np.square(dir_y)))
        p.resetBaseVelocity(cylinder_robot_Id,[cosx21/5,sinx21/5,0])
        p.stepSimulation()
        time.sleep(1./240.)    

# 第一种情况
def the_first_situation(cylinder_target_Id,cylinder_robot_Id,robot_x,robot_y,path_x,path_y,target_x,target_y):
    print("The first situation")
    flag = first_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y)
    time.sleep(3)
    print("*********************************************")
    #robot 移动到 target和path的延长线上,robot moves to the extension of target and path.
    second_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y,flag)
    time.sleep(3)
    orthocenter = orthocenter_point(robot_x,robot_y,path_x,path_y,target_x,target_y)
    flag = orthocenter[2]
    #robot begin to move to target and pull the target to the goal
    third_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y,flag)

# 第二种情况
def the_second_situation(cylinder_target_Id,cylinder_robot_Id,robot_x,robot_y,path_x,path_y,target_x,target_y):
    print("The second situation")
    orthocenter = orthocenter_point(robot_x,robot_y,path_x,path_y,target_x,target_y)
    flag = orthocenter[2]
    second_time_first_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y,flag)
    flag = first_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y)
    time.sleep(3)
    print("*********************************************")
    #robot 移动到 target和path的延长线上,robot moves to the extension of target and path.
    second_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y,flag)
    time.sleep(3)
    orthocenter = orthocenter_point(robot_x,robot_y,path_x,path_y,target_x,target_y)
    flag = orthocenter[2]
    third_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y,flag) 

# 第三种情况
def the_third_situation(cylinder_target_Id,cylinder_robot_Id,robot_x,robot_y,path_x,path_y,target_x,target_y):
    print("The third situation")
    first_move(cylinder_target_Id,cylinder_robot_Id,path_x,path_y)
    time.sleep(3)
    while 1:
        dir_x31 = target_x - path_x
        dir_y31 = target_y - path_y
        cosx31 = dir_x31/(math.sqrt(np.square(dir_x31)+np.square(dir_y31)))
        sinx31 = dir_y31/(math.sqrt(np.square(dir_x31)+np.square(dir_y31)))
        cylinder_target_data = p.getBasePositionAndOrientation(cylinder_target_Id)
        cylinder_robot_data = p.getBasePositionAndOrientation(cylinder_robot_Id)
        robot_x = cylinder_robot_data[0][0]
        robot_y = cylinder_robot_data[0][1]
        target_x = cylinder_target_data[0][0]
        target_y = cylinder_target_data[0][1]
        Scalar_Product_2 = (target_x - path_x)*(robot_x - path_x) + (target_y - path_y)*(robot_y - path_y)
        if Scalar_Product_2 > 0:
            break
        p.resetBaseVelocity(cylinder_robot_Id,[cosx31/5,sinx31/5,0])
        p.stepSimulation()
        time.sleep(1./240.)
    the_first_situation(cylinder_target_Id,cylinder_robot_Id,robot_x,robot_y,path_x,path_y,target_x,target_y)

# 第四种情况
def the_fourth_situation(cylinder_target_Id,cylinder_robot_Id,robot_x,robot_y,path_x,path_y,target_x,target_y):
    print("The fourth situation")
    while 1:
        dir_x = path_x - robot_x
        dir_y = path_y - robot_y
        cosx41 = dir_x/(math.sqrt(np.square(dir_x)+np.square(dir_y)))
        sinx41 = dir_y/(math.sqrt(np.square(dir_x)+np.square(dir_y)))
        cylinder_target_data = p.getBasePositionAndOrientation(cylinder_target_Id)
        cylinder_robot_data = p.getBasePositionAndOrientation(cylinder_robot_Id)
        robot_x = cylinder_robot_data[0][0]
        robot_y = cylinder_robot_data[0][1]
        path_x = path[i][0]
        path_y = path[i][1]
        target_x = cylinder_target_data[0][0]
        target_y = cylinder_target_data[0][1]            
        Scalar_Product_2 = (target_x - path_x)*(robot_x - path_x) + (target_y - path_y)*(robot_y - path_y)
        if Scalar_Product_2 > 0:
            start_time = datetime.datetime.now()
            while 1:
                end_time = datetime.datetime.now()
                interval = (end_time-start_time).seconds
                if interval == 1:
                    break
                p.resetBaseVelocity(cylinder_robot_Id,[cosx41/5,sinx41/5,0])
                p.stepSimulation()
                time.sleep(1./240.)
            break
        p.resetBaseVelocity(cylinder_robot_Id,[cosx41/5,sinx41/5,0])
        p.stepSimulation()
        time.sleep(1./240.)
    cylinder_target_data = p.getBasePositionAndOrientation(cylinder_target_Id)
    cylinder_robot_data = p.getBasePositionAndOrientation(cylinder_robot_Id)
    robot_x = cylinder_robot_data[0][0]
    robot_y = cylinder_robot_data[0][1]
    path_x = path[i][0]
    path_y = path[i][1]
    target_x = cylinder_target_data[0][0]
    target_y = cylinder_target_data[0][1]  
    Scalar_Product_1 = (path_x - robot_x)*(target_x - robot_x) + (path_y - robot_y)*(target_y - robot_y)
    Scalar_Product_2 = (target_x - path_x)*(robot_x - path_x) + (target_y - path_y)*(robot_y - path_y)
    if Scalar_Product_1 >= 0 and Scalar_Product_2 > 0:
        print("Before that I am the fourth situation")
        the_first_situation(cylinder_target_Id,cylinder_robot_Id,robot_x,robot_y,path_x,path_y,target_x,target_y)
        
    #点乘是钝角
    elif Scalar_Product_1 < 0 and Scalar_Product_2 > 0:
        print("Before that I am the fourth situation")
        the_second_situation(cylinder_target_Id,cylinder_robot_Id,robot_x,robot_y,path_x,path_y,target_x,target_y)
        
        
    elif Scalar_Product_2 == 0:
        print("Before that I am the fourth situation")
        the_third_situation(cylinder_target_Id,cylinder_robot_Id,robot_x,robot_y,path_x,path_y,target_x,target_y)
        
    else:
        print("Before that I am the fourth situation")
        the_fourth_situation(cylinder_target_Id,cylinder_robot_Id,robot_x,robot_y,path_x,path_y,target_x,target_y)

# %%
getVisualShapeData
您可以使用getVisualShapeData访问视觉形状信息。 您可以使用它将您自己的呈现方法与PyBullet模拟连接起来，并在每个模拟步骤之后手动同步世界转换。 您还可以使用GET网格数据，特别是对于可变形对象，来接收有关顶点位置的数据。
输入参数：

必需/可选参数	参数名字	参数类型	介绍
必需	objectUniqueId	int	对象的唯一id，由加载方法返回。
可选	flags	int	VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS还将提供纹理独特的ID。
可选	physicsClientId	int	物理客户端id由“连接”返回。
输出参数：

参数名字	参数类型	介绍
objectUniqueId	int	对象的唯一id，由加载方法返回。
linkIndex	int	基础的链接索引或-1。
visualGeometryType	int	视觉几何类型(TBD)。
dimensions	vec3, list of 3 floats	几何尺寸(尺寸，局部尺度）。
meshAssetFileName	string, list of chars	到三角形网格的路径，如果有的话。 通常相对于URDF、SDF或MJCF文件位置，但可能是绝对的。
localVisualFrame position	vec3, list of 3 floats	局部视觉框架的位置，相对于链接/关节框架。
localVisualFrame orientation	vec4, list of 4 floats	局部视觉框架相对于链路/关节框架的方向。
rgbaColor	vec4, list of 4 floats	红色/绿色/蓝色/阿尔法的URDF颜色(如果指定的话）。
textureUniqueId	int	(字段只存在于使用VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS标志时)纹理形状的唯一id，或-1如果没有。

changeDynamics
您可以使用changeDynamics改变质量、摩擦和归位系数等属性。

必需/可选参数	参数名字	参数类型	介绍
必需	objectUniqueId	int	对象的唯一id，由加载方法返回。
必需	linkIndex	int	链接指数或-1为基准。
可选	mass	double	改变链接的质量(或链接指数-1的基础)。
可选	lateralFriction	double	横向（线性）接触摩擦。
可选	spinningFriction	double	围绕接触法线的扭转摩擦。
可选	rollingFriction	double	与接触法线正交的扭转摩擦力（保持这个值非常接近于零，否则模拟会变得非常不真实）。

resetBaseVelocity
您可以使用重置基本速度重置身体底部的线性和/或角速度
输入参数：

必需/可选参数	参数名字	参数类型	介绍
必需	objectUniqueId	int	对象的唯一id，由加载方法返回。
可选	linearVelocity	vec3, list of 3 floats	直角世界坐标中的线速度[x，y，z]。
可选	angularVelocity	vec3, list of 3 floats	直角速度[wx，wy，wz]在笛卡尔世界坐标中。
可选	physicsClientId	int	如果您连接到多个服务器，您可以选择一个。
————————————————
版权声明：本文为CSDN博主「墨绿色的摆渡人」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/xzs1210652636/article/details/109234531
