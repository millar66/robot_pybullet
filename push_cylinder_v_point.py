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
import random
import copy

# %%
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

#path = [[0.4,0.4,0.3],[0.6,0.6,0.3],[0.8,0.8,0.3],[1.0,1.0,0.3],[1.2,1.2,0.3]]
# [[0.4,1.0,0.3],[0.6,-1.0,0.3],[0.8,2.0,0.3],[-1.0,0.0,0.3],[-1.2,-2.0,0.3]]
path_string = input("Please input the path:")

#path = [[0.4,0.0,0.3],[0.6,0.0,0.3],[0.8,0.0,0.3],[1.0,0.0,0.3],[1.2,0.0,0.3]]

#path = [[0.1,-0.02,0.3],[0.2,-0.04,0.3],[0.3,-0.06,0.3],[0.4,-0.082,0.3],[0.5,-0.107,0.3],[0.6,-0.135,0.3],[0.7,-0.168,0.3],[0.8,-0.205,0.3],[0.9,-0.251,0.3],[1.0,-0.309,0.3],[1.1,-0.375,0.3],[1.2,-0.45,0.3],[1.3,-0.535,0.3],[1.4,-0.622,0.3],[1.5,-0.718,0.3],[1.6,-0.812,0.3],[1.7,-0.914,0.3],[1.8,-1.015,0.3],[1.9,-1.117,0.3],[2.0,-1.224,0.3]]
#p.applyExternalForce(cylinder_target_Id,-1,[1000090,0,0],[0,0,0.3],p.WORLD_FRAME)
        
#p.changeDynamics(planeId,-1,lateralFriction = 0,spinningFriction = 0,rollingFriction = 0)

def SetSimulation():
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-9.81) #设置重力
    planeId = p.loadURDF("plane.urdf")
    
    cylinder_Obstacle_1_StartPos = [-0.4, 0.3, 0.2 ]
    cylinder_Obstacle_2_StartPos = [ 0.0,-0.6, 0.15]
    cylinder_Obstacle_3_StartPos = [ 0.6,-0.6, 0.1 ]
    cylinder_Obstacle_4_StartPos = [ 0.8, 0.2, 0.1]
    cylinder_target_StartPos = [ 0, 0.0, 0.3]
    cylinder_robot_StartPos = [ -0.4, 0.0, 0.3]
    
    cylinder_Obstacle_1_StartOrientation = p.getQuaternionFromEuler([0,0,0])
    cylinder_Obstacle_2_StartOrientation = p.getQuaternionFromEuler([0,0,0])
    cylinder_Obstacle_3_StartOrientation = p.getQuaternionFromEuler([0,0,0])
    cylinder_Obstacle_4_StartOrientation = p.getQuaternionFromEuler([0,0,0])
    cylinder_target_StartOrientation = p.getQuaternionFromEuler([0,0,0])
    cylinder_robot_StartOrientation = p.getQuaternionFromEuler([0,0,0])
    
    cylinder_Obstacle_text = p.addUserDebugText(text="p1", textPosition=path[0], textColorRGB=[0, 1, 0], textSize=1.2)
    cylinder_Obstacle_text = p.addUserDebugText(text="p2", textPosition=path[1], textColorRGB=[0, 1, 0], textSize=1.2)
    cylinder_Obstacle_text = p.addUserDebugText(text="p3", textPosition=path[2], textColorRGB=[0, 1, 0], textSize=1.2)
    cylinder_Obstacle_text = p.addUserDebugText(text="p4", textPosition=path[3], textColorRGB=[0, 1, 0], textSize=1.2)
    cylinder_Obstacle_text = p.addUserDebugText(text="p5", textPosition=path[4], textColorRGB=[0, 1, 0], textSize=1.2)
    
    cylinder_Obstacle_1_Id = p.loadURDF("mymodel/cylinder_obstacle1.urdf",cylinder_Obstacle_1_StartPos,cylinder_Obstacle_1_StartOrientation)
    cylinder_Obstacle_2_Id = p.loadURDF("mymodel/cylinder_obstacle2.urdf",cylinder_Obstacle_2_StartPos,cylinder_Obstacle_2_StartOrientation)
    cylinder_Obstacle_3_Id = p.loadURDF("mymodel/cylinder_obstacle3.urdf",cylinder_Obstacle_3_StartPos,cylinder_Obstacle_3_StartOrientation)
    cylinder_Obstacle_4_Id = p.loadURDF("mymodel/cylinder_obstacle4.urdf",cylinder_Obstacle_4_StartPos,cylinder_Obstacle_4_StartOrientation)
    cylinder_target_Id = p.loadURDF("mymodel/cylinder_target.urdf",cylinder_target_StartPos,cylinder_target_StartOrientation)
    cylinder_robot_Id = p.loadURDF("mymodel/cylinder_robot.urdf",cylinder_robot_StartPos,cylinder_robot_StartOrientation)
    return cylinder_robot_Id,cylinder_target_Id #我们需要直到机器人和目标的Id，然后才能对机器人和目标进行监控
    
def TransferToFloat(path_string):
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
    return path

def ComputeDistance(path_x,path_y,target_x,target_y):
    distance = (math.sqrt(np.square(path_x - target_x)+np.square(path_y - target_y)))
    return distance

def RollOut(Velocity,cylinder_robot_Id,cylinder_target_Id,path):
    length_path = len(path)
    distance = []
    for i in range(length_path):
    	#计时器：每一段的速度移动1s
        start_time = datetime.datetime.now()
        while 1:
            end_time = datetime.datetime.now()
            interval = (end_time-start_time).seconds
            if interval == 1:
                break
            p.resetBaseVelocity(cylinder_robot_Id,Velocity[i])
            p.stepSimulation()
            time.sleep(1./240.)
		#监控机器人和目标的信息
        cylinder_target_data = p.getBasePositionAndOrientation(cylinder_target_Id)
        target_x = cylinder_target_data[0][0]
        target_y = cylinder_target_data[0][1]
        path_x = path[i][0]
        path_y = path[i][1]    
        distance.append(ComputeDistance(path_x,path_y,target_x,target_y))
    #每一段cost加和
    length = len(distance)
    cost = 0
    for i in range(length):
        cost = cost + distance[i]   
    return cost

def FindABetterU(Velocity,cost,cylinder_robot_Id,cylinder_target_Id):
    #给速度添加随机值，这里我一次拿出来5个不同的速度列表，
    #然后让机器人仿真分别运行这五个不同的速度，
    #然后比较五个cost，选择最优的速度。
    #当然我们可以修改“5”,其实这个随便的，当然越大效果越好。
    velocity_with_noise = []
    for i in range(5):
        Velocity_temp = copy.deepcopy(Velocity)
        velocity_with_noise.append(Velocity_temp)
        velocity_with_noise[i] = AddNoiseOnU(velocity_with_noise[i])
    # print("Velocity1:",Velocity)
    # print("velocity_with_noise:",velocity_with_noise)
    #分别计算不同速度时候的cost
    cost_with_noise = []
    for i in range(5):
        cost_new = RollOut(velocity_with_noise[i],cylinder_robot_Id,cylinder_target_Id,path)
        cost_with_noise.append(cost_new)
        p.resetSimulation()
        cylinder_robot_Id,cylinder_target_Id = SetSimulation()
 	#找到cost最小的那个速度的索引
    index = cost_with_noise.index(min(cost_with_noise))
    print("cost_with_noise:",cost_with_noise)
    print("index:",index)
    #和初始的cost作比较（这里有个if，else的原因是，如果随机到的速度导致的cost非常大，甚至超过了原来的cost，我们可以不更新速度，然后重新给速度一个随机值）
    if cost_with_noise[index]<cost:
        print("cost_with_noise[index]<:",cost_with_noise[index],"cost:",cost)
        # print("Velocity2:",Velocity)
        # print("velocity_with_noise[index]:",velocity_with_noise[index])
        return velocity_with_noise[index],cost_with_noise[index]
    else:
        # print("cost_with_noise[index]>:",cost_with_noise[index],"cost:",cost)
        # print("Velocity2:",Velocity)
        print("velocity_with_noise[index]:",velocity_with_noise[index])
        return Velocity,cost

def AddNoiseOnU(VelocityNoise):
    length = len(VelocityNoise)
    for i in range(length):
        noisy_value_x = random.uniform(-0.005,0.005)
        noisy_value_y = random.uniform(-0.005,0.005)
        VelocityNoise[i][0] = VelocityNoise[i][0] + noisy_value_x
        VelocityNoise[i][1] = VelocityNoise[i][1] + noisy_value_y
    return VelocityNoise

Velocity = [[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0],[0.19,0,0]]

cylinder_robot_Id,cylinder_target_Id = SetSimulation()
path = TransferToFloat(path_string)
cost = RollOut(Velocity,cylinder_robot_Id,cylinder_target_Id,path)

print("cost:",cost)
p.resetSimulation()
cylinder_robot_Id,cylinder_target_Id = SetSimulation()

while cost > 0.1:


    print("**********************************************************")
    
    Velocity,cost = FindABetterU(Velocity,cost,cylinder_robot_Id,cylinder_target_Id)
    
    cost = RollOut(Velocity,cylinder_robot_Id,cylinder_target_Id,path)
    
    p.resetSimulation()
    cylinder_robot_Id,cylinder_target_Id = SetSimulation()
    
cost = RollOut(Velocity,cylinder_robot_Id,cylinder_target_Id,path)

# %%
p.disconnect()#有连接就有断开




# %%
# getVisualShapeData
# 您可以使用getVisualShapeData访问视觉形状信息。 您可以使用它将您自己的呈现方法与PyBullet模拟连接起来，并在每个模拟步骤之后手动同步世界转换。 您还可以使用GET网格数据，特别是对于可变形对象，来接收有关顶点位置的数据。
# 输入参数：

# 必需/可选参数	参数名字	参数类型	介绍
# 必需	objectUniqueId	int	对象的唯一id，由加载方法返回。
# 可选	flags	int	VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS还将提供纹理独特的ID。
# 可选	physicsClientId	int	物理客户端id由“连接”返回。
# 输出参数：

# 参数名字	参数类型	介绍
# objectUniqueId	int	对象的唯一id，由加载方法返回。
# linkIndex	int	基础的链接索引或-1。
# visualGeometryType	int	视觉几何类型(TBD)。
# dimensions	vec3, list of 3 floats	几何尺寸(尺寸，局部尺度）。
# meshAssetFileName	string, list of chars	到三角形网格的路径，如果有的话。 通常相对于URDF、SDF或MJCF文件位置，但可能是绝对的。
# localVisualFrame position	vec3, list of 3 floats	局部视觉框架的位置，相对于链接/关节框架。
# localVisualFrame orientation	vec4, list of 4 floats	局部视觉框架相对于链路/关节框架的方向。
# rgbaColor	vec4, list of 4 floats	红色/绿色/蓝色/阿尔法的URDF颜色(如果指定的话）。
# textureUniqueId	int	(字段只存在于使用VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS标志时)纹理形状的唯一id，或-1如果没有。

# changeDynamics
# 您可以使用changeDynamics改变质量、摩擦和归位系数等属性。

# 必需/可选参数	参数名字	参数类型	介绍
# 必需	objectUniqueId	int	对象的唯一id，由加载方法返回。
# 必需	linkIndex	int	链接指数或-1为基准。
# 可选	mass	double	改变链接的质量(或链接指数-1的基础)。
# 可选	lateralFriction	double	横向（线性）接触摩擦。
# 可选	spinningFriction	double	围绕接触法线的扭转摩擦。
# 可选	rollingFriction	double	与接触法线正交的扭转摩擦力（保持这个值非常接近于零，否则模拟会变得非常不真实）。

# resetBaseVelocity
# 您可以使用重置基本速度重置身体底部的线性和/或角速度
# 输入参数：

# 必需/可选参数	参数名字	参数类型	介绍
# 必需	objectUniqueId	int	对象的唯一id，由加载方法返回。
# 可选	linearVelocity	vec3, list of 3 floats	直角世界坐标中的线速度[x，y，z]。
# 可选	angularVelocity	vec3, list of 3 floats	直角速度[wx，wy，wz]在笛卡尔世界坐标中。
# 可选	physicsClientId	int	如果您连接到多个服务器，您可以选择一个。

