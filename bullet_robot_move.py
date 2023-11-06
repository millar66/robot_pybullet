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

# %%
# physicsVlient = p.connect(p.GUI,"option=opengl2")#or p.DIRECT for non-graphical version
physicsVlient = p.connect(p.GUI)

# 该语句的作用是禁用tinyrenderer，也就是不让CPU上的集成显卡来参与渲染工作。
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
# p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
# 渲染出来的GUI上许多控件，不显示
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)

planeId = p.loadURDF("plane.urdf")
# 除了使用loadURDF加载urdf模型外，我们还可以通过loadSDF来加载sdf模型，
# 通过loadMJCF来加载mjcf模型。其中sdf是数据库文件，mjcf是MuJoCo平台的机器人描述文件。
robotId = p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")

robotStartPos = [0,0,0]#机器人的初始位置
cylinderStartPos = [1,0,0.3]#圆柱体的初始位置（在环境中我将一个圆柱体设置为平台（一个台子而已，可以是任何东西））
boxStartPos = [1,0,0.6 + 0.05 + 0.01]#箱子的基础位置（我在台子上放了一个箱子当作目标，为以后机器人的抓取做准备）
#（机器人）会转换成一个四元数，可以理解成能够控制模型在空间中绕x，y，z轴旋转的参数。
#（参数是角度。e.g. [3.14,0,0] == [pai,0,0];[1.57,0,0] == [pai/2,0,0]。参数的正负表示旋转的不同方向。）
robotStartOrientation = p.getQuaternionFromEuler([0,0,0])
cylinderStartOrientation = p.getQuaternionFromEuler([0,0,0])#（圆柱体）同上
boxStartOrientation = p.getQuaternionFromEuler([0,0,0])#（箱子）同上

#按照以上的参数重新设置机器人的位置。
p.resetBasePositionAndOrientation(robotId[0],robotStartPos,robotStartOrientation)
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId[0])
print("-" * 20)
print(f"机器人的位置坐标为:{cubePos}\n机器人的朝向四元数为:{cubeOrn}")
print("-" * 20)
info_tuple = p.getJointInfo(robotId[0], 7)

cylinderId = p.loadURDF("mymodel/cylinder1.urdf",cylinderStartPos,cylinderStartOrientation)
boxId = p.loadURDF("mymodel/cube1.urdf",boxStartPos,boxStartOrientation)

robot7StartPos = [0,0,1.2]#机器人第7个结点的初始位置
robotEndPos = [0.75,0,0.625]#机器人第7个结点的结束位置
text_id = p.addUserDebugText(text="start", textPosition=robot7StartPos, textColorRGB=[0, 1, 0], textSize=1.2)
text_id = p.addUserDebugText(text="Destination", textPosition=robotEndPos, textColorRGB=[0, 0, 1], textSize=1.2)
p.addUserDebugLine(lineFromXYZ=robot7StartPos, lineToXYZ=robotEndPos, lineColorRGB=[0, 1, 0], lineWidth=2)

robotEndOrientation = p.getQuaternionFromEuler([1.57,0,1.57])#同上
startPos_array = np.array(robot7StartPos)#将参数转换成array然后才能进行“+，-，*，/”运算
endPos_array = np.array(robotEndPos)#同上
stepNum = 50#我将移动过程分成了5步
step_array = (endPos_array - startPos_array)/stepNum#每一步走的长度（其实就是每一步x,y,z坐标的变化量）

for j in range(stepNum):#循环次数
    print(j,"step")
    robotStepPos = list(step_array + startPos_array)#转换回list
    targetPositionsJoints = p.calculateInverseKinematics(robotId[0],7,robotStepPos,targetOrientation = robotEndOrientation)#计算IK的解决方案（个人理解）（下面会具体介绍参数）
    p.setJointMotorControlArray(robotId[0],range(11),p.POSITION_CONTROL,targetPositions = targetPositionsJoints)#执行方案（下面会具体介绍参数）
    for i in range (50):#移动时间
        p.stepSimulation()#这个是一定要写的，每次移动都要调用这个方法
        time.sleep(1./240.)
        # print("i:",i)
    print("------------------------------------------------------------------------------")
    startPos_array = np.array(robotStepPos)#因为移动过了，所以更新一下坐标
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId[0])
print("-" * 20)
print(f"机器人的位置坐标为:{cubePos}\n机器人的朝向四元数为:{cubeOrn}")
print("-" * 20)

robot7StartPos = [0.75,0,0.625]#机器人第7个结点的初始位置
robotEndPos = [0,0,1.2]#机器人第7个结点的结束位置
robotEndOrientation = p.getQuaternionFromEuler([0,0,0])#同上
startPos_array = np.array(robot7StartPos)#将参数转换成array然后才能进行“+，-，*，/”运算
endPos_array = np.array(robotEndPos)#同上
stepNum = 5#我将移动过程分成了5步
step_array = (endPos_array - startPos_array)/stepNum#每一步走的长度（其实就是每一步x,y,z坐标的变化量）

for j in range(stepNum):#循环次数
    print(j,"step")
    robotStepPos = list(step_array + startPos_array)#转换回list
    targetPositionsJoints = p.calculateInverseKinematics(robotId[0],7,robotStepPos,targetOrientation = robotEndOrientation)#计算IK的解决方案（个人理解）（下面会具体介绍参数）
    p.setJointMotorControlArray(robotId[0],range(11),p.POSITION_CONTROL,targetPositions = targetPositionsJoints)#执行方案（下面会具体介绍参数）
    for i in range (100):#移动时间
        p.stepSimulation()#这个是一定要写的，每次移动都要调用这个方法
        time.sleep(1./10.)
        # print("i:",i)
    print("------------------------------------------------------------------------------")
    startPos_array = np.array(robotStepPos)#因为移动过了，所以更新一下坐标
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId[0])
print("-" * 20)
print(f"机器人的位置坐标为:{cubePos}\n机器人的朝向四元数为:{cubeOrn}")
print("-" * 20)




# %%
p.resetSimulation(cylinderId)
p.disconnect()

# %%
cubeStarPos = [1,2,3]
text_id = p.addUserDebugText(text="start", textPosition=cubeStarPos, textColorRGB=[0, 1, 0], textSize=1.2)
text_id = p.addUserDebugText(text="Destination", textPosition=[0,0,0], textColorRGB=[0, 0, 1], textSize=1.2)
p.addUserDebugLine(lineFromXYZ=cubeStarPos, lineToXYZ=[0,0,0], lineColorRGB=[0, 1, 0], lineWidth=2)
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",cubeStarPos, cubeStartOrientation)

joint_num = p.getNumJoints(robotId[0])
print("节点数量为：", joint_num)
# JointInfo = p.getJointInfo(boxId,6)
print("节点信息：")
for joint_index in range(joint_num):
    info_tuple = p.getJointInfo(robotId[0], joint_index)
    print(f"关节序号：{info_tuple[0]}\n\
            关节名称：{info_tuple[1]}\n\
            关节类型：{info_tuple[2]}\n\
            机器人第一个位置的变量索引：{info_tuple[3]}\n\
            机器人第一个速度的变量索引：{info_tuple[4]}\n\
            保留参数：{info_tuple[5]}\n\
            关节的阻尼大小：{info_tuple[6]}\n\
            关节的摩擦系数：{info_tuple[7]}\n\
            slider和revolute(hinge)类型的位移最小值：{info_tuple[8]}\n\
            slider和revolute(hinge)类型的位移最大值：{info_tuple[9]}\n\
            关节驱动的最大值：{info_tuple[10]}\n\
            关节的最大速度：{info_tuple[11]}\n\
            节点名称：{info_tuple[12]}\n\
            局部框架中的关节轴系：{info_tuple[13]}\n\
            父节点frame的关节位置：{info_tuple[14]}\n\
            父节点frame的关节方向：{info_tuple[15]}\n\
            父节点的索引，若是基座返回-1：{info_tuple[16]}\n\n")

# %%
p.setRealTimeSimulation(1) # 实时模拟
while True:
    # pass
    time.sleep(1)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    print("-" * 20)
    print(f"机器人的位置坐标为:{cubePos}\n机器人的朝向四元数为:{cubeOrn}")
    print("-" * 20)


# %%
p.setRealTimeSimulation(0)  # 关闭实时模拟
p.setTimeStep(1./240.)  # 设置时间步
for i in range (2000):
  p.stepSimulation()
  time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print("-" * 20)
print(f"机器人的位置坐标为:{cubePos}\n机器人的朝向四元数为:{cubeOrn}")
print("-" * 20)


# %%
# 画一条空间内的直线
stars = [[1, 1, 0], [-1, 1, 0], [-1, 1, 3], [1, 1, 3]]
ends = [[-1, 1, 0], [-1, 1, 3], [1, 1, 3], [1, 1, 0]]
for i, j in zip(stars, ends):
    p.addUserDebugLine(lineFromXYZ=i, lineToXYZ=j, lineColorRGB=[0, 1, 0], lineWidth=2)
# 画仿真中物体的运动轨迹
p.addUserDebugLine(lineFromXYZ=body_position_w_pre, lineToXYZ=body_position_w,
                   lineColorRGB=[1, 0, 0], lineWidth=2)  # show body trajectory
p.addUserDebugLine(lineFromXYZ=foot_position_w_pre, lineToXYZ=foot_position_w,
                   lineColorRGB=[0, 0, 1], lineWidth=2, lifeTime=30)  # show foot trajectory

# 添加参数滑块，分别为速度，驱动力、转向
targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -10, 10, 0)  # 参数名称 最小值 最大值 起始值
maxForceSlider = p.addUserDebugParameter("maxForce", 0, 10, 10)
steeringSlider = p.addUserDebugParameter("steering", -0.5, 0.5, 0)

# 读取参数的值，读取速度，转向角度，驱动力参数
targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
maxForce = p.readUserDebugParameter(maxForceSlider)
steeringAngle = p.readUserDebugParameter(steeringSlider)

# 删除所有的滑块和按钮：
p.removeAllUserParameters(physicsClientId)

# 使用唯一id删除调试项（文本、线等）: 
p.removeUserDebugItem(itemUniqueId, physicsClientId)

# 删除所有调试项: 
p.removeAllUserDebugItems()

# 更改某个特定物体的特定link的颜色
p.setDebugObjectColor(objectUniqueId=robot_id, linkIndex=-1, objectDebugColorRGB=[0, 0, 1])






# %%
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0) # 不让引擎渲染没有加载好的场景
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0) # 不让CPU上的集成显卡参与渲染工作
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) # 不显示GUI上的控件
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1) # 打开渲染
p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)  # 一步步地渲染

