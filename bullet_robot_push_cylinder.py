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
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)
# p.setGravity(0,0,0)

# cylinderStartPos = [0,0,0.3]#圆柱体的初始位置
cylinderStartPos = [0.2,0,0.3]#圆柱体的初始位置
cylinderStartOrientation = p.getQuaternionFromEuler([0,0,0])#（圆柱体）这里的参数会转换成一个四元数，可以理解成能够控制模型在空间中绕x，y，z轴旋转的参数。（参数是角度。e.g. [3.14,0,0] == [pai,0,0];[1.57,0,0] == [pai/2,0,0]。参数的正负表示旋转的不同方向。）

planeId = p.loadURDF("plane.urdf")
cylinderId = p.loadURDF("mymodel/cylinder1.urdf",cylinderStartPos,cylinderStartOrientation)

# changeDynamics
# 您可以使用changeDynamics改变质量、摩擦和归位系数等属性。

# 必需/可选参数	参数名字	参数类型	介绍
# 必需	objectUniqueId	int	对象的唯一id，由加载方法返回。
# 必需	linkIndex	int	链接指数或-1为基准。
# 可选	mass	double	改变链接的质量(或链接指数-1的基础)。
# 可选	lateralFriction	double	横向（线性）接触摩擦。
# 可选	spinningFriction	double	围绕接触法线的扭转摩擦。
# 可选	rollingFriction	double	与接触法线正交的扭转摩擦力（保持这个值非常接近于零，否则模拟会变得非常不真实）。

#这里是为了设置“plane”的参数，让“plane”变成理想平面（无摩擦力）
p.changeDynamics(planeId,-1,lateralFriction = 0.1,spinningFriction = 1000,rollingFriction = 0)

# %%
p.setRealTimeSimulation(0) # 实时模拟
# for k in range(2):#移动三个阶段
while True:
    # p.applyExternalForce(cylinderId,-1,[1,0,0],[0,0,0.3],p.WORLD_FRAME)#施加延x轴正方向的力
    for i in range (480):#控制施加力的时间
        p.applyExternalForce(cylinderId,-1,[1,0,0],[0,0,0.3],p.WORLD_FRAME)#施加延x轴正方向的力
        # p.applyExternalForce(cylinderId,-1,[1,0,0],[0,0,0],p.WORLD_FRAME)#施加延x轴正方向的力
        # p.applyExternalForce(cylinderId,-1,[(480-i)/500,0,0],[0,0,0.3],p.WORLD_FRAME)#施加延x轴正方向的力
        p.stepSimulation()
        time.sleep(1./240.)
    # print("i:",i)
    # p.applyExternalForce(cylinderId,-1,[-1,0,0],[0,0,0.3],p.WORLD_FRAME)#施加延x轴负方向的力（为了减速）
    for j in range (480):#控制施加力的时间
        p.applyExternalForce(cylinderId,-1,[-1,0,0],[0,0,0.3],p.WORLD_FRAME)#施加延x轴负方向的力（为了减速）
        # p.applyExternalForce(cylinderId,-1,[-1,0,0],[0,0,0],p.WORLD_FRAME)#施加延x轴负方向的力（为了减速）
        # p.applyExternalForce(cylinderId,-1,[j/500,0,0],[0,0,0.3],p.WORLD_FRAME)#施加延x轴负方向的力（为了减速）
        p.stepSimulation()
        time.sleep(1./240.)

# %%
p.disconnect()
