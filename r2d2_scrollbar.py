#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 13 15:59:37 2023

@author: lihui.liu
"""

import pybullet as p 
import pybullet_data
from time import sleep

use_gui = True
if use_gui:
    cid = p.connect(p.GUI)
else:
    cid = p.connect(p.DIRECT)

# 添加资源
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5])

# 配置渲染逻辑
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

p.resetDebugVisualizerCamera(cameraTargetPosition=[0,0,0],\
                             cameraDistance=6.8,\
                             cameraPitch=-41.2,\
                             cameraYaw=66.4) #转变视角

# 绘制直线
froms = [[1, 1, 0], [-1, 1, 0], [-1, 1, 3], [1, 1, 3]]
tos = [[-1, 1, 0], [-1, 1, 3], [1, 1, 3], [1, 1, 0]]
for f, t in zip(froms, tos):
    p.addUserDebugLine(
        lineFromXYZ=f,
        lineToXYZ=t,
        lineColorRGB=[0, 1, 0],
        lineWidth=2
    )

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

Destination_id = p.addUserDebugText(
    text="Destination",
    textPosition=[0, 1, 3],
    textColorRGB=[1, 0, 0],
    textSize=1.2,
)

p.addUserDebugText(
    text="I'm R2D2",
    textPosition=[0, 0, 1.2],
    textColorRGB=[0, 0, 1],
    textSize=1.2
)

wheel_link_tuples = [(p.getJointInfo(robot_id, i)[0],p.getJointInfo(robot_id,i)[1].decode("utf-8"))  # 0:序号 1:名称
                     for i in range(p.getNumJoints(robot_id))
                     if "wheel" in p.getJointInfo(robot_id,i)[1].decode("utf-8")]

wheel_velocity_params_ids = [p.addUserDebugParameter(
    paramName=wheel_link_tuples[i][1] + " V",
    rangeMin=-50,
    rangeMax=50,
    startValue=0
    ) for i in range(4)]

wheel_force_params_ids = [p.addUserDebugParameter(
    paramName=wheel_link_tuples[i][1] + " F",
    rangeMin=-100,
    rangeMax=100,
    startValue=0
    ) for i in range(4)]

# 添加按钮控件
btn = p.addUserDebugParameter(
    paramName="reset",
    rangeMin=1,
    rangeMax=0,
    startValue=0
)
previous_btn_value = p.readUserDebugParameter(btn)

# %%
while True:
    p.stepSimulation()
    indices = [i for i, _ in wheel_link_tuples]
    velocities = [p.readUserDebugParameter(param_id) for param_id in wheel_velocity_params_ids]
    forces = [p.readUserDebugParameter(param_id) for param_id in wheel_force_params_ids]
    p.setJointMotorControlArray(
        bodyUniqueId=robot_id,
        jointIndices=indices,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=velocities,
        forces=forces)
    # 如果按钮的累加值发生变化了，说明clicked了
    if p.readUserDebugParameter(btn) != previous_btn_value:
        p.removeAllUserDebugItems()
        previous_btn_value = p.readUserDebugParameter(btn)
    
    p.getCameraImage(480, 320)
    sleep(1 / 240)

# %%
p.removeUserDebugItem(Destination_id)
p.disconnect(cid)

# %%
# 按下“W”切换线框视角，按下“G”打开或关闭GUI组件。我们还可以自定义键盘事件和鼠标事件。
p.setDebugObjectColor(
    objectUniqueId=robot_id,
    linkIndex=-1,
    objectDebugColorRGB=[0, 0, 1]
)
