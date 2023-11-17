#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 15:07:08 2023

@author: lihui.liu
"""
import sys
sys.path.append('/home/lihui.liu/mnt/workspace/python/robot/robot_pybullet')
import pybullet as p
import pybullet_data
from time import sleep
from m_class import Thread_print
from queue import Queue
from threading import Event
import math

event = Event()
print_text_queue = Queue(50)
print_content_queue = Queue(50)
thread = Thread_print(event, run_time=10)
thread.start()
sleep(2)
event.set()
sleep(1)
event.clear()
thread.join()
print("end")
sleep(1)

use_gui = True
if use_gui:
    cid = p.connect(p.GUI)
else:
    cid = p.connect(p.DIRECT)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5], useMaximalCoordinates=False)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

p.resetDebugVisualizerCamera(cameraTargetPosition=[0,0,0],\
                             cameraDistance=6.8,\
                             cameraPitch=-41.2,\
                             cameraYaw=66.4) #转变视角
    
text = []
debug_text_id = p.addUserDebugText(
    text="",
    textPosition=[0,0,2],
    textColorRGB=[0,0,1],
    textSize=1.5
    )

print_text_queue.queue.clear()
thread = Thread_print(event, print_text_queue=print_text_queue, run_time=10000)
thread.start()
keys = 0
while True:
    p.stepSimulation()
    keys = p.getKeyboardEvents()
    # print(keys)
    sleep(1/10)
    # if (ord("w") in keys) and (keys[ord("w")] == p.KEY_WAS_TRIGGERED):
    if (ord("w") in keys) and (keys[ord("w")] == 2):
        print_text_queue.put("w KEY_WAS_TRIGGERED")
    elif ord("w") in keys and keys[ord("w")] == p.KEY_IS_DOWN:
        print_text_queue.put("w KEY_IS_DOWN")
    elif ord("w") in keys and keys[ord("w")] == p.KEY_WAS_RELEASED:
        print_text_queue.put("w KEY_WAS_RELEASED")
        event.set()
        sleep(1./2.)
        event.clear()
        print_text_queue.queue.clear()
        break

# %%
textColor = [1, 1, 0]
textPosition = [0, 0, 2]

print_text_queue.queue.clear()
thread = Thread_print(event, print_text_queue=print_text_queue, run_time=10000)
thread.start()
while True:
    p.stepSimulation()
    key_dict = p.getKeyboardEvents()
    if (ord("q") in key_dict) and (key_dict[ord("q")] & p.KEY_WAS_TRIGGERED):
        event.set()
        sleep(1./2.)
        event.clear()
        print_text_queue.queue.clear()
        break
    if len(key_dict):
        print_text_queue.put(key_dict)
        # sleep(1/3)
        # if p.B3G_BACKSPACE in key_dict and key_dict[p.B3G_BACKSPACE] & p.KEY_WAS_TRIGGERED:
        if -1 in key_dict and key_dict[-1] & p.KEY_WAS_TRIGGERED:
            if len(text) != 0:
                text.pop()
                debug_text_id = p.addUserDebugText(
                    text="".join(text),
                    textPosition=[0,0,2],
                    textColorRGB=[0,0,1],
                    textSize=1.5,
                    replaceItemUniqueId=debug_text_id
                    )
        else:
            for k, v in key_dict.items():
                if v & p.KEY_WAS_TRIGGERED:     # 只考虑刚刚按下的按键
                    if ord("a") <= k <= ord("z"):
                        text.append(chr(k))
                    elif k == p.B3G_SPACE:
                        text.append(" ")
                    elif ord("0") <= k <= ord("9"):
                        text.append(chr(k))

            debug_text_id = p.addUserDebugText(
                text="".join(text),
                textPosition=textPosition,
                textColorRGB=textColor,
                textSize=1.5,
                replaceItemUniqueId=debug_text_id
                )

# %%
for i in range(p.getNumJoints(robot_id)):
    if "wheel" in p.getJointInfo(robot_id, i)[1].decode("utf-8"):
        print(p.getJointInfo(robot_id, i)[:2])

maxV = 30
maxF = 30

t = 2   # 左前或右前的轮子的速度差的倍数
p.STATE_LOGGING_VIDEO_MP4
logging_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./log/keyboard2.mp4")

while True:
    p.stepSimulation()
    key_dict = p.getKeyboardEvents()
    
    if (ord("q") in key_dict) and (key_dict[ord("q")] & p.KEY_WAS_TRIGGERED):
        break
    
    mouse = p.getMouseEvents()
    # if len(mouse):
    #     print(mouse)
    
    if len(key_dict):
        if p.B3G_UP_ARROW in key_dict and p.B3G_LEFT_ARROW in key_dict:
            p.setJointMotorControlArray(    # 2,3为右 6,7为左
                bodyUniqueId=robot_id,
                jointIndices=[2, 3],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV, maxV],
                forces=[maxF, maxF]
            )
            p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV / t, maxV / t],
                forces=[maxF / t, maxF / t]
            )
            debug_text_id = p.addUserDebugText(
                text="up + left",
                textPosition=[0, 0, 2],
                textColorRGB=textColor,
                textSize=2.5,
                replaceItemUniqueId=debug_text_id
            )
        elif p.B3G_UP_ARROW in key_dict and p.B3G_RIGHT_ARROW in key_dict:  # 右前
            p.setJointMotorControlArray(   # 2,3为右 6,7为左
                bodyUniqueId=robot_id,
                jointIndices=[6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV, maxV],
                forces=[maxF, maxF]
            )
            p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[2, 3],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV / t, maxV / t],
                forces=[maxF / t, maxF / t]
            )
            debug_text_id = p.addUserDebugText(
                text="up + right",
                textPosition=[0, 0, 2],
                textColorRGB=textColor,
                textSize=2.5,
                replaceItemUniqueId=debug_text_id
            )
            
        elif p.B3G_UP_ARROW in key_dict:        # 向前
            p.setJointMotorControlArray(   
                bodyUniqueId=robot_id,
                jointIndices=[2, 3, 6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV, maxV, maxV, maxV],
                forces=[maxF, maxF, maxF, maxF]
            )
            debug_text_id = p.addUserDebugText(
                text="up",
                textPosition=[0, 0, 2],
                textColorRGB=textColor,
                textSize=2.5,
                replaceItemUniqueId=debug_text_id
            )
            
        elif p.B3G_DOWN_ARROW in key_dict:        # 向后
            p.setJointMotorControlArray(   
                bodyUniqueId=robot_id,
                jointIndices=[2, 3, 6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[-maxV / t, -maxV / t, -maxV / t, -maxV / t],
                forces=[maxF / t, maxF / t, maxF / t, maxF / t]
            )
            debug_text_id = p.addUserDebugText(
                text="down",
                textPosition=[0, 0, 2],
                textColorRGB=textColor,
                textSize=2.5,
                replaceItemUniqueId=debug_text_id
            )


        elif p.B3G_LEFT_ARROW in key_dict:        # 原地左转
            p.setJointMotorControlArray(   
                bodyUniqueId=robot_id,
                jointIndices=[2, 3, 6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV / t, maxV / t, -maxV / t, -maxV / t],
                forces=[maxF / t, maxF / t, maxF / t, maxF / t]
            )
            debug_text_id = p.addUserDebugText(
                text="left",
                textPosition=[0, 0, 2],
                textColorRGB=textColor,
                textSize=2.5,
                replaceItemUniqueId=debug_text_id
            )
            
        elif p.B3G_RIGHT_ARROW in key_dict:        # 原地右转
            p.setJointMotorControlArray(   
                bodyUniqueId=robot_id,
                jointIndices=[2, 3, 6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[-maxV / t, -maxV / t, maxV / t, maxV / t],
                forces=[maxF / t, maxF / t, maxF / t, maxF / t]
            )
            debug_text_id = p.addUserDebugText(
                text="right",
                textPosition=[0, 0, 2],
                textColorRGB=textColor,
                textSize=2.5,
                replaceItemUniqueId=debug_text_id
            )

    else:           # 没有按键，则停下
        p.setJointMotorControlArray(   
            bodyUniqueId=robot_id,
            jointIndices=[2, 3, 6, 7],
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=[0, 0, 0, 0],
            forces=[0, 0, 0, 0]
        )
        debug_text_id = p.addUserDebugText(
            text="",
            textPosition=[0, 0, 2],
            textColorRGB=textColor,
            textSize=2.5,
            replaceItemUniqueId=debug_text_id
        )

p.stopStateLogging(logging_id)

def getRayFromTo(mouseX, mouseY):
    width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera(
    )
    camPos = [
        camTarget[0] - dist * camForward[0], 
        camTarget[1] - dist * camForward[1],
        camTarget[2] - dist * camForward[2]
    ]
    farPlane = 10000
    rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
    invLen = farPlane * 1. / (math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] *
                                        rayForward[1] + rayForward[2] * rayForward[2]))
    rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
    rayFrom = camPos
    oneOverWidth = float(1) / float(width)
    oneOverHeight = float(1) / float(height)
    dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
    dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
    rayToCenter = [
        rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
    ]
    rayTo = [
        rayFrom[0] + rayForward[0] - 0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] -
        float(mouseY) * dVer[0], rayFrom[1] + rayForward[1] - 0.5 * horizon[1] + 0.5 * vertical[1] +
        float(mouseX) * dHor[1] - float(mouseY) * dVer[1], rayFrom[2] + rayForward[2] -
        0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
    ]
    return rayFrom, rayTo

# %%
p.disconnect(cid)

# %%
# 说明	按键ID常量
# F1到F12	B3G_F1 … B3G_F12
# 上下左右方向键	B3G_LEFT_ARROW, B3G_RIGHT_ARROW, B3G_UP_ARROW, B3G_DOWN_ARROW
# 同一页向上/下，页尾，起始页	B3G_PAGE_UP, B3G_PAGE_DOWN, B3G_PAGE_END, B3G_HOME
# 删除，插入，Alt，Shift，Ctrl，Enter，Backspace，空格	B3G_DELETE, B3G_INSERT, B3G_ALT, B3G_SHIFT, B3G_CONTROL, B3G_RETURN, B3G_BACKSPACE, B3G_SPACE






