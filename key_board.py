#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 15:07:08 2023

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

while True:
    p.stepSimulation()
    keys = p.getKeyboardEvents()
    print(keys)
    sleep(1/3)
    if (ord("w") in keys) and (keys[ord("w")] & p.KEY_WAS_TRIGGERED):
        print("w KEY_WAS_TRIGGERED")
    elif ord("w") in keys and keys[ord("w")] & p.KEY_IS_DOWN:
        print("w KEY_IS_DOWN")
    elif ord("w") in keys and keys[ord("w")] & p.KEY_WAS_RELEASED:
        print("w KEY_WAS_RELEASED")
        break

# %%
while True:
    p.stepSimulation()
    key_dict = p.getKeyboardEvents()
    if (ord("q") in key_dict) and (key_dict[ord("q")] & p.KEY_WAS_TRIGGERED):
        break
    if len(key_dict):
        print(key_dict)
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
                textPosition=[0, 0, 2],
                textColorRGB=[0, 0, 1],
                textSize=1.5,
                replaceItemUniqueId=debug_text_id
                )






# %%
p.disconnect(cid)

# %%
# 说明	按键ID常量
# F1到F12	B3G_F1 … B3G_F12
# 上下左右方向键	B3G_LEFT_ARROW, B3G_RIGHT_ARROW, B3G_UP_ARROW, B3G_DOWN_ARROW
# 同一页向上/下，页尾，起始页	B3G_PAGE_UP, B3G_PAGE_DOWN, B3G_PAGE_END, B3G_HOME
# 删除，插入，Alt，Shift，Ctrl，Enter，Backspace，空格	B3G_DELETE, B3G_INSERT, B3G_ALT, B3G_SHIFT, B3G_CONTROL, B3G_RETURN, B3G_BACKSPACE, B3G_SPACE






