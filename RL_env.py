#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 16 16:10:56 2023

@author: lihui.liu
"""

import gym
from time import sleep
# import gym.envs.classic_control.cartpole as cartpole
# env = cartpole.CartPoleEnv()
env = gym.make("CartPole-v1", render_mode="human")  # 构建实验环境

for episode in range(10):
    env.reset() # 重置一个回合
    print("Episode finished after {} timesteps".format(episode))
    for ik in range(100):
        env.render() # 显示图形界面
        action = env.action_space.sample() # 从动作空间中随机选取一个动作
        observation, reward, done, info, _ = env.step(action)  # 用于提交动作，括号内是具体的动作
        if done:
            break
        sleep(0.02)
    
env.close()
