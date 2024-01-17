#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 14:14:44 2024

@author: lihui.liu
"""

import numpy as np

h_init = 0.
v_init = 0
h_final = 10
v_final = 0

h_min = 0
h_max = 10
N_h = 5
v_min = 0
v_max = 3
N_v = 3

Hd = np.linspace(h_min, h_max, N_h+1)
Vd = np.linspace(v_min, v_max, N_v+1)

u_min = -3
u_max = 2

J_costtogo = np.zeros([N_h+1, N_v+1])

Input_acc = np.zeros([N_h+1, N_v+1])

v_avg = 0.5 * (v_final+Vd)
T_delta = (h_max-h_min) / (N_h*v_avg)

