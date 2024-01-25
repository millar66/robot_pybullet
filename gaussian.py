#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 16:53:34 2024

@author: lihui.liu
"""

import numpy as np
import matplotlib  
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

t_symple = 0.04
x = np.arange(-0, 30, t_symple)
# y = np.arange(-0, 30, t_symple)
# a = A; b = avg;
u1 = 1.818
delt1 = 0.09
y1 = (1/np.sqrt(2*np.pi*delt1))*np.exp(-(2.818-40/x)**2/(2*delt1))

u2 = 20
delt2 = 9
y2 = (1/np.sqrt(2*np.pi*delt2))*np.exp(-(x-u2)**2/(2*delt2))

p_x = (1/np.sqrt(2*np.pi*delt2))*np.exp(-(20-u2)**2/(2*delt2))

y3 = y1*y2*6

p_x1_sum = 0
for i in y1:
    p_x1_sum = p_x1_sum + i
p_x1_sum = p_x1_sum * t_symple
print(p_x1_sum)

p_x2_sum = 0
for i in y2:
    p_x2_sum = p_x2_sum + i
p_x2_sum = p_x2_sum * t_symple
print(p_x2_sum)


y4 = np.exp((y1)*2)/4 + (y1)**2 + np.log(y1)/20

# G4 = (np.exp((b1)*2)/2 + 2*(b1) + 1/(b1*np.log(b1)*20))
# y5 = ((np.exp((b1)*2)/4+(b1)**2+np.log(b1)/20) + G4 * (y1-b1))

plt.xlabel('x')
plt.ylabel('y')
plt.title("pos")
# plt.plot(x, y1, 'b')
plt.plot(x, y2, 'r')
plt.plot(x, y3)
# plt.plot(x, y4)
# plt.plot(x, y5)
plt.show()

# shannon information
p_x1 = 0.9
p_x2 = 0.5
h_x1 = (-(p_x1*np.log(p_x1) + (1-p_x1)*np.log(1-p_x1)))
h_x2 = (-(p_x2*np.log(p_x2) + (1-p_x2)*np.log(1-p_x2)))

# mutual information


# LDU--UDL
A11 = np.array([[1,2,3],[2,5,7],[3,8,1]])
B12 = np.array([[1,2,3],[4,5,6],[7,8,9]])
C21 = np.array([[1,2,3],[4,5,6],[7,8,9]])
D22 = np.array([[1.5,2.3,3.8],[6,5.3,9],[7.5,1,10]])

SMW = np.vstack((np.hstack((np.linalg.inv(A11), -B12)), np.hstack((C21, D22))))

LDU1 = np.vstack((np.hstack((np.eye(3), np.zeros((3,3)))), np.hstack((C21@A11, np.eye(3)))))
LDU2 = np.vstack((np.hstack((np.linalg.inv(A11), np.zeros((3,3)))), np.hstack((np.zeros((3,3)), D22+C21@A11@B12))))
LDU3 = np.vstack((np.hstack((np.eye(3), -A11@B12)), np.hstack((np.zeros((3,3)), np.eye(3)))))

LDU1 @ LDU2 @ LDU3 == SMW

UDL1 = np.vstack((np.hstack((np.eye(3), -B12 @ np.linalg.inv(D22))), np.hstack((np.zeros((3,3)), np.eye(3)))))
UDL2 = np.vstack((np.hstack((np.linalg.inv(A11)+B12 @ np.linalg.inv(D22)@C21, np.zeros((3,3)))), np.hstack((np.zeros((3,3)), D22))))
UDL3 = np.vstack((np.hstack((np.eye(3), np.zeros((3,3)))), np.hstack((np.linalg.inv(D22)@C21, np.eye(3)))))

UDL1 @ UDL2 @ UDL3 == SMW

# SMW ===










