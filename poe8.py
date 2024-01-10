#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 25 13:58:37 2023

@author: lihui.liu
"""
q1 = np.array([[0],[0],[0.27985]])
q2 = np.array([[0],[0],[0.27985]])
q3 = np.array([[0],[0],[0.27985]])
q4 = np.array([[-0.04951],[0],[0.27985+0.36330]])
q5 = np.array([[0],[0],[0.27985+0.36330+0.36665]])
q6 = np.array([[0],[0],[0.27985+0.36330+0.36665]])

w1 = np.array([[0],[0],[1]])
w2 = np.array([[0],[1],[0]])  
w3 = np.array([[0],[0],[1]])
w4 = np.array([[0],[-1],[0]])
w5 = np.array([[0],[0],[1]])
w6 = np.array([[0],[1],[0]])
M0 = np.array([[0, -1, 0, 0],[0, 0, 1, 0],[-1, 0, 0, 0.27985+0.36330+0.36665],[0, 0, 0, 1]])

DRV1_LOW = -2.85
DRV1_HIGH = 2.85
DRV2_LOW  = -2.181
DRV2_HIGH = 2.181
# DRV2_LOW  = -1.57
# DRV2_HIGH = 1.57
DRV3_LOW  = -3
DRV3_HIGH = 1.25
# DRV3_HIGH = 1.25
DRV4_LOW  = 0.3
DRV4_HIGH = 2.8
DRV5_LOW  = -2.70526
DRV5_HIGH = 2.70526
# DRV6_LOW  = 0.61
# DRV6_HIGH = 2.53
# DRV6_LOW  = -0.4
# DRV6_HIGH = 0.61
DRV6_LOW  = -np.pi/3
DRV6_HIGH = np.pi/3
DRV7_LOW  = -0.07
DRV7_HIGH = 0.07
DRV8_LOW  = -5.75
DRV8_HIGH = 5.75

# %%
joint_T_all = DHParameter().func_dh8_all(theta_i)
joint_T = DHParameter().func_dh(theta_i)
end_pos_new = DHParameter().DH_compute(theta_i)

d34 = 0.04951
d3e = d34 * sympy.tan(theta4/2)
d3e_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), d3e, "numpy")
d3e_v = d3e_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
d0 = np.array([0., 0, 0])
d30 = sympy.Matrix([(joint_T_all[2][0,3]), (joint_T_all[2][1,3]), (joint_T_all[2][2,3])])
r30 = joint_T_all[2]
dw = sympy.Matrix([joint_T_all[1][0,3], joint_T_all[1][1,3], joint_T_all[1][2,3]])
ds = sympy.Matrix([(joint_T_all[7][0,3]), (joint_T_all[7][1,3]), (joint_T_all[7][2,3])])
de = sympy.Matrix((joint_T_all[2] * sympy.Matrix([0, 0, d3e, 1]))[0:3])
dew = sympy.Matrix.norm(de-dw)
dew_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), dew, "numpy")
dew_v = dew_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])

d30_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), d30, "numpy")
r30_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), r30, "numpy")
de_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), de, "numpy")
d30_v = d30_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
r30_v = r30_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
de_v = de_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
r40 = joint_T_all[3][0:3,0:3]
r40_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), r40, "numpy")

joint_T_i = DHParameter().func_dh_i()
r64 = joint_T_i[4][0:3,0:3]*joint_T_i[5][0:3,0:3]
r74 = joint_T_i[4][0:3,0:3]*joint_T_i[5][0:3,0:3]*joint_T_i[6][0:3,0:3]
r84 = joint_T_i[4][0:3,0:3]*joint_T_i[5][0:3,0:3]*joint_T_i[6][0:3,0:3]*joint_T_i[7][0:3,0:3]

r64_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), r64, "numpy")
r64_v = r64_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
r74_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), r74, "numpy")
r74_v = r74_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
r84_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), r84, "numpy")
r84_v = r84_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])

# %%
L = [0.27985, 0, 0.36330, 0, 0.36665, 0, 0, 0.55443-0.16, 0.16, 0, 0]
L1 = 0.27985
d34 = 0.04951
L3 = 0.36330
L5 = 0.36665
L8 = 0.55443-0.16
d78 = 0.04050
theta_i = [0, 3.14159/3.8, 0, 3.1415926/1.5, 0, 0, 0, 0, 0, 0, 0]
theta_i = [0, 3.14159/3.8, 0, 3.1415926/1.5, 0, 0.5, 0, 0, 0, 0, 0]
theta_i = np.array([0, 0.7, 0.6, 2.3, -0.68, -0., -0.06, 0., 0.5, 0.5, 0.5])
# theta_i = np.array([0, 0., 0., 0, -0., 0., -0.0, 0., 0.5, 0.5, 0.5])
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i)
end_point_start = DHParameter().DH_compute(theta_i)
end_point_poe = PF().fk6(theta_i)
# p.addUserDebugPoints(pointPositions=[end_point[0:3,3]], pointColorsRGB=[[1,1,0]], pointSize=10)

ds6 = sympy.Matrix([(joint_T_all[4][0,3]), (joint_T_all[4][1,3]), (joint_T_all[4][2,3])])

ds6_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), ds6, "numpy")
ds6_v_s = ds6_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])

# %%
end6 = joint_T_all[5]
end8 = joint_T_all[7]
end6_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), end6, "numpy")
# end6_v = end6_np(theta_i_0[0],theta_i_0[1],theta_i_0[2],theta_i_0[3],theta_i_0[4],theta_i_0[5],theta_i_0[6],theta_i_0[7])
end6_v = end6_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])

end8_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), end8, "numpy")
# end8_v = end8_np(theta_i_0[0],theta_i_0[1],theta_i_0[2],theta_i_0[3],theta_i_0[4],theta_i_0[5],theta_i_0[6],theta_i_0[7])
end8_v = end8_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5]+np.pi/2,theta_i[6],theta_i[7])

end_bias = np.array([[1, 0, 0, 0.04050], [0, 1, 0, 0], [0, 0, 1, -0.55443+0.16+theta_i[6]], [0, 0, 0, 1]])
end0_d = np.dot(end_point_start, end_bias)
# ds6_r = np.dot(end0_d[0:3,0:3], np.array([[1, 0, 0],[0, 0, -1],[0, 1, 0]]))

# %%

ds6_v = end0_d[0:3,3].reshape(3,1)
p.addUserDebugPoints(pointPositions=[end0_d[0:3,3]], pointColorsRGB=[[1,1,0]], pointSize=10)

dw = np.array([[0], [0], [L1]])
ds6w_v = np.linalg.norm(ds6_v - dw)

dso4 = np.sqrt(d34**2 + L5**2)
dwo4 = np.sqrt(d34**2 + L3**2)
thetaO4 = np.arccos((dwo4**2 + dso4**2 - ds6w_v**2) / (2*dwo4*dso4))
thetaOd1 = np.arctan2(L3,d34)
thetaOd2 = np.arctan2(L5,d34)
theta42 = np.pi*2 - (np.pi*2-thetaO4) - thetaOd1 - thetaOd2
theta41 = np.pi*2 - thetaO4 - thetaOd1 - thetaOd2

theta4_0 = theta41

# %%
em4 = PF().eps(-theta4_0,3)
pm4 = em4 @ np.vstack((q3,1))

g1 = end_point_poe @ np.linalg.inv(M0)
q3tilde = np.vstack((q3,1))
pm6 = np.linalg.inv(g1) @ q3tilde

u_vec = pm6[0:3] - M0[0:3,3].reshape(3,1)
v_vec = pm4[0:3] - M0[0:3,3].reshape(3,1)

alpha_axis = w5.T @ v_vec
beta_axis = w6.T @ u_vec
gamma_axis = np.sqrt(np.linalg.norm(u_vec)**2-alpha_axis**2-beta_axis**2)
z_vec = alpha_axis*w5 + beta_axis*w6 + gamma_axis*(np.cross(w5.T,w6.T)).T
c_vec = z_vec + M0[0:3,3].reshape(3,1)

u_throw = u_vec - w6@w6.T@u_vec
v_throw = v_vec - w5@w5.T@v_vec

v6_throw = z_vec - w6@w6.T@z_vec
u5_throw = z_vec - w5@w5.T@z_vec

theta6_0 = np.arctan2(w6.T@(np.cross(u_throw.T,v6_throw.T).T), u_throw.T@v6_throw)
theta5_0 = np.arctan2(w5.T@(np.cross(u5_throw.T,v_throw.T).T), u5_throw.T@v_throw)

ems6 = PF().eps(-theta6_0,5)
ems5 = PF().eps(-theta5_0,4)
ems4 = PF().eps(-theta4_0,3)

p456 = g1 @ ems6 @ ems5 @ ems4
theta2_0 = np.arccos(p456[2,2])
theta1_0 = np.arctan2(p456[1,2],p456[0,2])
theta3_0 = np.arctan2(p456[2,1],-p456[2,0])

# %%
theta_i_0 = theta_i.copy()
theta_i_0[0:6] = theta1_0, theta2_0, theta3_0, theta4_0, theta5_0[0,0], theta6_0[0,0]
# theta_i_0[7] = theta8_0
print(theta1_0/6.28*360,'\n',theta2_0/6.28*360,'\n',theta3_0/6.28*360,'\n',theta4_0/6.28*360)
print(theta_i[0]/6.28*360,'\n',theta_i[1]/6.28*360,'\n',theta_i[2]/6.28*360,'\n',theta_i[3]/6.28*360)
print(theta5_0/6.28*360,'\n',(theta6_0)/6.28*360,'\n')
print(theta_i[4]/6.28*360,'\n',theta_i[5]/6.28*360,'\n')



end_point_poe = PF().fk6(theta_i)


# %%
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i_0)

theta_i = np.array([0, 0.7, 0.6, 2.3, -0.68, -0., -0.09, 0.8, 0.5, 0.5, 0.5])
theta_i = np.array([-0.166, 0.7422, 0.502, 2.478, -0.749, 0.29869, -0.0, 0.78, 0.5, 0.5, 0.5])
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i)
end_point_0 = DHParameter().DH_compute(theta_i)

# %%
# log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "/home/lihui.liu//mnt/workspace/python/robot/vedio/fixed_joint6.mp4")
theta_i = np.array([-0.166, 0.7422, 0.502, 2.478, -0.749, 0.29869, -0.09, 0.78, 0.5, 0.5, 0.5])
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i)
# end_bias = np.array([[1, 0, 0, 0.04050], [0, 1, 0, 0], [0, 0, 1, -0.55443+0.16+0.09], [0, 0, 0, 1]])
end_point_start = DHParameter().DH_compute(theta_i)

theta7_dir = end_point_start[2,3] - end_point_0[2,3]
if theta7_dir > 0:
    theta7_0 = -np.linalg.norm(end_point_start[0:3,3] - end_point_0[0:3,3])
else:
    theta7_0 = np.linalg.norm(end_point_start[0:3,3] - end_point_0[0:3,3])
# theta7_0 = 0
end_bias = np.array([[1, 0, 0, 0.04050], [0, 1, 0, 0], [0, 0, 1, -0.55443+0.16+theta7_0], [0, 0, 0, 1]])
                     
psi = theta_i[7]
theta8_0 = theta_i[7]
# for i in np.arange(0,np.pi,0.0001):
    # psi=i
sign8 = -1
sign = -1
signj = -1
    
step_x = 0.00
step_y = 0.00
step_z = 0.001
# step_x = 0.00
# step_y = -0.000
# step_z = 0.000
# step_y = -0.0001
# step_z = 0.0001
thetax = 0
thetay = 0.0
thetaz = 0.0
# thetax = 0.02

# end8_v = end_point_end
end_point_k = end_point_start
end_pos_new = DHParameter().DH_compute(theta_i)
end_pos_new_1 = end_pos_new
end_point_k_1 = end_point_k.copy()
ds6_v_1 = ds6_v
theta_i_0 = theta_i.copy()

theta7_list = []
theta7_dir_list = []

# %%
# log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "/home/lihui.liu//mnt/workspace/python/robot/vedio/poe71_run.mp4")

for i in range(3000):
    
    if i % 160 == 0:
        sign = -sign
    if i % 28 == 0:
        signj = -signj
    if i % 300 == 0:
        sign8 = -sign8
    if sign8 == 1:
        psi += 0.002
    else:
        psi -= 0.002

    theta8_0 = psi
    T_x = np.array([[1, 0, 0, step_x*sign],
                    [0, np.cos(thetax*derr*signj), -np.sin(thetax*derr*signj), step_y*sign],
                    [0, np.sin(thetax*derr*signj),  np.cos(thetax*derr*signj), step_z*sign],
                    [0, 0, 0, 1]])
    T_y = np.array([[np.cos(thetay*derr*signj), 0, np.sin(thetay*derr*signj), 0],
                    [0,               1, 0,            0],
                    [-np.sin(thetay*derr*signj), 0, np.cos(thetay*derr*signj), 0],
                    [0,               0, 0,            1]])
    T_z = np.array([[np.cos(thetaz*derr*signj), -np.sin(thetaz*derr*signj), 0, 0],
                    [np.sin(thetaz*derr*signj), np.cos(thetaz*derr*signj),    0, 0],
                    [0,            0,               1, 0],
                    [0,            0,               0, 1]])

    # end_point_k = np.matmul(end_point_k, T_step)
    end_point_k = np.matmul(end_point_k, T_z)
    end_point_k = np.matmul(end_point_k, T_y)
    end_point_k = np.matmul(end_point_k, T_x)
    # end_point_k = end_point[i]
    
    # theta_i[5] = theta_i[5] + np.pi/2
    # joint_T_v = joint_T_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
    # test_end_point_k = np.dot(joint_T_v,[0,0,-0.5,1])
    # np.random.normal(loc=0.0, scale=0.002, size=(3,4))
    # noise = np.random.normal(loc=0.0, scale=0.002, size=8)
    # noise = np.random.uniform(-0.002, 0.002, 8)

    # p.addUserDebugLine(end_pos_new_1[0:3,3], end_point_k[0:3,3], lineColorRGB=[0.3,0.2,0.6], lineWidth=1)
    # p.addUserDebugPoints(pointPositions=[end_point_k[0:3,3]], pointColorsRGB=[[0.3,1,0]], pointSize=8)
    # p.addUserDebugPoints(pointPositions=[end_point_0[0:3,3]], pointColorsRGB=[[0.8,0.1,0.7]], pointSize=8)
    end_pos_new_1 = end_point_k
    # start_time2 = time()
    # end_bias = np.array([[1, 0, 0, 0.04050], [0, 1, 0, 0], [0, 0, 1, -0.55443+0.16], [0, 0, 0, 1]])
    end_point_k8 = end_point_k @ np.array([[np.cos(-psi),-np.sin(-psi),0,0],[np.sin(-psi),np.cos(-psi),0,0],[0,0,1,0],[0,0,0,1]])
    
    theta7_dir = end_point_k8[2,3] - end_point_0[2,3]
    if theta7_dir > 0:
        theta7_0 = -np.linalg.norm(end_point_k8[0:3,3] - end_point_0[0:3,3])
    else:
        theta7_0 = np.linalg.norm(end_point_k8[0:3,3] - end_point_0[0:3,3])
        # break
    # theta7_0 = 0
    end_bias = np.array([[1, 0, 0, 0.04050], [0, 1, 0, 0], [0, 0, 1, -0.55443+0.16-theta7_0], [0, 0, 0, 1]])
    
    end0_d = np.dot(end_point_k8, end_bias)
    end0_d = end0_d @ np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]])
    ds6_v = end0_d[0:3,3].reshape(3,1)
    # p.addUserDebugPoints(pointPositions=[end0_d[0:3,3]], pointColorsRGB=[[1,1,0]], pointSize=10)
    
    dw = np.array([[0], [0], [L1]])
    ds6w_v = np.linalg.norm(ds6_v - dw)
    
    # dso4 = np.sqrt(d34**2 + L5**2)
    # dwo4 = np.sqrt(d34**2 + L3**2)
    thetaO4 = np.arccos((dwo4**2 + dso4**2 - ds6w_v**2) / (2*dwo4*dso4))
    thetaOd1 = np.arctan2(L3,d34)
    thetaOd2 = np.arctan2(L5,d34)
    theta42 = np.pi*2 - (np.pi*2-thetaO4) - thetaOd1 - thetaOd2
    theta41 = np.pi*2 - thetaO4 - thetaOd1 - thetaOd2
    theta4_0 = theta41

    em4 = PF().eps(-theta4_0,3)
    pm4 = em4 @ np.vstack((q3,1))
    
    g1 = end0_d @ np.linalg.inv(M0)
    q3tilde = np.vstack((q3,1))
    pm6 = np.linalg.inv(g1) @ q3tilde
    
    u_vec = pm6[0:3] - M0[0:3,3].reshape(3,1)
    v_vec = pm4[0:3] - M0[0:3,3].reshape(3,1)
    
    alpha_axis = w5.T @ v_vec
    beta_axis = w6.T @ u_vec
    gamma_axis = np.sqrt(np.linalg.norm(u_vec)**2-alpha_axis**2-beta_axis**2)
    z_vec = alpha_axis*w5 + beta_axis*w6 + gamma_axis*(np.cross(w5.T,w6.T)).T
    c_vec = z_vec + M0[0:3,3].reshape(3,1)
    
    u_throw = u_vec - w6@w6.T@u_vec
    v_throw = v_vec - w5@w5.T@v_vec
    
    v6_throw = z_vec - w6@w6.T@z_vec
    u5_throw = z_vec - w5@w5.T@z_vec
    
    theta6_0 = np.arctan2(w6.T@(np.cross(u_throw.T,v6_throw.T).T), u_throw.T@v6_throw)
    theta5_0 = np.arctan2(w5.T@(np.cross(u5_throw.T,v_throw.T).T), u5_throw.T@v_throw)
    
    ems6 = PF().eps(-theta6_0,5)
    ems5 = PF().eps(-theta5_0,4)
    ems4 = PF().eps(-theta4_0,3)
    
    p456 = g1 @ ems6 @ ems5 @ ems4
    theta2_0 = np.arccos(p456[2,2])
    theta1_0 = np.arctan2(p456[1,2],p456[0,2])
    theta3_0 = np.arctan2(p456[2,1],-p456[2,0])
    theta_i_0[0:6] = theta1_0, theta2_0, theta3_0, theta4_0, theta5_0[0,0], theta6_0[0,0]
    theta_i_0[6] = theta7_0
    theta_i_0[7] = theta8_0
    theta7_list.append(theta7_0)
    theta7_dir_list.append(theta7_dir)
    
    if not(DRV1_LOW < theta1_0 < DRV1_HIGH and DRV2_LOW < theta2_0 < DRV2_HIGH and DRV3_LOW < theta3_0 < DRV3_HIGH and \
        DRV4_LOW < theta4_0 < DRV4_HIGH and DRV5_LOW < theta5_0[0,0] < DRV5_HIGH and DRV6_LOW < theta6_0[0,0] < DRV6_HIGH):
        print('*************limit***************')
        break

    p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i_0)
    sleep(1./240.)

print('*************  The End  ***************')
# p.stopStateLogging(log_id)

# %%

p.removeAllUserDebugItems()

p.addUserDebugLine([0, 0, 0], [1, 0, 0], lineColorRGB=[1,0,0], lineWidth=5)
p.addUserDebugLine([0, 0, 0], [0, 1, 0], lineColorRGB=[0,1,0], lineWidth=5)
p.addUserDebugLine([0, 0, 0], [0, 0, 1], lineColorRGB=[0,1,0], lineWidth=5)

theta_i = [0, 3.14159/3.8, 0, 3.1415926/1.5, 0, 0, 0, 0, 0, 0, 0]
# theta_i = [0, 3.14159/3.8, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# theta_i = [0, 0.7, 0.8, 2.3, -0.5, 0.3, -0.06, 0.5, 0.5, 0.5, 0.5]

ds6 = sympy.Matrix([(joint_T_all[4][0,3]), (joint_T_all[4][1,3]), (joint_T_all[4][2,3])])

ds6_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), ds6, "numpy")
ds6_v = ds6_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i)


# %%

# theta_i[0] = DRV1_LOW*(pi/180) + (DRV1_HIGH-DRV1_LOW)*(pi/180)*rand

for i in range(20000):
    theta_i[0] = random.uniform(DRV1_LOW, DRV1_HIGH)
    theta_i[1] = random.uniform(DRV2_LOW, DRV2_HIGH)
    theta_i[2] = random.uniform(DRV3_LOW, DRV3_HIGH)
    theta_i[3] = random.uniform(DRV4_LOW, DRV4_HIGH)
    theta_i[4] = random.uniform(DRV5_LOW, DRV5_HIGH)
    theta_i[5] = random.uniform(DRV6_LOW, DRV6_HIGH)
    theta_i[6] = random.uniform(DRV7_LOW, DRV7_HIGH)
    theta_i[7] = random.uniform(DRV8_LOW, DRV8_HIGH)
    p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i)
    end_point_space = DHParameter().DH_compute(theta_i)
    p.addUserDebugPoints(pointPositions=[end_point_space[0:3,3]], pointColorsRGB=[[0.8,0.1,0.7]], pointSize=3)
    sleep(1./240.)
    if i == 500:
        # print(j,'\n')
        print('500')
    elif i == 1000:
        print('1000')
    elif i == 3000:
        print('3000')
    elif i == 6000:
        print('6000')
    elif i == 8000:
        print('8000')
    elif i == 11000:
        print('11000')
    elif i == 13000:
        print('13000')
    elif i == 16000:
        print('16000')
    elif i == 18000:
        print('18000')



# %%

p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i_0)
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i)

joint_T_all[2].subs([(theta1,theta_i_0[0]), (theta2,theta_i_0[1]), (theta3,theta_i_0[2])])
joint_T_all[0].subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2])])

joint_T_all[2].subs([(theta1,theta_i_tt[0]), (theta2,theta_i_tt[1]), (theta3,theta_i_tt[2])])


As = np.dot(u_sw0_x, R30_v)
Bs = np.dot(-np.dot(u_sw0_x, u_sw0_x), R30_v)
Cs = np.dot(np.dot(u_sw0, u_sw0.T), R30_v)

pl1 = p.addUserDebugLine(end_point[0:3,3], end6_v[0:3,3].reshape(3), lineColorRGB=[0.6,0.2,0.9], lineWidth=5)


a = (ds6_v).reshape(3)
b = se0 + dw.reshape(3)
c = a-b
xxxxx = xyz13.copy()
xxxxx[2] = xxxxx[2] + L1
# aa = (d30_v-dw).reshape(3)
pl1 = p.addUserDebugLine(a, dw.reshape(3), lineColorRGB=[0.6,0.2,0.9], lineWidth=5)
# pl2 = p.addUserDebugLine(aa, dw.reshape(3), lineColorRGB=[0.2,0.4,0.9], lineWidth=5)
pl3 = p.addUserDebugLine(e_l0, [0, 0, 0], lineColorRGB=[0.2,0.2,0.9], lineWidth=5)
pl4 = p.addUserDebugLine(b, dw.reshape(3), lineColorRGB=[0.6,0.2,0.9], lineWidth=5)
pl5 = p.addUserDebugLine(a, b, lineColorRGB=[0.6,0.2,0.9], lineWidth=5)
# pl6 = p.addUserDebugLine([-0.90976862, -0.41511573,  1.], dw.reshape(3), lineColorRGB=[0.2,0.6,0.4], lineWidth=5)
# pl7 = p.addUserDebugLine([-0.90976862, -0.41511573,  10.], dw.reshape(3), lineColorRGB=[0.2,0.6,0.4], lineWidth=5)
pl8 = p.addUserDebugLine(xxxxx, dw.reshape(3), lineColorRGB=[0.6,0.2,0.9], lineWidth=5)

p.removeUserDebugItem(pl6)

pl2 = p.addUserDebugLine([-4.13329534e-01, 0, 0], [-4.13329534e-01, 0, 1], lineColorRGB=[0.8,0.1,0.2], lineWidth=5)

p.removeUserDebugItem(pl7)

x = []
y = []
theta4_t = 0.3
x.append(theta4_t)
y.append(d34 * np.tan(theta4_t/2))
theta4_t = np.pi/6
x.append(theta4_t)
y.append(d34 * np.tan(theta4_t/2))
theta4_t = np.pi/4
x.append(theta4_t)
y.append(d34 * np.tan(theta4_t/2))
theta4_t = np.pi/3
x.append(theta4_t)
y.append(d34 * np.tan(theta4_t/2))
theta4_t = np.pi/2
x.append(theta4_t)
y.append(d34 * np.tan(theta4_t/2))
theta4_t = np.pi/3*2
x.append(theta4_t)
y.append(d34 * np.tan(theta4_t/2))
theta4_t = np.pi/4*3
x.append(theta4_t)
y.append(d34 * np.tan(theta4_t/2))
theta4_t = np.pi/6*5
x.append(theta4_t)
y.append(d34 * np.tan(theta4_t/2))
theta4_t = 2.8
x.append(theta4_t)
y.append(d34 * np.tan(theta4_t/2))
print(x)


d3e = d34 * np.tan(theta_i[3]/2)
dew = L3 + d3e
ds6e = L5 + d3e

np.dot(np.array([[1, 0, 0, 1],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]), r30_v)

x = []
y = []
z = []
for i in np.arange(0.3,2.8,0.01):
    x.append(i)
    y.append(d34 * np.tan(i/2))
    
    d3e = d34 * np.tan(theta_i[3]/2)
    dew = L3 + d3e
    ds6e = L5 + d3e
    
    dse = np.sqrt(dew**2 + ds6e**2 - 2*dew*ds6e*np.cos(np.pi-i))
    z.append(dse)

# from mpl_toolkits.mplot3d import Axes3D
# import numpy as np
# from matplotlib import pyplot as plt
# plt.rcParams['font.sans-serif'] = ['SimHei'] #用来正常显示中文标签
# plt.rcParams['axes.unicode_minus'] = False #用来正常显示负号

plt.xlabel('x')
plt.ylabel('y')
plt.title("一元一次函数")
plt.plot(x, z)
plt.show()


z1 = np.polyfit(z, x, 2) #用3次多项式拟合，输出系数从高到0
p1 = np.poly1d(z1) #使用次数合成多项式
y_pre = p1(z)
 
plt.plot(z,x,'.')
plt.plot(z,y_pre)
plt.show()

x = []
y = []
z = []
for i in np.arange(0.1347528670732862,0.8875642128840595,0.001):
    x.append(i)
    y.append(2.36409579*i**2 + 0.57104513*i + 0.26689191)

plt.plot(x, y)
plt.show()




theta4t = sympy.symbols('theta4t')
tg2 = (1-sympy.cos(theta4t))/sympy.sin(theta4t)

d3et = d34 * tg2
dewt = L3 + d3et
ds6et = L5 + d3et




eq = sympy.Eq(np.pi - sympy.acos(-(ds6w_v**2 - dewt**2 - ds6et**2) / (2 * dewt * ds6et)), theta4t)
theta4_0t = sympy.solve(eq, theta4t)


theta_i_tt = theta_i_0.copy()
theta_i_tt[0] = -1.1
theta_i_tt[5] = theta_i_tt[5]
theta_i_tt[2] = theta_i_tt[2] - 1.1
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i_tt)
end_pos_new = DHParameter().DH_compute(theta_i_tt)




As = u_sw0_x * R30
Bs = -(u_sw0_x * u_sw0_x) * R30
Cs = u_sw0 * u_sw0.T * R30

# theta1 = sympy.atan2(-sympy.sign(sympy.sin(theta2))*(As[1,1]*sympy.sin(psi) + Bs[1,1]*sympy.cos(psi) + Cs[1,1]),
#                      -sympy.sign(sympy.sin(theta2))*(As[0,1]*sympy.sin(psi) + Bs[0,1]*sympy.cos(psi) + Cs[0,1]))
# theta2 = sympy.acos(-As[2,1]*sympy.sin(psi) - Bs[2,1]*sympy.cos(psi) - Cs[2,1])
# theta3 = sympy.atan2(sympy.sign(sympy.sin(theta2))*(As[2,2]*sympy.sin(psi) + Bs[2,2]*sympy.cos(psi) + Cs[2,2]),
#                      -sympy.sign(sympy.sin(theta2))*(As[2,0]*sympy.sin(psi) + Bs[2,0]*sympy.cos(psi) + Cs[2,0]))

theta1 = sympy.symbols('theta1')
theta2 = sympy.symbols('theta2')
theta3 = sympy.symbols('theta3')
theta4 = sympy.symbols('theta4')
theta5 = sympy.symbols('theta5')
theta6 = sympy.symbols('theta6')
theta7 = sympy.symbols('theta7')
theta8 = sympy.symbols('theta8')

args = sympy.Matrix([theta1, theta2, theta3])

psi = 0
f1 = sympy.atan2(-sympy.sign(sympy.sin(theta2))*(As[1,1]*sympy.sin(psi) + Bs[1,1]*sympy.cos(psi) + Cs[1,1]),
                  -sympy.sign(sympy.sin(theta2))*(As[0,1]*sympy.sin(psi) + Bs[0,1]*sympy.cos(psi) + Cs[0,1])) - theta1
f2 = sympy.acos(-As[2,1]*sympy.sin(psi) - Bs[2,1]*sympy.cos(psi) - Cs[2,1]) - theta2
f3 = sympy.atan2(sympy.sign(sympy.sin(theta2))*(As[2,2]*sympy.sin(psi) + Bs[2,2]*sympy.cos(psi) + Cs[2,2]),
                -sympy.sign(sympy.sin(theta2))*(As[2,0]*sympy.sin(psi) + Bs[2,0]*sympy.cos(psi) + Cs[2,0])) - theta3
# f1 = sympy.atan2(-1*(As[1,1]*sympy.sin(psi) + Bs[1,1]*sympy.cos(psi) + Cs[1,1]),
#                   -1*(As[0,1]*sympy.sin(psi) + Bs[0,1]*sympy.cos(psi) + Cs[0,1])) - theta1
# f2 = sympy.acos(-As[2,1]*sympy.sin(psi) - Bs[2,1]*sympy.cos(psi) - Cs[2,1]) - theta2
# f3 = sympy.atan2((As[2,2]*sympy.sin(psi) + Bs[2,2]*sympy.cos(psi) + Cs[2,2]),
#                 -1*(As[2,0]*sympy.sin(psi) + Bs[2,0]*sympy.cos(psi) + Cs[2,0])) - theta3

# f1.subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2])])
# f1.subs([(theta1,theta_i_o[0]), (theta2,theta_i_o[1]), (theta3,theta_i_o[2])])

funcs = sympy.Matrix([f1, f2, f3])
res = funcs.jacobian(args)

f_jacobian = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), res, "numpy")
f_funcs = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), funcs, "numpy")

# f_jacobian1 = sympy.lambdify(('theta1','theta2','theta3'), res[0], "numpy")
# f_jacobian2 = sympy.lambdify(('theta1','theta2','theta3'), res, "numpy")
# f_jacobian3 = sympy.lambdify(('theta1','theta2','theta3'), res, "numpy")

# JointJacobian1 = f_jacobian1(theta_i[0],theta_i[1],theta_i[2])
theta_i = [0, 3.14159/6, 0, 3.1415926/3, 0, 0, 0, 0, 0, 0, 0]
theta_i = [0.5, 1, 2, 3.1415926/3, 0, 0, 0, 0, 0, 0, 0]
f_theta_i = f_funcs(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
err_list = []
theta_i_list = theta_i_o
for i in range(10000):
    # JointJacobian = f_jacobian(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
    JointJacobian = res.subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2])])

    JointJacobianNpPinv = np.linalg.pinv(np.array(JointJacobian).astype(float))
    # JointJacobianNpPinv = np.linalg.pinv(JointJacobian)
    theta_i[0:3] = theta_i[0:3] - np.matmul(JointJacobianNpPinv, f_theta_i).T[0]
    f_theta_i = f_funcs(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
    

    err = np.linalg.norm(f_theta_i,2)

    err_list.append(err)
    theta_i_list.append(theta_i)
    # print(theta_i)
    # print(err_robot)
    if err < 3e-8:
        # print('+' * 50)
        # print('i = ',i)
        break
    if i == 500:
        print(j,'\n',i)
        print('500')
    elif i == 1000:
        print('1000')
    elif i == 3000:
        print('3000')
    elif i == 6000:
        print('6000')








# %%

As.subs([(theta1,theta_i[0]), (theta2,theta_i[1]), (theta3,theta_i[2])])




aa = np.array([[1,2,3],[4,5,6]])
bb = np.array([[4,5,6]])
cc = np.dot(aa,bb.T)
dd = aa * bb
ee = sympy.Matrix([[1,2,3],[4,5,6]])
ff = sympy.Matrix([[4,5,6],[1,2,3],[1,2,3]])
gg = sympy.dot(aa,bb.T)
hh = ee * ff
ii = np.dot(ee,ff)

eqs = [sympy.Eq(sympy.atan2(-sympy.sign(sympy.sin(theta2))*(As[1,1]*sympy.sin(psi) + Bs[1,1]*sympy.cos(psi) + Cs[1,1]),
                     -sympy.sign(sympy.sin(theta2))*(As[0,1]*sympy.sin(psi) + Bs[0,1]*sympy.cos(psi) + Cs[0,1])), theta1),
       sympy.Eq(sympy.acos(-As[2,1]*sympy.sin(psi) - Bs[2,1]*sympy.cos(psi) - Cs[2,1]), theta2),
       sympy.Eq(sympy.atan2(sympy.sign(sympy.sin(theta2))*(As[2,2]*sympy.sin(psi) + Bs[2,2]*sympy.cos(psi) + Cs[2,2]),
                            -sympy.sign(sympy.sin(theta2))*(As[2,0]*sympy.sin(psi) + Bs[2,0]*sympy.cos(psi) + Cs[2,0])), theta3)]
print(sympy.solve(eqs, [theta1, theta2, theta3]))



eqs = [sympy.Eq(sympy.atan2(-1*(Bs[1,1] + Cs[1,1]),
                            -1*(Bs[0,1] + Cs[0,1])), theta1),
       sympy.Eq(sympy.acos( - Bs[2,1] - Cs[2,1]), theta2),
       sympy.Eq(sympy.atan2((Bs[2,2] + Cs[2,2]),
                            -1*(Bs[2,0] + Cs[2,0])), theta3)]
print(sympy.solve(eqs, [theta1, theta2, theta3]))



tt = joint_T_all[3] * sympy.Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d3e], [0, 0, 0, 1]])
tt_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), tt, "numpy")
tt_v = tt_np(theta_i[0],theta_i[1],0,theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])


theta4 = sympy.symbols('theta4')
print(sympy.solve(eq, theta4))
aa = sympy.solve(eq, theta4)

a, b, c = sympy.symbols('a b c')

eqs = [sympy.Eq(0.5*b/(sympy.exp(a*(1-0.5))-1), 8000),
       sympy.Eq(0.8*b/(sympy.exp(a*(1-0.8))-1), 100000)]
       # sympy.Eq(-x - y + 5 * z, 42)]
print(sympy.solve(eqs, [a, b, c]))



tt = joint_T_all[3]
tt_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), tt, "numpy")
tt_v = tt_np(theta_i[0],theta_i[1],0,theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
a = tt_v[0:3,3]
b = a - np.array([0, 0, L1])
c = np.linalg.norm(b)
d = np.linalg.norm(ds6_v.T - a)
o4 = np.arccos((c**2 + d**2 - ds6w_v**2) / 2*c*d)
theta4 = np.pi*2 - o4 - thetaOd1 - thetaOd2
print(o4/np.pi*360/2,'\n',thetaOd1/np.pi/2*360,'\n',thetaOd2/np.pi/2*360,'\n',theta4/np.pi/2*360)



a = np.array([[1],[2],[3]])
b = np.array([[1.8,5.2,3.2]])

c = np.dot(a,b)
np.linalg.matrix_rank(c)





dse = sympy.Matrix.norm(ds-de)
dew = sympy.Matrix.norm(de-dw)
dsw = sympy.Matrix.norm(ds-dw)


dw = T0 * T1
de = T0 * T1 * T2 * T3 + d_theta
ds = T

cos(theta4) = (dsw2 - dse2 - des2) / (2*(dse*dew))


theta_i = [0, 3.14159/6, 0, 3.1415926/3, 0, 0, 0, 0, 0, 0, 0]

p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i)



aa = sympy.Matrix([1, 2, 3])
bb = sympy.Matrix([1, 4, 6])

sympy.Matrix.norm(aa-bb)



a = np.array([[1, 2, 3, 4],
              [5, 6, 7, 8],
              [1, 0, 1, 2],
              [3, 2, 0, 1]])

b = np.array([[4, 3, 2, 1],
              [8, 7, 6, 5],
              [0, 1, 1, 0],
              [0, 1, 2, 0]])



