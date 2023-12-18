#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Dec 14 10:40:10 2023

@author: lihui.liu
"""

T_xyz = np.array([[1, 0, 0, step_x],
                  [0, 1, 0, step_y],
                  [0, 0, 1, step_z],
                  [0, 0, 0, 1]])

T_roll = np.array([[1, 0,            0,               0],
                   [0, orn_cos_x, orn_sin_x*-1, 0],
                   [0, orn_sin_x, orn_cos_x,    0],
                   [0, 0,            0,               1]])
T_pitch = np.array([[orn_cos_y,    0, orn_sin_y, 0],
                    [0,               1, 0,            0],
                    [orn_sin_y*-1, 0, orn_cos_y, 0],
                    [0,               0, 0,            1]])
T_yaw = np.array([[orn_cos_z, orn_sin_z*-1, 0, 0],
                  [orn_sin_z, orn_cos_z,    0, 0],
                  [0,            0,               1, 0],
                  [0,            0,               0, 1]])

T_all = np.dot(T_roll, np.dot(T_pitch,T_yaw))
# theta_i = [-1.1, 0.45272920, 1.142726-1.1, 1.909714, -0.35656702, 0.4630274, 0, 1.2834317, 0, 0, 0]

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
theta_i = [-1.14272, 0.452729, 0.0, 1.909714728, -0.325663, 0.491105, 0, 1.1958022, 0, 0, 0]
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i)
# theta_i = [0.0, 0.8043512, -0.4094627, 2.08250059, -0.000558530805, 0.52248341, 0, -0.40918375, 0, 0, 0]
# theta_i = [0, 3.14159/3.8, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# theta_i = [0, 0.7, 0.8, 2.3, -0.5, 0.3, -0.06, 0.5, 0.5, 0.5, 0.5]

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
# end8_v = np.array([[ 0.29851418,  0.        ,  0.9544052 ,  0.26295908],
#                    [ 0.        , -1.        ,  0.        ,  0.        ],
#                    [ 0.9544052 ,  0.        , -0.29851418,  0.56263871],
#                    [ 0.        ,  0.        ,  0.        ,  1.        ]])
# end8_v = np.array([[ 6.94454039e-01,  2.06981063e-16, -7.19537065e-01, -4.13329534e-01],
#                    [ 1.54421550e-16, -1.00000000e+00, -1.38620230e-16, -1.05227436e-16],
#                    [-7.19537065e-01, -1.48466495e-17, -6.94454039e-01, 4.74262818e-01],
#                    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
end_point = np.array([[ 0.69445404,  0.        , -0.71953706, -4.13329534e-01],
                      [ 0.        , -1.        , -0.        ,  0.22222222],
                      [-0.71953706,  0.        , -0.69445404,  4.74262818e-01],
                      [ 0.        ,  0.        ,  0.        ,  1.        ]])
end_point = np.array([[-0.64029595, -0.76779092,  0.02276398, -0.11186384],
                      [-0.58731913,  0.50846206,  0.62970038,  0.49846492],
                      [-0.49505286,  0.38982488, -0.77650449,  0.43479268],
                      [ 0.        ,  0.        ,  0.        ,  1.        ]])

end_bias = np.array([[1, 0, 0, 0.04050], [0, 1, 0, 0], [0, 0, 1, -0.55443+0.16], [0, 0, 0, 1]])
end0_d = np.dot(end_point, end_bias)
# ds6_r = np.dot(end0_d[0:3,0:3], np.array([[1, 0, 0],[0, 0, -1],[0, 1, 0]]))

# %% test
end_pos_new = np.array([[-0.64029595, -0.76779092,  0.02276398, -0.11186384],
                        [-0.58731913,  0.50846206,  0.62970038,  0.49846492],
                        [-0.49505286,  0.38982488, -0.77650449,  0.43479268],
                        [ 0.        ,  0.        ,  0.        ,  1.        ]])

aa = np.dot(end_pos_new, [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.55443+0.16], [0, 0, 0, 1]])
bb = np.dot(aa, [[1, 0, 0, 0.04050], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
cc = np.dot(aa, [[1, 0, 0, 0], [0, 1, 0, -0.04050], [0, 0, 1, 0], [0, 0, 0, 1]])
# p.addUserDebugLine(end_point_k[0:3,3], end_point_k_1[0:3,3], lineColorRGB=[0.8,0.1,0.2], lineWidth=5)
p.addUserDebugPoints(pointPositions=[aa[0:3,3]], pointColorsRGB=[[1,1,0]], pointSize=10)
p.addUserDebugPoints(pointPositions=[bb[0:3,3]], pointColorsRGB=[[1,1,0]], pointSize=10)
p.addUserDebugPoints(pointPositions=[cc[0:3,3]], pointColorsRGB=[[1,1,0]], pointSize=10)

T8 = np.array([[np.cos(theta_i[7]),np.sin(theta_i[7]),0,0],
              [-np.sin(theta_i[7]),np.cos(theta_i[7]),0,0],
              [0, 0, 1,0],
              [0, 0, 0, 1]])
np.dot(cc,T8)

z8 = end_pos_new[0:3,3] - aa[0:3,3]
z8_1 = z8/np.linalg.norm(z8)
l8 = z8/np.linalg.norm(z8)
# l8 = np.cross(sv0, sw0)
# e_l[2] = e_l[2] + L1
# e_l0 = e_l/np.linalg.norm(e_l)
# e_l0[2] = e_l0[2]
# e_l0 = np.array([0, 1, 0])
l8_x = l8[0]
l8_y = l8[1]
l8_z = l8[2]
l8_X = np.array([[0, -l8_z, l8_y],
                 [l8_z, 0, -l8_x],
                 [-l8_y, l8_x, 0]])
l8_t = I3 + l8_X * np.sin(-theta_i[7]) + np.dot(l8_X, l8_X) * (1 - np.cos(-theta_i[7]))

dd = np.dot(l8_t,(bb[0:3,3]-aa[0:3,3]).T) + aa[0:3,3]
p.addUserDebugPoints(pointPositions=[dd], pointColorsRGB=[[0.5,0.2,1]], pointSize=10)


# %%
# d3e = d34 * sympy.tan(theta4/2)
# dew = L3 + d3e
# ds6e = L5 + d3e

# dew_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), dew, "numpy")
# dew_v = dew_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
# ds6e_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), ds6e, "numpy")
# ds6e_v = ds6e_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])

# ds6 = sympy.Matrix([(joint_T_all[4][0,3]), (joint_T_all[4][1,3]), (joint_T_all[4][2,3])])
ds6_v = end0_d[0:3,3].reshape(3,1)
# ds6_v = np.array([[-0.10139713],
#                   [ 0.        ],
#                   [ 0.71903507]])
# ds6w = sympy.Matrix.norm(ds6-dw)
# ds6_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), ds6, "numpy")
# ds6_v = ds6_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
# ds6w_np = sympy.lambdify(('theta1','theta2','theta3','theta4','theta5','theta6','theta7','theta8'), ds6w, "numpy")
# ds6w_v = ds6w_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
dw = np.array([[0], [0], [L1]])
ds6w_v = np.linalg.norm(ds6_v - dw)
# theta4_0 = (-2.59824312*ds6w_v**2 + -0.3155959*ds6w_v + 2.81212862)
# d3e = d34 * np.tan(theta4_0/2)
# dew = L3 + d3e
# ds6e = L5 + d3e

dso4 = np.sqrt(d34**2 + L5**2)
dwo4 = np.sqrt(d34**2 + L3**2)
thetaO4 = np.arccos((dwo4**2 + dso4**2 - ds6w_v**2) / (2*dwo4*dso4))
thetaOd1 = np.arctan2(L3,d34)
thetaOd2 = np.arctan2(L5,d34)
theta42 = np.pi*2 - (np.pi*2-thetaO4) - thetaOd1 - thetaOd2
theta41 = np.pi*2 - thetaO4 - thetaOd1 - thetaOd2
# print(thetaO4/np.pi*360/2,'\n',thetaOd1/np.pi/2*360,'\n',thetaOd2/np.pi/2*360,'\n',theta41/np.pi/2*360,'\n',theta42/np.pi/2*360)

# %%
# theta4_0 = np.pi - np.arccos(-(ds6w_v**2 - ds6e**2 - dew**2) / (2 * ds6e * dew))
theta4_0 = theta41
alpha1 = np.arctan2(d34,L3)
alpha2 = np.arccos((dwo4**2 + ds6w_v**2 - dso4**2) / (2 * dwo4 * ds6w_v))
alpha_0 = -alpha1 - alpha2
# alpha = np.arccos(L3/ds6w_v)
# altha = sympy.symbols('altha')
# if ds6e_v**2 > ds6w_v**2 + dew_v**2:
#     # alpha = np.pi - np.arccos(-(ds6w_v**2 - ds6e_v**2 - dew_v**2) / (2 * ds6e_v * dew_v))
#     eq = sympy.Eq(np.pi - np.arccos(-(ds6e_v**2 - ds6w_v**2 - dew_v**2) / (2 * ds6w_v * dew_v)), alpha)
#     alpha_0 = sympy.solve(eq, alpha)
# else:
#     # alpha = np.pi - np.arccos((ds6w_v**2 - ds6e_v**2 - dew_v**2) / (2 * ds6e_v * dew_v))
#     eq = sympy.Eq(np.pi - np.arccos((ds6e_v**2 - ds6w_v**2 - dew_v**2) / (2 * ds6w_v * dew_v)), alpha)
#     alpha_0 = sympy.solve(eq, alpha)
# theta4_0 = np.array(theta4_0).astype(float)
# alpha_0 = np.array(alpha_0).astype(float)

# alpha_0 = -np.arccos(np.sqrt(0.20176883**2+0.44219705**2) / 0.87392855)
# psi = sympy.symbols('psi')
I3 = np.eye(3)

# u_sw0 = (ds6-dw)/(ds6-dw).norm()
u_sw0 = ((ds6_v-dw)/np.linalg.norm(ds6_v-dw)).reshape(3)
u_sw0_x = np.array([[0, -u_sw0[2], u_sw0[1]],
                    [u_sw0[2], 0, -u_sw0[0]],
                    [-u_sw0[1], u_sw0[0], 0]])
# u_sw0_x = u_sw0_x/np.linalg.norm(u_sw0_x)
# R_sw_psi = I3 + u_sw0_x * sympy.sin(-psi) + u_sw0_x*u_sw0_x*(1-sympy.cos(-psi))
# R_sw_psi = sympy.Matrix(I3 + u_sw0_x * sympy.sin(psi) + u_sw0_x*u_sw0_x*(1-sympy.cos(psi)))
# R_sw_psi.subs(psi, np.pi/30)
sv0 = np.array([0, 0, 1])
sw0 = ((ds6_v-dw)/np.linalg.norm(ds6_v-dw)).reshape(3)

e_l = np.cross(sv0, sw0)
# e_l[2] = e_l[2] + L1
e_l0 = e_l/np.linalg.norm(e_l)
# e_l0[2] = e_l0[2]
# e_l0 = np.array([0, 1, 0])
e_l_x = e_l0[0]
e_l_y = e_l0[1]
e_l_z = e_l0[2]

e_l_X = np.array([[0, -e_l_z, e_l_y],
                  [e_l_z, 0, -e_l_x],
                  [-e_l_y, e_l_x, 0]])

R_l_alpha = I3 + e_l_X * np.sin(alpha_0) + np.dot(e_l_X, e_l_X) * (1 - np.cos(alpha_0))

d3e0 = d34 * np.tan(theta4_0/2)
dew0 = L3 + d3e0
se0 = dew0 * np.dot(R_l_alpha, sw0)

r30_v = r30_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
# array([[ 0.67728209, -0.        ,  0.73572344],
#        [ 0.        ,  1.        ,  0.        ],
#        [-0.73572344,  0.        ,  0.67728209]

# z300 = (se0/np.linalg.norm(se0))
# # y300 = -np.sign(np.sin(theta4_0)) * e_l0
# y300 = e_l0
# x300 = np.cross(y300, z300)
z300 = (se0/np.linalg.norm(se0))
x30 = z300[0]
y30 = z300[1]
z30 = z300[2]
alpha13 = -np.arctan2(np.sqrt(x30**2+y30**2),z30)
# sv13 = np.array([0, 0, 1])
# e_l13 = np.cross(se0, sv13)
# e_l130 = e_l13/np.linalg.norm(e_l13)
# q1 = np.quaternion(1,2,3,4)
# q2 = quaternion.from_float_array([1,2,3,4])
# q3 = quaternion.from_rotation_matrix([[1,2,3],[1,2,3],[1,2,3]])
# q4 = quaternion.from_euler_angles([1,2,3])
# print(q1,q2,q3,q4)
# e_l_x13 = e_l130[0]
# e_l_y13 = e_l130[1]
# e_l_z13 = e_l130[2]
# e_l_X13 = np.array([[0, -e_l_z13, e_l_y13],
#                     [e_l_z13, 0, -e_l_x13],
#                     [-e_l_y13, e_l_x13, 0]])
# R3001 = (I3 + e_l_X13 * np.sin(alpha13) + np.dot(e_l_X13, e_l_X13) * (1 - np.cos(alpha13)))
R3001 = (I3 + e_l_X * np.sin(alpha13) + np.dot(e_l_X, e_l_X) * (1 - np.cos(alpha13))).T
y13 = np.dot([0, 1, 0],R3001)
y13_5 = np.linalg.norm(e_l0-y13)
alphay13 = np.arccos((2-y13_5**2)/2)
e_l_Xy13 = np.array([[0, -z30, y30],
                     [z30, 0, -x30],
                     [-y30, x30, 0]])
R3002 = (I3 + e_l_Xy13 * np.sin(alphay13) + np.dot(e_l_Xy13, e_l_Xy13) * (1 - np.cos(alphay13))).T
R3003 = np.dot(R3001,R3002).T
# R300 = R3001
# p.addUserDebugLine([y13[0],y13[1],y13[2]+L1], [0, 0, L1], lineColorRGB=[1.,0.8,0.], lineWidth=5)
# yyy = np.dot(y13,R3002)
# p.addUserDebugLine([yyy[0],yyy[1],yyy[2]+L1], [0, 0, L1], lineColorRGB=[1.,0.8,0.8], lineWidth=5)

# for i in np.arange(0,np.pi*2,np.pi/12):
#     r3002 = (I3 + e_l_Xy13 * np.sin(i) + np.dot(e_l_Xy13, e_l_Xy13) * (1 - np.cos(i)))
#     yyy = np.dot(r3002,y13)
#     p.addUserDebugLine([yyy[0],yyy[1],yyy[2]+L1], [0, 0, L1], lineColorRGB=[0.3,0.8,0.8], lineWidth=5)
#     sleep(1.)

# np.dot(R300,[0, 1, 0])
# xyz13 = np.dot([0, 0, 1],R3001)
# p.addUserDebugLine([xyz13[0],xyz13[1],xyz13[2]+L1], [0, 0, L1], lineColorRGB=[1.,0.8,0.], lineWidth=5)

rz13 = np.array([[-1,0,0],[0,-1,0],[0,0,1]])
R300 = np.dot(R3003,rz13)



# r13x = np.dot([1,0,0],r30_v[0:3,0:3])
# r13y = np.dot([0,1,0],r30_v[0:3,0:3])
# r13z = np.dot([0,0,1],r30_v[0:3,0:3])
# p.addUserDebugLine([r13x[0],r13x[1],r13x[2]+L1], [0, 0, L1], lineColorRGB=[1.,0.8,0.], lineWidth=5)
# p.addUserDebugLine([r13y[0],r13y[1],r13y[2]+L1], [0, 0, L1], lineColorRGB=[1.,0.8,0.], lineWidth=5)
# p.addUserDebugLine([r13z[0],r13z[1],r13z[2]+L1], [0, 0, L1], lineColorRGB=[1.,0.8,0.], lineWidth=5)

# r31x = np.dot(r30_v[0:3,0:3],[1,0,0])
# r31y = np.dot(r30_v[0:3,0:3],[0,1,0])
# r31z = np.dot(r30_v[0:3,0:3],[0,0,1])
# p.addUserDebugLine([r31x[0],r31x[1],r31x[2]+L1], [0, 0, L1], lineColorRGB=[0.2,0.8,0.], lineWidth=5)
# p.addUserDebugLine([r31y[0],r31y[1],r31y[2]+L1], [0, 0, L1], lineColorRGB=[0.2,0.8,0.], lineWidth=5)
# p.addUserDebugLine([r31z[0],r31z[1],r31z[2]+L1], [0, 0, L1], lineColorRGB=[0.2,0.8,0.], lineWidth=5)

# r113x = np.dot([1,0,0],R3002)
# r113y = np.dot([0,1,0],R3002)
# r113z = np.dot([0,0,1],R3002)
# p.addUserDebugLine([r31x[0],r31x[1],r31x[2]+L1], [0, 0, L1], lineColorRGB=[0.2,0.8,0.], lineWidth=5)
# p.addUserDebugLine([r31y[0],r31y[1],r31y[2]+L1], [0, 0, L1], lineColorRGB=[0.2,0.8,0.], lineWidth=5)
# p.addUserDebugLine([r31z[0],r31z[1],r31z[2]+L1], [0, 0, L1], lineColorRGB=[0.2,0.8,0.], lineWidth=5)


# r30z = np.arcsin(np.sqrt(x30**2+y30**2)/z30)
# r30x = np.arcsin(np.sqrt(y30**2+z30**2)/x30)
# z300 = (se0/np.linalg.norm(se0))
# y300 = e_l0
# x300 = np.cross(y300, z300)
# rt = np.array([[-1, 0, 0],[0, -1, 0],[0, 0, 1]])
# R300 = np.dot(np.array([x300,y300,z300]).reshape(3,3),rt)

As = np.dot(u_sw0_x, R300)
Bs = np.dot(-np.dot(u_sw0_x, u_sw0_x), R300)
Cs = np.dot(np.dot(u_sw0.reshape(3,1), u_sw0.reshape(1,3)), R300)
Bs[2,1] = 0
Cs[0:3,1] = 0
# As = np.dot(u_sw0_x, r30_v[0:3,0:3])
# Bs = np.dot(-np.dot(u_sw0_x, u_sw0_x), r30_v[0:3,0:3])
# Cs = np.dot(np.dot(u_sw0.reshape(3,1), u_sw0.reshape(1,3)), r30_v[0:3,0:3])
# Cs = np.dot(np.dot(u_sw0, u_sw0), R300)
# psi = 1.5707
psi = 0
theta1_0 = np.arctan2((As[1,2]*np.sin(psi) + Bs[1,2]*np.cos(psi) + Cs[1,2]),(As[0,2]*np.sin(psi) + Bs[0,2]*np.cos(psi) + Cs[0,2]))
theta2_0 = np.arccos(As[2,2]*np.sin(psi) + Bs[2,2]*np.cos(psi) + Cs[2,2])
theta3_0 = np.arctan2((As[2,1]*np.sin(psi) + Bs[2,1]*np.cos(psi) + Cs[2,1]),(-As[2,0]*np.sin(psi) - Bs[2,0]*np.cos(psi) - Cs[2,0]))
theta_i_0 = theta_i.copy()
theta_i_0[0:4] = [theta1_0, theta2_0, theta3_0, theta4_0]
# p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i_0)
print(theta1_0/6.28*360,'\n',theta2_0/6.28*360,'\n',theta3_0/6.28*360)
print(theta_i[0]/6.28*360,'\n',theta_i[1]/6.28*360,'\n',theta_i[2]/6.28*360)
# %%
r40_v = r40_np(theta_i_0[0],theta_i_0[1],theta_i_0[2],theta_i_0[3],theta_i_0[4],theta_i_0[5],theta_i_0[6],theta_i_0[7])
# r64_v = r40_np(theta_i_0[0],theta_i_0[1],theta_i_0[2],theta_i_0[3],theta_i_0[4],theta_i_0[5],theta_i_0[6],theta_i_0[7])
# r74_v = r40_np(theta_i_0[0],theta_i_0[1],theta_i_0[2],theta_i_0[3],theta_i_0[4],theta_i_0[5],theta_i_0[6],theta_i_0[7])
# r84_v = r40_np(theta_i_0[0],theta_i_0[1],theta_i_0[2],theta_i_0[3],theta_i_0[4],theta_i_0[5],theta_i_0[6],theta_i_0[7])

r43 = np.array([[np.cos(theta4_0), -np.sin(theta4_0), 0],[0,0,-1],[np.sin(theta4_0),  np.cos(theta4_0), 0]])

# r60 = end6_v[0:3,0:3]
r80 = end8_v[0:3,0:3]

Aw = np.dot(np.dot(r43.T, As.T),r80)
Bw = np.dot(np.dot(r43.T, Bs.T),r80)
Cw = np.dot(np.dot(r43.T, Cs.T),r80)

theta5_0 = np.arctan2(Aw[2,2]*np.sin(psi) + Bw[2,2]*np.cos(psi) + Cw[2,2],-Aw[0,2]*np.sin(psi) - Bw[0,2]*np.cos(psi) - Cw[0,2])
theta6_0 = np.arccos(-Aw[1,2]*np.sin(psi) - Bw[1,2]*np.cos(psi) - Cw[1,2])
theta8_0 = np.arctan2(Aw[1,1]*np.sin(psi) + Bw[1,1]*np.cos(psi) + Cw[1,1],-Aw[1,0]*np.sin(psi) - Bw[1,0]*np.cos(psi) - Cw[1,0])

theta_i_0 = theta_i.copy()
theta_i_0[0:6] = [theta1_0, theta2_0, theta3_0, theta4_0, theta5_0, theta6_0 - np.pi/2]
theta_i_0[7] = theta8_0
print(theta5_0/6.28*360,'\n',(theta6_0- np.pi/2)/6.28*360,'\n',theta8_0/6.28*360)
print(theta_i[4]/6.28*360,'\n',theta_i[5]/6.28*360,'\n',theta_i[7]/6.28*360)

# %%
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i_0)

p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i)
# %%
# log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "/home/lihui.liu//mnt/workspace/python/robot/vedio/fixed_joint6.mp4")
p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i)
psi = 0
# for i in np.arange(0,np.pi,0.0001):
    # psi=i
    
run_x = 0.3
run_y = 0.
run_z = 0.
end_roll = 0
end_pitch = 0
end_yaw = 0

end_point_start = end8_v

roll = np.pi
roll_step = roll / 420
T_roll = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
# T8 = np.array([[np.cos(theta_i[7]),np.sin(theta_i[7]),0,0],
#               [-np.sin(theta_i[7]),np.cos(theta_i[7]),0,0],
#               [0, 0, 1,0],
#               [0, 0, 0, 1]])
# end_point_end = np.matmul(end_point_start, T_run)
step_x = run_x / 180.
step_y = run_y / 180.
step_z = run_z / 180.

T_step = np.array([[1, 0, 0, step_x],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

# end8_v = end_point_end
end_point_k = end_point.copy()
end_pos_new = DHParameter().DH_compute(theta_i)
end_pos_new_1 = end_pos_new
end_point_k_1 = end_point_k.copy()
ds6_v_1 = ds6_v

# %%
# log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "/home/lihui.liu//mnt/workspace/python/robot/vedio/analysis_z.mp4")

for i in range(100):
    
    end_point_k[0,3] = end_point_k[0,3] + step_x
    end_point_k[1,3] = end_point_k[1,3] + step_y
    end_point_k[2,3] = end_point_k[2,3] + step_z
    # end0_d = np.dot(end_point_k, end_bias)

    aa = np.dot(end_point_k, [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.55443+0.16], [0, 0, 0, 1]])
    bb = np.dot(aa, [[1, 0, 0, 0.04050], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    z8 = end_point_k[0:3,3] - aa[0:3,3]
    # p.addUserDebugPoints(pointPositions=[aa[0:3,3]], pointColorsRGB=[[0.3,1,0]], pointSize=8)
    # p.addUserDebugPoints(pointPositions=[end_point_k[0:3,3]], pointColorsRGB=[[0.3,1,0]], pointSize=8)
    l8 = z8/np.linalg.norm(z8)
    # p.addUserDebugLine(z8, end_point_k_1[0:3,3], lineColorRGB=[0.8,0.1,0.2], lineWidth=5)
    l8_x = l8[0]
    l8_y = l8[1]
    l8_z = l8[2]
    l8_X = np.array([[0, -l8_z, l8_y],
                     [l8_z, 0, -l8_x],
                     [-l8_y, l8_x, 0]])
    
    for j in np.arange(0,np.pi*2,np.pi/10000):
        # p.addUserDebugPoints(pointPositions=[aa[0:3,3]], pointColorsRGB=[[1,1,0]], pointSize=10)
        # p.addUserDebugPoints(pointPositions=[bb[0:3,3]], pointColorsRGB=[[1,1,0]], pointSize=10)
        l8_t = I3 + l8_X * np.sin(-j) + np.dot(l8_X, l8_X) * (1 - np.cos(-j))
        ds6_v = (np.dot(l8_t,(bb[0:3,3]-aa[0:3,3]).T) + aa[0:3,3]).reshape(3,1)
        
        # ds6_v = end0_d[0:3,3].reshape(3,1)
        # p.addUserDebugPoints(pointPositions=[ds6_v.reshape(3)], pointColorsRGB=[[1,0,1]], pointSize=8)
        # ds6_v = np.array([[end_point_k[0,3] + 0.01125], [end_point_k[1,3]], [end_point_k[2,3] - 0.55443 + 0.16]])
    
        ds6w_v = np.linalg.norm(ds6_v - dw)
        
        thetao4err = (dwo4**2 + dso4**2 - ds6w_v**2) / (2*dwo4*dso4)
        # if -1 <= thetao4err <= 1:
        #     thetaO4 = np.arccos((dwo4**2 + dso4**2 - ds6w_v**2) / (2*dwo4*dso4))
        # else:
        #     continue
        # thetaOd1 = np.arctan2(L3,d34)
        # thetaOd2 = np.arctan2(L5,d34)
        # theta42 = np.pi*2 - (np.pi*2-thetaO4) - thetaOd1 - thetaOd2
        theta41 = np.pi*2 - thetaO4 - thetaOd1 - thetaOd2
    
        theta4_0 = theta41
        # alpha1 = np.arctan2(d34,L3)
        alpha2err = (dwo4**2 + ds6w_v**2 - dso4**2) / (2 * dwo4 * ds6w_v)
        # if -1 <= thetao4err <= 1:
        #     alpha2 = np.arccos((dwo4**2 + ds6w_v**2 - dso4**2) / (2 * dwo4 * ds6w_v))
        # else:
        #     continue
        alpha_0 = -alpha1 - alpha2
    
        # I3 = np.eye(3)
        # u_sw0 = (ds6-dw)/(ds6-dw).norm()
        u_sw0 = ((ds6_v-dw)/np.linalg.norm(ds6_v-dw)).reshape(3)
        u_sw0_x = np.array([[0, -u_sw0[2], u_sw0[1]],
                            [u_sw0[2], 0, -u_sw0[0]],
                            [-u_sw0[1], u_sw0[0], 0]])
        # u_sw0_x = u_sw0_x/np.linalg.norm(u_sw0_x)
        # R_sw_psi = I3 + u_sw0_x * sympy.sin(-psi) + u_sw0_x*u_sw0_x*(1-sympy.cos(-psi))
        # R_sw_psi = sympy.Matrix(I3 + u_sw0_x * sympy.sin(psi) + u_sw0_x*u_sw0_x*(1-sympy.cos(psi)))
        # R_sw_psi.subs(psi, np.pi/30)
        sv0 = np.array([0, 0, 1])
        sw0 = ((ds6_v-dw)/np.linalg.norm(ds6_v-dw)).reshape(3)
        
        e_l = np.cross(sv0, sw0)
        # e_l[2] = e_l[2] + L1
        e_l0 = e_l/np.linalg.norm(e_l)
        # e_l0[2] = e_l0[2]
        # e_l0 = np.array([0, 1, 0])
        e_l_x = e_l0[0]
        e_l_y = e_l0[1]
        e_l_z = e_l0[2]
        
        e_l_X = np.array([[0, -e_l_z, e_l_y],
                          [e_l_z, 0, -e_l_x],
                          [-e_l_y, e_l_x, 0]])
        
        R_l_alpha = I3 + e_l_X * np.sin(alpha_0) + np.dot(e_l_X, e_l_X) * (1 - np.cos(alpha_0))
        
        d3e0 = d34 * np.tan(theta4_0/2)
        dew0 = L3 + d3e0
        se0 = dew0 * np.dot(R_l_alpha, sw0)
        
        r30_v = r30_np(theta_i[0],theta_i[1],theta_i[2],theta_i[3],theta_i[4],theta_i[5],theta_i[6],theta_i[7])
        # array([[ 0.67728209, -0.        ,  0.73572344],
        #        [ 0.        ,  1.        ,  0.        ],
        #        [-0.73572344,  0.        ,  0.67728209]
        
        # z300 = (se0/np.linalg.norm(se0))
        # # y300 = -np.sign(np.sin(theta4_0)) * e_l0
        # y300 = e_l0
        # x300 = np.cross(y300, z300)
        z300 = (se0/np.linalg.norm(se0))
        x30 = z300[0]
        y30 = z300[1]
        z30 = z300[2]
        alpha13 = -np.arctan2(np.sqrt(x30**2+y30**2),z30)
        # sv13 = np.array([0, 0, 1])
        # e_l13 = np.cross(se0, sv13)
        # e_l130 = e_l13/np.linalg.norm(e_l13)
        # q1 = np.quaternion(1,2,3,4)
        # q2 = quaternion.from_float_array([1,2,3,4])
        # q3 = quaternion.from_rotation_matrix([[1,2,3],[1,2,3],[1,2,3]])
        # q4 = quaternion.from_euler_angles([1,2,3])
        # print(q1,q2,q3,q4)
        # e_l_x13 = e_l130[0]
        # e_l_y13 = e_l130[1]
        # e_l_z13 = e_l130[2]
        # e_l_X13 = np.array([[0, -e_l_z13, e_l_y13],
        #                     [e_l_z13, 0, -e_l_x13],
        #                     [-e_l_y13, e_l_x13, 0]])
        # R3001 = (I3 + e_l_X13 * np.sin(alpha13) + np.dot(e_l_X13, e_l_X13) * (1 - np.cos(alpha13)))
        R3001 = (I3 + e_l_X * np.sin(alpha13) + np.dot(e_l_X, e_l_X) * (1 - np.cos(alpha13))).T
        y13 = np.dot([0, 1, 0],R3001)
        y13_5 = np.linalg.norm(e_l0-y13)
        alphay13 = np.arccos((2-y13_5**2)/2)
        e_l_Xy13 = np.array([[0, -z30, y30],
                             [z30, 0, -x30],
                             [-y30, x30, 0]])
        R3002 = (I3 + e_l_Xy13 * np.sin(alphay13) + np.dot(e_l_Xy13, e_l_Xy13) * (1 - np.cos(alphay13))).T
        R3003 = np.dot(R3001,R3002).T
        # R300 = R3001
        # p.addUserDebugLine([y13[0],y13[1],y13[2]+L1], [0, 0, L1], lineColorRGB=[1.,0.8,0.], lineWidth=5)
        # yyy = np.dot(y13,R3002)
        # p.addUserDebugLine([yyy[0],yyy[1],yyy[2]+L1], [0, 0, L1], lineColorRGB=[1.,0.8,0.8], lineWidth=5)
        
        # for i in np.arange(0,np.pi*2,np.pi/12):
        #     r3002 = (I3 + e_l_Xy13 * np.sin(i) + np.dot(e_l_Xy13, e_l_Xy13) * (1 - np.cos(i)))
        #     yyy = np.dot(r3002,y13)
        #     p.addUserDebugLine([yyy[0],yyy[1],yyy[2]+L1], [0, 0, L1], lineColorRGB=[0.3,0.8,0.8], lineWidth=5)
        #     sleep(1.)
        
        # np.dot(R300,[0, 1, 0])
        # xyz13 = np.dot([0, 0, 1],R3001)
        # p.addUserDebugLine([xyz13[0],xyz13[1],xyz13[2]+L1], [0, 0, L1], lineColorRGB=[1.,0.8,0.], lineWidth=5)
        
        rz13 = np.array([[-1,0,0],[0,-1,0],[0,0,1]])
        R300 = np.dot(R3003,rz13)
    
        As = np.dot(u_sw0_x, R300)
        Bs = np.dot(-np.dot(u_sw0_x, u_sw0_x), R300)
        Cs = np.dot(np.dot(u_sw0.reshape(3,1), u_sw0.reshape(1,3)), R300)
    
        theta1_0 = np.arctan2((As[1,2]*np.sin(psi) + Bs[1,2]*np.cos(psi) + Cs[1,2]),(As[0,2]*np.sin(psi) + Bs[0,2]*np.cos(psi) + Cs[0,2]))
        theta2_0 = np.arccos(As[2,2]*np.sin(psi) + Bs[2,2]*np.cos(psi) + Cs[2,2])
        theta3_0 = np.arctan2((As[2,1]*np.sin(psi) + Bs[2,1]*np.cos(psi) + Cs[2,1]),(-As[2,0]*np.sin(psi) - Bs[2,0]*np.cos(psi) - Cs[2,0]))
    
        r43 = np.array([[np.cos(theta4_0), -np.sin(theta4_0), 0],[0,0,-1],[np.sin(theta4_0),  np.cos(theta4_0), 0]])
        r80 = end8_v[0:3,0:3]
        Aw = np.dot(np.dot(r43.T, As.T),r80)
        Bw = np.dot(np.dot(r43.T, Bs.T),r80)
        Cw = np.dot(np.dot(r43.T, Cs.T),r80)
        theta8_0 = np.arctan2(Aw[1,1]*np.sin(psi) + Bw[1,1]*np.cos(psi) + Cw[1,1],-Aw[1,0]*np.sin(psi) - Bw[1,0]*np.cos(psi) - Cw[1,0])
        # print('*'*20,'\n',theta8_0,'\n',theta_i[7])
        theta5_0 = np.arctan2(Aw[2,2]*np.sin(psi) + Bw[2,2]*np.cos(psi) + Cw[2,2],-Aw[0,2]*np.sin(psi) - Bw[0,2]*np.cos(psi) - Cw[0,2])
        theta6_0 = np.arccos(-Aw[1,2]*np.sin(psi) - Bw[1,2]*np.cos(psi) - Cw[1,2])
    
        theta_i_0 = theta_i.copy()
        theta_i_0[0:6] = [theta1_0, theta2_0, theta3_0, theta4_0, theta5_0, theta6_0 - np.pi/2]
        theta_i_0[7] = theta8_0
    
        end_pos_new = DHParameter().DH_compute(theta_i_0)
        # print(j)
        if np.abs(np.linalg.norm(end_point_k[0:3,3]) - np.linalg.norm(end_pos_new[0:3,3])) < 0.005 \
            and np.abs(np.linalg.norm(end_point_k[0:3,0:3]) - np.linalg.norm(end_pos_new[0:3,0:3])) < 0.01:
            break
        if j > np.pi*2-0.001:
            print('*'*20,i)

    # p.addUserDebugLine(end_pos_new_1[0:3,3], end_pos_new[0:3,3], lineColorRGB=[0.3,0.2,0.6], lineWidth=5)
    # p.addUserDebugLine(end_point_k[0:3,3], end_point_k_1[0:3,3], lineColorRGB=[0.8,0.1,0.2], lineWidth=5)
    p.addUserDebugPoints(pointPositions=[end_pos_new[0:3,3]], pointColorsRGB=[[0.5,0,0.3]], pointSize=3)
    p.addUserDebugPoints(pointPositions=[end_point_k[0:3,3]], pointColorsRGB=[[0.1,0.5,0.2]], pointSize=3)
    # p.addUserDebugLine(ds6_v, ds6_v_1, lineColorRGB=[0.6,0.2,0.9], lineWidth=5)
    end_pos_new_1 = end_pos_new
    # end_point_k_1 = end_point_k
    ds6_v_1 = ds6_v
    # end_point_k = end_pos_new
    p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i_0)
    sleep(1./240.)
# p.stopStateLogging(log_id)
    
# %%
for i in np.arange(np.pi,0,-0.002):
    psi=i
    theta1_0 = np.arctan2((As[1,2]*np.sin(psi) + Bs[1,2]*np.cos(psi) + Cs[1,2]),(As[0,2]*np.sin(psi) + Bs[0,2]*np.cos(psi) + Cs[0,2]))
    theta2_0 = np.arccos(As[2,2]*np.sin(psi) + Bs[2,2]*np.cos(psi) + Cs[2,2])
    theta3_0 = np.arctan2((As[2,1]*np.sin(psi) + Bs[2,1]*np.cos(psi) + Cs[2,1]),(-As[2,0]*np.sin(psi) - Bs[2,0]*np.cos(psi) - Cs[2,0]))
    theta_i_0 = theta_i.copy()
    theta_i_0[0:4] = [theta1_0, theta2_0, theta3_0, theta4_0]
    p.setJointMotorControlArray(robot_id,range(11),p.POSITION_CONTROL,targetPositions=theta_i_0)
    sleep(1./240.)
    # print(theta1/6.28*360,'\n',theta2/6.28*360,'\n',theta3/6.28*360)
    # ds6_v_new = ds6_np(theta_i_0[0],theta_i_0[1],theta_i_0[2],theta_i_0[3],theta_i_0[4],theta_i_0[5],theta_i_0[6],theta_i_0[7])
    # p.addUserDebugLine(ds6_v_new1, ds6_v_new, lineColorRGB=[0.3,0.2,0.6], lineWidth=5)
    # ds6_v_new1 = ds6_v_new
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



