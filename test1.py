
ParameterInit.pos_lim()

use_gui = True
if use_gui:
    physicsClientId = p.connect(p.GUI)
else:
    physicsClientId = p.connect(p.DIRECT)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0) # 0:不让CPU上的集成显卡参与渲染工作
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1) # 0:不显示GUI上的控件
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1) # 1:打开渲染

p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

p.resetDebugVisualizerCamera(cameraTargetPosition=[0.05,0.02,0.39],\
                             cameraDistance=1.20,\
                             cameraPitch=-30.40,\
                             cameraYaw=24.40) #转变视角

p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
robot_id = p.loadURDF("./ccc/000PSM_10.SLDASM/urdf/modified.urdf",
                      basePosition=[0, 0, 0], useMaximalCoordinates=False, useFixedBase=True)

numJoints = 11
# %%
pos, orn = DHParameter().getPosOrn()
timer = timeit.default_timer()
print(timer)
# %%
%%timeit
Td = np.zeros((numJoints, 4, 4))
Tx = np.zeros((numJoints, 4, 4))
Ty = np.zeros((numJoints, 4, 4))
Tz = np.zeros((numJoints, 4, 4))
orn_cos_x = np.zeros(numJoints)
orn_sin_x = np.zeros(numJoints)
orn_cos_y = np.zeros(numJoints)
orn_sin_y = np.zeros(numJoints)
orn_cos_z = np.zeros(numJoints)
orn_sin_z = np.zeros(numJoints)
for i in range(numJoints):
    orn_cos_x[i] = np.cos(orn[i][0])
    orn_sin_x[i] = np.sin(orn[i][0])
    orn_cos_y[i] = np.cos(orn[i][1])
    orn_sin_y[i] = np.sin(orn[i][1])
    orn_cos_z[i] = np.cos(orn[i][2])
    orn_sin_z[i] = np.sin(orn[i][2])
    Td[i] = np.array([[1, 0, 0, pos[i][0]],
                      [0, 1, 0, pos[i][1]],
                      [0, 0, 1, pos[i][2]],
                      [0, 0, 0, 1        ]])
    Tx[i] = np.array([[1, 0,            0,               0],
                      [0, orn_cos_x[i], orn_sin_x[i]*-1, 0],
                      [0, orn_sin_x[i], orn_cos_x[i],    0],
                      [0, 0,            0,               1]])
    Ty[i] = np.array([[orn_cos_y[i],    0, orn_sin_y[i], 0],
                      [0,               1, 0,            0],
                      [orn_sin_y[i]*-1, 0, orn_cos_y[i], 0],
                      [0,               0, 0,            1]])
    Tz[i] = np.array([[orn_cos_z[i], orn_sin_z[i]*-1, 0, 0],
                      [orn_sin_z[i], orn_cos_z[i],    0, 0],
                      [0,            0,               1, 0],
                      [0,            0,               0, 1]])


    
    
    
    
    
    
    
    
    
    
    
    
# %%
targetPosition_init = [0, -0, 0, -0, 0, 0, 0, 0, 0, 0, 0]
p.setJointMotorControlArray(robot_id,
                            range(11),
                            p.POSITION_CONTROL,
                            targetPositions=targetPosition_init)