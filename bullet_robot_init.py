# %%
import pybullet as p
import time
import pybullet_data

# %%
# physicsVlient = p.connect(p.GUI,"option=opengl2")#or p.DIRECT for non-graphical version
physicsVlient = p.connect(p.GUI)

# 该语句的作用是禁用tinyrenderer，也就是不让CPU上的集成显卡来参与渲染工作。
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
# p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
# p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
# 渲染出来的GUI上许多控件，不显示
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(-1,-2,-10)

# 除了使用loadURDF加载urdf模型外，我们还可以通过loadSDF来加载sdf模型，
# 通过loadMJCF来加载mjcf模型。其中sdf是数据库文件，mjcf是MuJoCo平台的机器人描述文件。
planeID = p.loadURDF("plane.urdf")

cubeStarPos = [1,2,3]
text_id = p.addUserDebugText(text="start", textPosition=cubeStarPos, textColorRGB=[0, 1, 0], textSize=1.2)
text_id = p.addUserDebugText(text="Destination", textPosition=[0,0,0], textColorRGB=[0, 0, 1], textSize=1.2)
p.addUserDebugLine(lineFromXYZ=cubeStarPos, lineToXYZ=[0,0,0], lineColorRGB=[0, 1, 0], lineWidth=2)
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",cubeStarPos, cubeStartOrientation)

joint_num = p.getNumJoints(boxId)
print("r2d2的节点数量为：", joint_num)
# JointInfo = p.getJointInfo(boxId,6)
print("r2d2的信息：")
for joint_index in range(joint_num):
    info_tuple = p.getJointInfo(boxId, joint_index)
    print(f"关节序号：{info_tuple[0]}\n\
            关节名称：{info_tuple[1]}\n\
            关节类型：{info_tuple[2]}\n\
            机器人第一个位置的变量索引：{info_tuple[3]}\n\
            机器人第一个速度的变量索引：{info_tuple[4]}\n\
            保留参数：{info_tuple[5]}\n\
            关节的阻尼大小：{info_tuple[6]}\n\
            关节的摩擦系数：{info_tuple[7]}\n\
            slider和revolute(hinge)类型的位移最小值：{info_tuple[8]}\n\
            slider和revolute(hinge)类型的位移最大值：{info_tuple[9]}\n\
            关节驱动的最大值：{info_tuple[10]}\n\
            关节的最大速度：{info_tuple[11]}\n\
            节点名称：{info_tuple[12]}\n\
            局部框架中的关节轴系：{info_tuple[13]}\n\
            父节点frame的关节位置：{info_tuple[14]}\n\
            父节点frame的关节方向：{info_tuple[15]}\n\
            父节点的索引，若是基座返回-1：{info_tuple[16]}\n\n")

# %%
p.setRealTimeSimulation(1) # 实时模拟
while True:
    # pass
    time.sleep(1)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    print("-" * 20)
    print(f"机器人的位置坐标为:{cubePos}\n机器人的朝向四元数为:{cubeOrn}")
    print("-" * 20)


# %%
p.setRealTimeSimulation(0)  # 关闭实时模拟
p.setTimeStep(1./240.)  # 设置时间步
for i in range (2000):
  p.stepSimulation()
  time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print("-" * 20)
print(f"机器人的位置坐标为:{cubePos}\n机器人的朝向四元数为:{cubeOrn}")
print("-" * 20)

# %%
p.resetSimulation(planeID)
p.disconnect()

# %%
# 画一条空间内的直线
stars = [[1, 1, 0], [-1, 1, 0], [-1, 1, 3], [1, 1, 3]]
ends = [[-1, 1, 0], [-1, 1, 3], [1, 1, 3], [1, 1, 0]]
for i, j in zip(stars, ends):
    p.addUserDebugLine(lineFromXYZ=i, lineToXYZ=j, lineColorRGB=[0, 1, 0], lineWidth=2)
# 画仿真中物体的运动轨迹
p.addUserDebugLine(lineFromXYZ=body_position_w_pre, lineToXYZ=body_position_w,
                   lineColorRGB=[1, 0, 0], lineWidth=2)  # show body trajectory
p.addUserDebugLine(lineFromXYZ=foot_position_w_pre, lineToXYZ=foot_position_w,
                   lineColorRGB=[0, 0, 1], lineWidth=2, lifeTime=30)  # show foot trajectory

# 添加参数滑块，分别为速度，驱动力、转向
targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -10, 10, 0)  # 参数名称 最小值 最大值 起始值
maxForceSlider = p.addUserDebugParameter("maxForce", 0, 10, 10)
steeringSlider = p.addUserDebugParameter("steering", -0.5, 0.5, 0)

# 读取参数的值，读取速度，转向角度，驱动力参数
targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
maxForce = p.readUserDebugParameter(maxForceSlider)
steeringAngle = p.readUserDebugParameter(steeringSlider)

# 删除所有的滑块和按钮：
p.removeAllUserParameters(physicsClientId)

# 使用唯一id删除调试项（文本、线等）: 
p.removeUserDebugItem(itemUniqueId, physicsClientId)

# 删除所有调试项: 
p.removeAllUserDebugItems()

# 更改某个特定物体的特定link的颜色
p.setDebugObjectColor(objectUniqueId=robot_id, linkIndex=-1, objectDebugColorRGB=[0, 0, 1])






# %%
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0) # 不让引擎渲染没有加载好的场景
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0) # 不让CPU上的集成显卡参与渲染工作
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) # 不显示GUI上的控件
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1) # 打开渲染
p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)  # 一步步地渲染

