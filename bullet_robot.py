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

