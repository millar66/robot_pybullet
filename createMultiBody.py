# %%
import pybullet as p
import pybullet_data
import time

# %%
_ = p.connect(p.GUI)
p.setGravity(0,0,-9.81)#设置重力
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=10)
p.loadURDF("plane100.urdf", useMaximalCoordinates=True)
p.resetDebugVisualizerCamera(cameraTargetPosition=[0.11,0.5,0.25],\
                             cameraDistance=7,\
                             cameraPitch=-35.8,\
                             cameraYaw=-118.05) #转变视角
# 创建过程中不渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
# 不展示GUI的套件
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# 禁用 tinyrenderer 
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)

# 创建视觉模型和碰撞箱模型时共用的两个参数
# 可选的有
# GEOM_SPHERE, GEOM_BOX, GEOM_CAPSULE, 
# GEOM_CYLINDER, GEOM_PLANE, GEOM_MESH。
# 基本上大家看看意思就知道是什么意思了。我们使用的是GEOM_MESH，也就是网格模型，
shift = [0, -0.02, 0]
scale = [1, 1, 1]

# 创建视觉形状和碰撞箱形状
visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="duck,obj",
    rgbaColor=[1, 1, 1, 1],
    specularColor=[0.4, 0.4, 0],
    visualFramePosition=shift,
    meshScale=scale
)

# 再根据pybullet_data下的duck_vhacd.obj文件来创建鸭子的碰撞箱模型：
collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName="duck_vhacd.obj",
    collisionFramePosition=shift,
    meshScale=scale
)

# 使用创建的视觉形状和碰撞箱形状使用createMultiBody将两者结合在一起

# %%
for i in range(3):
    p.createMultiBody(
        baseMass=1000,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[0, 0, 2 * i],
        useMaximalCoordinates=True
    )

# 创建结束，重新开启渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

while True:
    time.sleep(1./240.)

# %%
p.disconnect()

