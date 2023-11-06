# %%
import pybullet as p
import time
import pybullet_data
physicsVlient = p.connect(p.GUI,"option=opengl2")#or p.DIRECT for non-graphical version
# %%
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeID = p.loadURDF("plane.urdf")
cubeStarPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",cubeStarPos, cubeStartOrientation)
for i in range (10000):
  p.stepSimulation()
  time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(bosId)
print(cubePos,cubeOrn)
p.disconnect()

# %%

