# import pybullet
# import pybullet_data
# datapath=pybullet_data.getDataPath()
# pybullet.connect(pybullet.GUI)
# pybullet.setAdditionalSearchPath(datapath)
# pybullet.loadURDF("r2d2.urdf",[0,0,1])
#

# import pybullet
# import pybullet_data
#
# datapath=pybullet_data.getDataPath()
#
# # Starting the simulation is dead simple...
# pybullet.connect(pybullet.GUI)
# pybullet.setAdditionalSearchPath(datapath)
# # plane = pybullet.loadURDF("plane.urdf")
# pybullet.loadURDF("urdf/probot_anno_with_camera.urdf",[0,0,1])


import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("urdf/probot_anno_with_camera.urdf",cubeStartPos, cubeStartOrientation)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    print(cubePos,cubeOrn)
p.disconnect()