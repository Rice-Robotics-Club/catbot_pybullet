import pybullet as p
import pybullet_data
import os 

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("catbot_pybullet/meshes/oneshape.urdf", cubeStartPos, cubeStartOrientation)
p.setRealTimeSimulation(1)
while True:
    pass