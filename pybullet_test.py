import math
import pybullet as p
import pybullet_data as pd
import pybullet_utils.bullet_client as bc
import os 

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pd.getDataPath()) #used by loadURDF
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("catbot_pybullet/meshes/catbot.urdf", startPos, startOrientation)
p.setRealTimeSimulation(1)

# # get joint infos
jnt_infos = []
for i in range(p.getNumJoints(robotId)):
    jnt = p.getJointInfo(bodyUniqueId=robotId, jointIndex=i)[1]
    print(jnt)
    jnt_infos.append(jnt)
while True:
    p.setJointMotorControlArray(bodyUniqueId=robotId,
                                        jointIndices=range(12),
                                        controlMode=p.POSITION_CONTROL,
                                        targetPositions=[.4] + [0.0]*11)


pos, orn = bullet_client.getBasePositionAndOrientation(greenboxID)

print("Position of the red box:", pos)
print("Orientation of the red box:", orn)

lin_vel, ang_vel = bullet_client.getBaseVelocity(redboxID)

print("Linear velocity of the red box:", lin_vel)
print("Angular velocity of the red box:", ang_vel)

bullet_client.resetBasePositionAndOrientation(redboxID, 
                                              posObj=[0, 0, 0.5], 
                                              ornObj=[0, 0, 0, 1])

# save state
stateID = bullet_client.saveState()

print("The state ID of the saved state:", stateID)
print("red box pose:", bullet_client.getBasePositionAndOrientation(redboxID))
print("red box velocity:", bullet_client.getBaseVelocity(redboxID))

#restore state
bullet_client.restoreState(1)

print("red box pose:", bullet_client.getBasePositionAndOrientation(redboxID))
print("red box velocity:", bullet_client.getBaseVelocity(redboxID))

# num joints
print("Number of joints of the blue box:", 
      bullet_client.getNumJoints(blueboxID))

print("Number of joints of the red box:", 
      bullet_client.getNumJoints(redboxID))

# get joint info 

jnt_info = bullet_client.getJointInfo(bodyUniqueId=blueboxID,
                                      jointIndex=0)
print(jnt_info)

jnt_state = bullet_client.getJointState(bodyUniqueId=blueboxID, 
                                        jointIndex=0)

print("jointPosition:", jnt_state[0])
print("jointVelocity:", jnt_state[1])
print("jointReactionForces:", jnt_state[2])
print("appliedJointMotorTorque:", jnt_state[3])

# control

bullet_client.setJointMotorControl2(bodyUniqueId=blueboxID,
                                    jointIndex=0,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=1.5)

bullet_client.setJointMotorControlArray(bodyUniqueId=pandaID,
                                        jointIndices=range(7),
                                        controlMode=p.POSITION_CONTROL,
                                        targetPositions=[0.9]*7)
#  You can apply a force or torque to a body using 
# **applyExternalForce** and **applyExternalTorque**. 
# Note that this method will only work when explicitly stepping the 
# simulation using stepSimulation. After each simulation step, 
# the external forces are cleared to zero.
import time

while True:
    bullet_client.applyExternalForce(objectUniqueId=redboxID, 
                                     linkIndex=-1, 
                                     forceObj=[0.5, 0, 0],
                                     posObj=[0, 0, 0],
                                     # flags=p.LINK_FRAME
                                     flags=p.WORLD_FRAME
                                    )
    bullet_client.stepSimulation()
    time.sleep(0.01)

# change mass firction and restitution
bullet_client.changeDynamics(bodyUniqueId=redboxID, 
                             linkIndex=-1,
                             lateralFriction=0.1)
                             #lateralFriction=0.5)

#contact points
conpts = bullet_client.getContactPoints(greenboxID, redboxID)
print("Number of contact points:", len(conpts))
