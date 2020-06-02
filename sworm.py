import pybullet as p
import time
import math
import pybullet_data
import matplotlib.pyplot as plt

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
scale = 10
dt = 1. / 240.

#%% Define object

## Plane
plane = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, plane)

## Tunnel 
CW = 0.5*scale # Width
n = 10 # Length parameter
for i in range(2*n):
  channelpos = [CW/2,i-n+1,0]
  channel = p.loadURDF("cube_rotate.urdf", channelpos[0], channelpos[1], channelpos[2])
  p.createMultiBody(0, channel)
for i in range(2*n):
  channelpos = [-CW/2,i-n+1,0]
  channel = p.loadURDF("cube_rotate.urdf", channelpos[0], channelpos[1], channelpos[2])
  p.createMultiBody(0, channel)

## Muscle and joint

# Building block
L = 0.162*scale
colBoxId = p.createCollisionShape(p.GEOM_BOX,halfExtents=[L/10, L, L/10])
colJointId = p.createCollisionShape(p.GEOM_SPHERE,radius=L/10)

# Body parameters
mass = 1
visualShapeId = -1
useMaximalCoordinates = True

link_Masses = []
linkCollisionShapeIndices = []
linkVisualShapeIndices = []
linkPositions = []
linkOrientations = []
linkInertialFramePositions = []
linkInertialFrameOrientations = []
indices = []
jointTypes = []
axis = []

n = 4
for i in range(n):
    if i % 2 == 1:  
      link_Masses.append(1)
      linkCollisionShapeIndices.append(colBoxId)
      linkVisualShapeIndices.append(-1)
      linkPositions.append([0, 11*L/10, 0])
      linkOrientations.append([0, 0, 0, 1])
      linkInertialFramePositions.append([0, 0, 0])
      linkInertialFrameOrientations.append([0, 0, 0, 1])
      indices.append(i)
      jointTypes.append(p.JOINT_FIXED)
      axis.append([0, 0, 1])
    else:
      link_Masses.append(1)
      linkCollisionShapeIndices.append(colJointId)
      linkVisualShapeIndices.append(-1)
      linkPositions.append([0, 11*L/10, 0])
      linkOrientations.append([0, 0, 0, 1])
      linkInertialFramePositions.append([0, 0, 0])
      linkInertialFrameOrientations.append([0, 0, 0, 1])
      indices.append(i)
      jointTypes.append(p.JOINT_REVOLUTE)
      axis.append([0, 0, 1])

# Initial position parameter
W = 0.015*scale
delta_x = 0.1
x_a = W
y_a = -L-L/5-L/2
x_b = -W
y_b = -L-L/5-L/2-delta_x

# Initial position
basePosition1 = [x_a, y_a, 1]
baseOrientation1 = [0, 0, 0, 1]
basePosition2 = [x_b, y_b, 1]
baseOrientation2 = [0, 0, 0, 1]

# Agent 1
sworm_a = p.createMultiBody(mass,
                            colBoxId,
                            visualShapeId,
                            basePosition1,
                            baseOrientation1,
                            linkMasses=link_Masses,
                            linkCollisionShapeIndices=linkCollisionShapeIndices,
                            linkVisualShapeIndices=linkVisualShapeIndices,
                            linkPositions=linkPositions,
                            linkOrientations=linkOrientations,
                            linkInertialFramePositions=linkInertialFramePositions,
                            linkInertialFrameOrientations=linkInertialFrameOrientations,
                            linkParentIndices=indices,
                            linkJointTypes=jointTypes,
                            linkJointAxis=axis,
                            useMaximalCoordinates=useMaximalCoordinates)

# Agent 2
sworm_b = p.createMultiBody(mass,
                            colBoxId,
                            visualShapeId,
                            basePosition2,
                            baseOrientation2,
                            linkMasses=link_Masses,
                            linkCollisionShapeIndices=linkCollisionShapeIndices,
                            linkVisualShapeIndices=linkVisualShapeIndices,
                            linkPositions=linkPositions,
                            linkOrientations=linkOrientations,
                            linkInertialFramePositions=linkInertialFramePositions,
                            linkInertialFrameOrientations=linkInertialFrameOrientations,
                            linkParentIndices=indices,
                            linkJointTypes=jointTypes,
                            linkJointAxis=axis,
                            useMaximalCoordinates=useMaximalCoordinates)

#%% Enviorment set
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)
anistropicFriction = [1, 0.01, 0.01]

p.changeDynamics(sworm_a, -1, lateralFriction=2, anisotropicFriction=anistropicFriction)
p.getNumJoints(sworm_a)
for i in range(p.getNumJoints(sworm_a)):
  p.getJointInfo(sworm_a, i)
  p.changeDynamics(sworm_a, i, lateralFriction=2, anisotropicFriction=anistropicFriction)

p.changeDynamics(sworm_b, -1, lateralFriction=2, anisotropicFriction=anistropicFriction)
p.getNumJoints(sworm_b)
for i in range(p.getNumJoints(sworm_b)):
  p.getJointInfo(sworm_b, i)
  p.changeDynamics(sworm_b, i, lateralFriction=2, anisotropicFriction=anistropicFriction)

# Motor actuation
delta_psi = 0.5
epsilon = 1
m_wavefrequency = 0.5
m_waveAmplitude = 40
m_waveFront = 2*math.pi/3*epsilon
maximumforce = 50

#%% Simulation and observation 
pos = []
t_list = []

t = 0
while (1):

  t += dt

  # Map phase to curvature
  targetPos_a_front = m_waveAmplitude*math.sin(-2*math.pi*m_wavefrequency*t)
  targetPos_a_rear = m_waveAmplitude*math.sin(m_waveFront-2*math.pi*m_wavefrequency*t)
  targetPos_b_front = m_waveAmplitude*math.sin(delta_psi-2*math.pi*m_wavefrequency*t)
  targetPos_b_rear = m_waveAmplitude*math.sin(delta_psi+m_waveFront-2*math.pi*m_wavefrequency*t)
  
  # Observation
  pos.append(targetPos_a_front)
  t_list.append(t)  
  
  # Set a_front motor
  p.setJointMotorControl2(sworm_a,
                          0,
                          p.POSITION_CONTROL,
                          targetPosition=targetPos_a_front,
                          force=maximumforce)
  # Set a_rear motor
  p.setJointMotorControl2(sworm_a,
                          2,
                          p.POSITION_CONTROL,
                          targetPosition=targetPos_a_rear,
                          force=maximumforce)
                        
  # Set b_front motor
  p.setJointMotorControl2(sworm_b,
                          0,
                          p.POSITION_CONTROL,
                          targetPosition=targetPos_b_front,
                          force=maximumforce)
  # Set b_rear motor
  p.setJointMotorControl2(sworm_b,
                          2,
                          p.POSITION_CONTROL,
                          targetPosition=targetPos_b_rear,
                          force=maximumforce)

  p.stepSimulation()
  
  plt.plot(t_list, pos)
