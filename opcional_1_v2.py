import pybullet as p
import pybullet_data
import time

INIT_VALUE = 0
MIN_VALUE_J0 = -1.57
MAX_VALUE_J0 = 1.57

MIN_VALUE_J1 = -3.14
MAX_VALUE_J1 = 3.14

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

planeId = p.loadURDF("plane.urdf")

startPosition = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

robotId = p.loadURDF("urdf/robot.urdf", startPosition, startOrientation)

numJoints = p.getNumJoints (robotId)
print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j) [0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

x_pos_J0 = p.addUserDebugParameter("joint1_angle", MIN_VALUE_J0, MAX_VALUE_J0, INIT_VALUE)
x_pos_J1 = p.addUserDebugParameter("joint2_angle", MIN_VALUE_J1, MAX_VALUE_J1, INIT_VALUE)

while True:
    p.stepSimulation()

    joint1_angle = p.readUserDebugParameter(x_pos_J0)
    joint2_angle = p.readUserDebugParameter(x_pos_J1)

    p.setJointMotorControl2(
        robotId,
        0,
        p.POSITION_CONTROL,
        targetPosition=joint1_angle,
        force=8
    )

    p.setJointMotorControl2(
        robotId,
        1,
        p.POSITION_CONTROL,
        targetPosition=joint2_angle,
        force=8
    )

    time.sleep(1./240.)

p.disconnect()
