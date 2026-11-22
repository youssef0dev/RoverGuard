import pybullet as p
import pybullet_data
import time


# 1. N7ell simulation
p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)


# 2. Ndakhlou l2ard
plane_id = p.loadURDF("plane.urdf")


# 3. Ndakhlou RoverGuard
robot_id = p.loadURDF(
    "robot/roverguard.urdf",
    basePosition=[0, 0, 0.15]
)


# 4. N9raw joints dyal robot
number_of_joints = p.getNumJoints(robot_id)

print("Nombre de joints:", number_of_joints)

joint_indices = {}

for joint_index in range(number_of_joints):
    joint_info = p.getJointInfo(robot_id, joint_index)

    joint_name = joint_info[1].decode("utf-8")
    link_name = joint_info[12].decode("utf-8")

    joint_indices[joint_name] = joint_index

    print(
        "Index:", joint_index,
        "| Joint:", joint_name,
        "| Link:", link_name
    )


# 5. Nakhdo ar9am joints bla ma n7efdo 0 w 1
left_joint = joint_indices["left_wheel_joint"]
right_joint = joint_indices["right_wheel_joint"]
caster_joint = joint_indices["rear_caster_joint"]


# 6. Friction bin roues w l2ard
p.changeDynamics(
    robot_id,
    left_joint,
    lateralFriction=1.2,
    rollingFriction=0.001
)

p.changeDynamics(
    robot_id,
    right_joint,
    lateralFriction=1.2,
    rollingFriction=0.001
)

# Lkora sghira khas-ha friction 9lila bach mat3ettelch dawran
p.changeDynamics(
    robot_id,
    caster_joint,
    lateralFriction=0.01,
    rollingFriction=0.001
)


# 7. Camera
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

p.resetDebugVisualizerCamera(
    cameraDistance=1.6,
    cameraYaw=45,
    cameraPitch=-35,
    cameraTargetPosition=[0.25, 0, 0.05]
)


# 8. Nkhalliw robot yهبط w ysta9er 1 seconde
for step in range(240):
    p.stepSimulation()
    time.sleep(1 / 240)


start_position, _ = p.getBasePositionAndOrientation(robot_id)

print("Position 9bel l7araka:", start_position)


# 9. N7erko roue lisra
p.setJointMotorControl2(
    bodyUniqueId=robot_id,
    jointIndex=left_joint,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity=4.0,
    force=2.0
)


# 10. N7erko roue limna
p.setJointMotorControl2(
    bodyUniqueId=robot_id,
    jointIndex=right_joint,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity=4.0,
    force=2.0
)


# 11. Robot ymchi 3 secondes
for step in range(720):
    p.stepSimulation()
    time.sleep(1 / 240)


# 12. Nw9fo jouj roues
p.setJointMotorControl2(
    robot_id,
    left_joint,
    p.VELOCITY_CONTROL,
    targetVelocity=0,
    force=2.0
)

p.setJointMotorControl2(
    robot_id,
    right_joint,
    p.VELOCITY_CONTROL,
    targetVelocity=0,
    force=2.0
)


# 13. Ntsennaw seconde
for step in range(240):
    p.stepSimulation()
    time.sleep(1 / 240)


final_position, final_orientation = (
    p.getBasePositionAndOrientation(robot_id)
)

print("Position finale:", final_position)

distance_x = final_position[0] - start_position[0]

print("Distance f jiha X:", distance_x, "metre")


p.disconnect()