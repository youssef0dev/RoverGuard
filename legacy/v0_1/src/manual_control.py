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


# 8. Nkhalliw robot yهبط w ysta9er
for step in range(240):
    p.stepSimulation()
    time.sleep(1 / 240)


print("")
print("CONTROLS:")
print("Fleche fo9   = lmchi l9dam")
print("Fleche ta7t  = lmchi lwer")
print("Fleche lisra = dawran lisra")
print("Fleche limna = dawran limna")
print("Q            = khroj")
print("")


running = True

while running:

    # F kol doura kanbdaw b sor3a zero
    forward_speed = 0.0
    turn_speed = 0.0

    # N9raw clavier
    keys = p.getKeyboardEvents()

    # L9dam
    if (
        p.B3G_UP_ARROW in keys
        and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN
    ):
        forward_speed = 5.0

    # Lwer
    if (
        p.B3G_DOWN_ARROW in keys
        and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN
    ):
        forward_speed = -5.0

    # Dawran lisra
    if (
        p.B3G_LEFT_ARROW in keys
        and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN
    ):
        turn_speed = 3.0

    # Dawran limna
    if (
        p.B3G_RIGHT_ARROW in keys
        and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN
    ):
        turn_speed = -3.0

    # Q bach nkhrjo
    if ord("q") in keys:
        if keys[ord("q")] & p.KEY_WAS_TRIGGERED:
            running = False

    # Nkhlto l7araka lmosta9ima m3a dawran
    left_speed = forward_speed - turn_speed
    right_speed = forward_speed + turn_speed

    # Moteur roue lisra
    p.setJointMotorControl2(
        robot_id,
        left_joint,
        p.VELOCITY_CONTROL,
        targetVelocity=left_speed,
        force=2.0
    )

    # Moteur roue limna
    p.setJointMotorControl2(
        robot_id,
        right_joint,
        p.VELOCITY_CONTROL,
        targetVelocity=right_speed,
        force=2.0
    )

    p.stepSimulation()
    time.sleep(1 / 240)


final_position, final_orientation = (
    p.getBasePositionAndOrientation(robot_id)
)

print("Position mlli salina:", final_position)

p.disconnect()