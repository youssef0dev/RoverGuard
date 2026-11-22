import math
import random
import time

import pybullet as p
import pybullet_data

from odometry import DifferentialDriveOdometry


TIME_STEP = 1 / 240

WHEEL_RADIUS = 0.05
WHEEL_SEPARATION = 0.23

MAX_DISTANCE = 1.0
STOP_DISTANCE = 0.25
CLEAR_DISTANCE = 0.30

NOISE_STD = 0.02
FILTER_ALPHA = 0.20


connection_id = p.connect(p.GUI)

if connection_id < 0:
    raise RuntimeError("Ma9drnach nrbtou m3a PyBullet.")


try:
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(TIME_STEP)

    plane_id = p.loadURDF("plane.urdf")

    obstacle_id = p.loadURDF(
        "world/obstacle.urdf",
        basePosition=[0.90, 0, 0],
        useFixedBase=True,
    )

    robot_id = p.loadURDF(
        "robot/roverguard.urdf",
        basePosition=[0, 0, 0.15],
    )

    number_of_joints = p.getNumJoints(robot_id)
    print("Nombre de joints:", number_of_joints)

    joint_indices = {}

    for joint_index in range(number_of_joints):
        joint_info = p.getJointInfo(robot_id, joint_index)
        joint_name = joint_info[1].decode("utf-8")
        link_name = joint_info[12].decode("utf-8")

        joint_indices[joint_name] = joint_index

        print(
            "Index:",
            joint_index,
            "| Joint:",
            joint_name,
            "| Link:",
            link_name,
        )

    left_joint = joint_indices["left_wheel_joint"]
    right_joint = joint_indices["right_wheel_joint"]
    caster_joint = joint_indices["rear_caster_joint"]
    sensor_joint = joint_indices["front_sensor_joint"]

    p.changeDynamics(
        robot_id,
        left_joint,
        lateralFriction=1.2,
        rollingFriction=0.001,
    )

    p.changeDynamics(
        robot_id,
        right_joint,
        lateralFriction=1.2,
        rollingFriction=0.001,
    )

    p.changeDynamics(
        robot_id,
        caster_joint,
        lateralFriction=0.01,
        rollingFriction=0.001,
    )

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    p.resetDebugVisualizerCamera(
        cameraDistance=1.6,
        cameraYaw=45,
        cameraPitch=-35,
        cameraTargetPosition=[0.25, 0, 0.05],
    )

    for _ in range(240):
        p.stepSimulation()
        time.sleep(TIME_STEP)

    initial_left_angle = p.getJointState(
        robot_id,
        left_joint,
    )[0]

    initial_right_angle = p.getJointState(
        robot_id,
        right_joint,
    )[0]

    odometry = DifferentialDriveOdometry(
        wheel_radius=WHEEL_RADIUS,
        wheel_separation=WHEEL_SEPARATION,
    )

    odometry.reset(
        initial_left_angle,
        initial_right_angle,
    )

    start_real_position, _ = p.getBasePositionAndOrientation(robot_id)

    filtered_distance = None
    debug_line_id = -1
    safety_blocked = False
    counter = 0
    running = True

    print("")
    print("CONTROLS")
    print("Fleche fo9  : l9dam")
    print("Fleche ta7t : lwer")
    print("Fleche lisra/limna : dawran")
    print("Q : khroj")
    print("")

    while running:
        if not p.isConnected():
            print("\nPyBullet ma-b9ach connected.")
            break

        forward_speed = 0.0
        turn_speed = 0.0

        keys = p.getKeyboardEvents()

        if (
            p.B3G_UP_ARROW in keys
            and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN
        ):
            forward_speed = 5.0

        if (
            p.B3G_DOWN_ARROW in keys
            and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN
        ):
            forward_speed = -5.0

        if (
            p.B3G_LEFT_ARROW in keys
            and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN
        ):
            turn_speed = 3.0

        if (
            p.B3G_RIGHT_ARROW in keys
            and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN
        ):
            turn_speed = -3.0

        if ord("q") in keys and keys[ord("q")] & p.KEY_WAS_TRIGGERED:
            running = False
            break

        sensor_state = p.getLinkState(
            robot_id,
            sensor_joint,
            computeForwardKinematics=True,
        )

        sensor_position = sensor_state[0]
        sensor_orientation = sensor_state[1]

        rotation = p.getMatrixFromQuaternion(sensor_orientation)

        forward_vector = [
            rotation[0],
            rotation[3],
            rotation[6],
        ]

        ray_start = [
            sensor_position[0] + forward_vector[0] * 0.03,
            sensor_position[1] + forward_vector[1] * 0.03,
            sensor_position[2] + forward_vector[2] * 0.03,
        ]

        ray_end = [
            ray_start[0] + forward_vector[0] * MAX_DISTANCE,
            ray_start[1] + forward_vector[1] * MAX_DISTANCE,
            ray_start[2] + forward_vector[2] * MAX_DISTANCE,
        ]

        ray_result = p.rayTestBatch([ray_start], [ray_end])[0]

        hit_object_id = ray_result[0]
        hit_fraction = ray_result[2]

        if hit_object_id == -1:
            true_distance = MAX_DISTANCE
            line_end = ray_end
        else:
            true_distance = hit_fraction * MAX_DISTANCE
            line_end = ray_result[3]

        noise = random.gauss(0.0, NOISE_STD)
        raw_distance = true_distance + noise
        raw_distance = max(0.0, min(MAX_DISTANCE, raw_distance))

        if filtered_distance is None:
            filtered_distance = raw_distance
        else:
            filtered_distance = (
                FILTER_ALPHA * raw_distance
                + (1.0 - FILTER_ALPHA) * filtered_distance
            )

        if (
            not safety_blocked
            and filtered_distance < STOP_DISTANCE
        ):
            safety_blocked = True
            print("\nSTOP: obstacle 9rib!")

        elif (
            safety_blocked
            and filtered_distance > CLEAR_DISTANCE
        ):
            safety_blocked = False
            print("\nPATH CLEAR: robot y9der ymchi")

        if safety_blocked and forward_speed > 0:
            forward_speed = 0.0

        if safety_blocked:
            line_color = [1, 0, 0]
        else:
            line_color = [0, 1, 0]

        debug_line_id = p.addUserDebugLine(
            ray_start,
            line_end,
            line_color,
            lineWidth=3,
            replaceItemUniqueId=debug_line_id,
        )

        left_speed = forward_speed - turn_speed
        right_speed = forward_speed + turn_speed

        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=left_joint,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=left_speed,
            force=2.0,
        )

        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=right_joint,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=right_speed,
            force=2.0,
        )

        p.stepSimulation()
        time.sleep(TIME_STEP)

        left_angle = p.getJointState(robot_id, left_joint)[0]
        right_angle = p.getJointState(robot_id, right_joint)[0]

        odom_x, odom_y, odom_theta = odometry.update(
            left_angle,
            right_angle,
        )

        counter += 1

        if counter % 60 == 0:
            real_position, _ = p.getBasePositionAndOrientation(robot_id)

            real_x = real_position[0] - start_real_position[0]
            real_y = real_position[1] - start_real_position[1]

            print(
                f"\rSensor: {filtered_distance:.2f} m"
                f" | Odom X: {odom_x:.2f}"
                f" Y: {odom_y:.2f}"
                f" Angle: {math.degrees(odom_theta):.1f} deg"
                f" | Real X: {real_x:.2f}"
                f" Y: {real_y:.2f}",
                end="",
            )

    print("")

    if p.isConnected():
        final_position, _ = p.getBasePositionAndOrientation(robot_id)
        print("Position finale:", final_position)

except Exception as error:

    if not p.isConnected():

        print(
            "\nPyBullet tsed. Khoroj normal."
        )

    else:

        print("")
        print(
            "NO3 ERROR:",
            type(error).__name__
        )

        print(
            "MESSAGE ERROR:",
            error
        )

        input(
            "Dghat Enter bach nsddo programme..."
        )
finally:
    if p.isConnected():
        p.disconnect()
