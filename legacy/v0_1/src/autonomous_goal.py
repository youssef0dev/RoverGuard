import math
import random
import time

import pybullet as p
import pybullet_data

from controllers import GoToGoalController
from odometry import DifferentialDriveOdometry


TIME_STEP = 1 / 240
MAX_SIMULATION_STEPS = 240 * 30

WHEEL_RADIUS = 0.05
WHEEL_SEPARATION = 0.23
MOTOR_FORCE = 2.0

MAX_DISTANCE = 1.0
STOP_DISTANCE = 0.25
CLEAR_DISTANCE = 0.30
NOISE_STD = 0.02
FILTER_ALPHA = 0.20

GOAL_X = 0.50
GOAL_Y = 0.30


def set_wheel_speeds(
    robot_id,
    left_joint,
    right_joint,
    left_speed,
    right_speed,
):
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=left_joint,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=left_speed,
        force=MOTOR_FORCE,
    )

    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=right_joint,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=right_speed,
        force=MOTOR_FORCE,
    )


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

    joint_indices = {}

    for joint_index in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, joint_index)
        joint_name = joint_info[1].decode("utf-8")
        joint_indices[joint_name] = joint_index

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
        cameraDistance=1.8,
        cameraYaw=45,
        cameraPitch=-45,
        cameraTargetPosition=[0.25, 0.15, 0],
    )

    goal_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        radius=0.05,
        length=0.01,
        rgbaColor=[0.1, 0.9, 0.1, 1.0],
    )

    goal_marker_id = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=goal_visual,
        basePosition=[GOAL_X, GOAL_Y, 0.005],
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

    controller = GoToGoalController(
        wheel_radius=WHEEL_RADIUS,
        wheel_separation=WHEEL_SEPARATION,
    )

    start_real_position, _ = p.getBasePositionAndOrientation(robot_id)

    filtered_distance = None
    safety_blocked = False
    sensor_line_id = -1
    goal_line_id = -1
    mission_finished = False

    print("")
    print("MISSION AUTONOME")
    print(f"Goal X: {GOAL_X:.2f} m | Goal Y: {GOAL_Y:.2f} m")
    print("Robot ghadi ymchi bo7do.")
    print("")

    for simulation_step in range(MAX_SIMULATION_STEPS):
        if not p.isConnected():
            break

        left_angle = p.getJointState(robot_id, left_joint)[0]
        right_angle = p.getJointState(robot_id, right_joint)[0]

        odom_x, odom_y, odom_theta = odometry.update(
            left_angle,
            right_angle,
        )

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

        if ray_result[0] == -1:
            true_distance = MAX_DISTANCE
            sensor_line_end = ray_end
        else:
            true_distance = ray_result[2] * MAX_DISTANCE
            sensor_line_end = ray_result[3]

        raw_distance = true_distance + random.gauss(0.0, NOISE_STD)
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
            print("\nSTOP SAFETY: obstacle 9rib!")

        elif (
            safety_blocked
            and filtered_distance > CLEAR_DISTANCE
        ):
            safety_blocked = False
            print("\nPATH CLEAR")

        (
            left_speed,
            right_speed,
            goal_distance,
            angle_error,
            goal_reached,
        ) = controller.compute(
            odom_x,
            odom_y,
            odom_theta,
            GOAL_X,
            GOAL_Y,
        )

        if goal_reached:
            set_wheel_speeds(
                robot_id,
                left_joint,
                right_joint,
                0.0,
                0.0,
            )

            mission_finished = True
            print("\nMISSION SUCCESS: robot wsel l goal!")
            break

        if safety_blocked:
            left_speed = 0.0
            right_speed = 0.0

        set_wheel_speeds(
            robot_id,
            left_joint,
            right_joint,
            left_speed,
            right_speed,
        )

        sensor_color = [1, 0, 0] if safety_blocked else [0, 1, 0]

        sensor_line_id = p.addUserDebugLine(
            ray_start,
            sensor_line_end,
            sensor_color,
            lineWidth=3,
            replaceItemUniqueId=sensor_line_id,
        )

        real_position, _ = p.getBasePositionAndOrientation(robot_id)

        goal_line_id = p.addUserDebugLine(
            [real_position[0], real_position[1], 0.08],
            [GOAL_X, GOAL_Y, 0.08],
            [0.2, 0.4, 1.0],
            lineWidth=2,
            replaceItemUniqueId=goal_line_id,
        )

        p.stepSimulation()
        time.sleep(TIME_STEP)

        if simulation_step % 60 == 0:
            real_x = real_position[0] - start_real_position[0]
            real_y = real_position[1] - start_real_position[1]

            print(
                f"\rGoal distance: {goal_distance:.2f} m"
                f" | Angle error: {math.degrees(angle_error):.1f} deg"
                f" | Odom: ({odom_x:.2f}, {odom_y:.2f})"
                f" | Real: ({real_x:.2f}, {real_y:.2f})",
                end="",
            )

    if not mission_finished and p.isConnected():
        set_wheel_speeds(
            robot_id,
            left_joint,
            right_joint,
            0.0,
            0.0,
        )

        print("\nMISSION STOPPED: timeout wla fenêtre tsedat.")

    if p.isConnected():
        for _ in range(240):
            p.stepSimulation()
            time.sleep(TIME_STEP)

except Exception as error:
    if not p.isConnected():
        print("\nPyBullet tsed. Khoroj normal.")
    else:
        print("")
        print("NO3 ERROR:", type(error).__name__)
        print("MESSAGE ERROR:", error)
        input("Dghat Enter bach nsddo programme...")

finally:
    if p.isConnected():
        p.disconnect()
