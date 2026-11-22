import math
import random
import time

import pybullet as p
import pybullet_data

from controllers import GoToGoalController, normalize_angle
from mission_logger import MissionLogger
from odometry import DifferentialDriveOdometry


TIME_STEP = 1 / 240
MAX_SIMULATION_STEPS = 240 * 90
INSPECTION_STEPS = 120

WHEEL_RADIUS = 0.05
WHEEL_SEPARATION = 0.23
MOTOR_FORCE = 2.0

# Sensor w safety
SENSOR_RANGE = 1.0
AVOID_TRIGGER_DISTANCE = 0.35
EMERGENCY_DISTANCE = 0.12

# BODY_QUERY_DISTANCE hiya ghir ch7al mn metre n9elbou 3la obstacle.
# Khass-ha tkoun kbar mn BODY_CLEARANCE_TRIGGER bach ma-ytra-ch
# trigger b sabab ta9rib dyal ar9am.
BODY_QUERY_DISTANCE = 0.30
BODY_CLEARANCE_TRIGGER = 0.12
BODY_EMERGENCY_CLEARANCE = 0.03

SIDE_SENSOR_ANGLE = math.radians(45)
NOISE_STD = 0.01
FILTER_ALPHA = 0.20

# Localization w logging
LOCALIZATION_INTERVAL_STEPS = 24
LOCALIZATION_POSITION_STD = 0.004
LOCALIZATION_ANGLE_STD = math.radians(0.5)
LOCALIZATION_GAIN = 0.25
LOG_INTERVAL_STEPS = 12

# Navigation
# Robot dyalna 30 cm tol; 1.5 cm tolerance kanet 9as7a bzaf.
GOAL_TOLERANCE = 0.08

# 0.80 m kanet katdir dawra kbira. 0.45 m katb9a amna lhad obstacle,
# walakin had reactive avoidance mazal machi a9sar tari9 b7al A*.
BYPASS_DISTANCE = 0.45
BYPASS_LINEAR_SPEED = 0.15
TURN_TOLERANCE = math.radians(5)
MAX_TURN_SPEED = 1.6

# P1 t7erket 10 cm l9dam bach jsm robot kamel ma-yb9ach
# 7da obstacle b marge sghira bzaf.
WAYPOINTS = [
    ("P1", 1.15, 0.00),
    ("P2", 1.15, 0.55),
    ("P3", 0.00, 0.55),
    ("BASE", 0.00, 0.00),
]


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


def robot_speeds_to_wheels(linear_speed, angular_speed):
    left_linear_speed = (
        linear_speed - angular_speed * WHEEL_SEPARATION / 2.0
    )
    right_linear_speed = (
        linear_speed + angular_speed * WHEEL_SEPARATION / 2.0
    )

    return (
        left_linear_speed / WHEEL_RADIUS,
        right_linear_speed / WHEEL_RADIUS,
    )


def rotated_direction(forward_vector, angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)

    return [
        cosine * forward_vector[0] - sine * forward_vector[1],
        sine * forward_vector[0] + cosine * forward_vector[1],
        forward_vector[2],
    ]


def cast_sensor_ray(sensor_position, direction):
    ray_start = [
        sensor_position[0] + direction[0] * 0.03,
        sensor_position[1] + direction[1] * 0.03,
        sensor_position[2] + direction[2] * 0.03,
    ]

    ray_end = [
        ray_start[0] + direction[0] * SENSOR_RANGE,
        ray_start[1] + direction[1] * SENSOR_RANGE,
        ray_start[2] + direction[2] * SENSOR_RANGE,
    ]

    result = p.rayTestBatch([ray_start], [ray_end])[0]

    if result[0] == -1:
        true_distance = SENSOR_RANGE
        line_end = ray_end
    else:
        true_distance = result[2] * SENSOR_RANGE
        line_end = result[3]

    noisy_distance = true_distance + random.gauss(0.0, NOISE_STD)
    noisy_distance = max(0.0, min(SENSOR_RANGE, noisy_distance))

    return noisy_distance, ray_start, line_end


def create_waypoint_markers():
    marker_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        radius=0.045,
        length=0.01,
        rgbaColor=[0.1, 0.9, 0.1, 1.0],
    )

    for name, x, y in WAYPOINTS:
        # Visual fa9at: ma-kayn 7ta collision shape.
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=marker_visual,
            basePosition=[x, y, 0.005],
        )

        p.addUserDebugText(
            name,
            [x, y, 0.07],
            textColorRGB=[0.1, 0.1, 0.1],
            textSize=1.2,
        )


logger = None
connection_id = p.connect(p.GUI)

if connection_id < 0:
    raise RuntimeError("Ma9drnach nrbtou m3a PyBullet.")


try:
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(TIME_STEP)

    p.loadURDF("plane.urdf")

    obstacle_id = p.loadURDF(
        "world/avoidance_obstacle.urdf",
        basePosition=[0.70, 0.00, 0],
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
        cameraDistance=2.7,
        cameraYaw=45,
        cameraPitch=-55,
        cameraTargetPosition=[0.45, 0.25, 0],
    )

    create_waypoint_markers()

    for _ in range(240):
        p.stepSimulation()
        time.sleep(TIME_STEP)

    initial_left_angle = p.getJointState(robot_id, left_joint)[0]
    initial_right_angle = p.getJointState(robot_id, right_joint)[0]

    odometry = DifferentialDriveOdometry(
        wheel_radius=WHEEL_RADIUS,
        wheel_separation=WHEEL_SEPARATION,
    )
    odometry.reset(initial_left_angle, initial_right_angle)

    controller = GoToGoalController(
        wheel_radius=WHEEL_RADIUS,
        wheel_separation=WHEEL_SEPARATION,
        distance_tolerance=GOAL_TOLERANCE,
    )

    start_real_position, start_real_orientation = (
        p.getBasePositionAndOrientation(robot_id)
    )
    start_real_yaw = p.getEulerFromQuaternion(
        start_real_orientation
    )[2]

    current_waypoint_index = 0
    navigation_state = "NAVIGATE"
    avoid_direction = 1.0
    avoid_target_theta = 0.0
    bypass_start_x = 0.0
    bypass_start_y = 0.0

    filtered_front_distance = None
    mission_finished = False

    logger = MissionLogger("logs")

    print("")
    print("ROVERGUARD AVOIDANCE PATROL - FIXED")
    print("State: NAVIGATE -> TURN_AWAY -> BYPASS -> NAVIGATE")
    print("Mission log:", logger.path)
    print("")

    for simulation_step in range(MAX_SIMULATION_STEPS):
        if not p.isConnected():
            break

        waypoint_name, goal_x, goal_y = WAYPOINTS[
            current_waypoint_index
        ]

        left_angle = p.getJointState(robot_id, left_joint)[0]
        right_angle = p.getJointState(robot_id, right_joint)[0]

        odom_x, odom_y, odom_theta = odometry.update(
            left_angle,
            right_angle,
        )

        if simulation_step % LOCALIZATION_INTERVAL_STEPS == 0:
            real_position, real_orientation = (
                p.getBasePositionAndOrientation(robot_id)
            )
            real_yaw = p.getEulerFromQuaternion(real_orientation)[2]

            measured_x = (
                real_position[0]
                - start_real_position[0]
                + random.gauss(0.0, LOCALIZATION_POSITION_STD)
            )
            measured_y = (
                real_position[1]
                - start_real_position[1]
                + random.gauss(0.0, LOCALIZATION_POSITION_STD)
            )
            measured_theta = normalize_angle(
                real_yaw
                - start_real_yaw
                + random.gauss(0.0, LOCALIZATION_ANGLE_STD)
            )

            odom_x, odom_y, odom_theta = odometry.correct_pose(
                measured_x,
                measured_y,
                measured_theta,
                LOCALIZATION_GAIN,
            )

        sensor_state = p.getLinkState(
            robot_id,
            sensor_joint,
            computeForwardKinematics=True,
        )
        sensor_position = sensor_state[0]
        sensor_orientation = sensor_state[1]
        rotation = p.getMatrixFromQuaternion(sensor_orientation)

        forward_vector = [rotation[0], rotation[3], rotation[6]]
        left_direction = rotated_direction(
            forward_vector,
            SIDE_SENSOR_ANGLE,
        )
        right_direction = rotated_direction(
            forward_vector,
            -SIDE_SENSOR_ANGLE,
        )

        front_distance, front_start, front_end = cast_sensor_ray(
            sensor_position,
            forward_vector,
        )
        left_distance, left_start, left_end = cast_sensor_ray(
            sensor_position,
            left_direction,
        )
        right_distance, right_start, right_end = cast_sensor_ray(
            sensor_position,
            right_direction,
        )

        if filtered_front_distance is None:
            filtered_front_distance = front_distance
        else:
            filtered_front_distance = (
                FILTER_ALPHA * front_distance
                + (1.0 - FILTER_ALPHA) * filtered_front_distance
            )

        closest_points = p.getClosestPoints(
            bodyA=robot_id,
            bodyB=obstacle_id,
            distance=BODY_QUERY_DISTANCE,
        )

        if closest_points:
            body_clearance = min(point[8] for point in closest_points)
        else:
            body_clearance = BODY_QUERY_DISTANCE

        left_speed = 0.0
        right_speed = 0.0
        goal_distance = math.hypot(goal_x - odom_x, goal_y - odom_y)
        event = ""

        emergency_detected = (
            filtered_front_distance < EMERGENCY_DISTANCE
            or body_clearance < BODY_EMERGENCY_CLEARANCE
        )
        obstacle_detected = (
            filtered_front_distance < AVOID_TRIGGER_DISTANCE
            or body_clearance < BODY_CLEARANCE_TRIGGER
        )

        if navigation_state == "NAVIGATE":
            # Tartib mohim:
            # 1) collision/emergency
            # 2) goal reached
            # 3) obstacle avoidance
            # 4) normal navigation
            if emergency_detected:
                avoid_direction = (
                    1.0 if left_distance >= right_distance else -1.0
                )
                avoid_target_theta = normalize_angle(
                    odom_theta + avoid_direction * math.pi / 2.0
                )
                navigation_state = "TURN_AWAY"
                print("\nEMERGENCY: obstacle las9, ghadi nb3do")
                event = "EMERGENCY_FROM_NAVIGATE"

            elif goal_distance <= GOAL_TOLERANCE:
                set_wheel_speeds(
                    robot_id,
                    left_joint,
                    right_joint,
                    0.0,
                    0.0,
                )

                print(f"\n{waypoint_name} WSEL: inspection...")
                event = f"WAYPOINT_REACHED_{waypoint_name}"

                logger.log(
                    simulation_time_s=simulation_step * TIME_STEP,
                    state=navigation_state,
                    target=waypoint_name,
                    odom_x=odom_x,
                    odom_y=odom_y,
                    odom_theta_deg=math.degrees(odom_theta),
                    front_distance=filtered_front_distance,
                    left_distance=left_distance,
                    right_distance=right_distance,
                    body_clearance=body_clearance,
                    event=event,
                )

                for _ in range(INSPECTION_STEPS):
                    if not p.isConnected():
                        break
                    p.stepSimulation()
                    time.sleep(TIME_STEP)

                current_waypoint_index += 1

                if current_waypoint_index >= len(WAYPOINTS):
                    mission_finished = True
                    print("MISSION SUCCESS: patrol salat w robot rje3 l-base!")
                    logger.log(
                        simulation_time_s=simulation_step * TIME_STEP,
                        state="MISSION_COMPLETE",
                        target=waypoint_name,
                        odom_x=odom_x,
                        odom_y=odom_y,
                        odom_theta_deg=math.degrees(odom_theta),
                        front_distance=filtered_front_distance,
                        left_distance=left_distance,
                        right_distance=right_distance,
                        body_clearance=body_clearance,
                        event="MISSION_SUCCESS",
                    )
                    break

                next_name = WAYPOINTS[current_waypoint_index][0]
                print(f"NEXT TARGET: {next_name}")
                continue

            elif obstacle_detected:
                avoid_direction = (
                    1.0 if left_distance >= right_distance else -1.0
                )
                avoid_target_theta = normalize_angle(
                    odom_theta + avoid_direction * math.pi / 2.0
                )
                navigation_state = "TURN_AWAY"

                side_name = "LISRA" if avoid_direction > 0 else "LIMNA"
                print(f"\nOBSTACLE: ghadi ndor {side_name}")
                event = f"OBSTACLE_TURN_{side_name}"

            else:
                (
                    left_speed,
                    right_speed,
                    goal_distance,
                    _angle_error,
                    _goal_reached,
                ) = controller.compute(
                    odom_x,
                    odom_y,
                    odom_theta,
                    goal_x,
                    goal_y,
                )

        elif navigation_state == "TURN_AWAY":
            turn_error = normalize_angle(avoid_target_theta - odom_theta)

            if abs(turn_error) <= TURN_TOLERANCE:
                navigation_state = "BYPASS"
                bypass_start_x = odom_x
                bypass_start_y = odom_y
                print("TURN COMPLETE: daba ghandouz 7da obstacle")
                event = "TURN_COMPLETE"
            else:
                angular_speed = max(
                    -MAX_TURN_SPEED,
                    min(MAX_TURN_SPEED, 3.0 * turn_error),
                )
                left_speed, right_speed = robot_speeds_to_wheels(
                    0.0,
                    angular_speed,
                )

        elif navigation_state == "BYPASS":
            bypass_travelled = math.hypot(
                odom_x - bypass_start_x,
                odom_y - bypass_start_y,
            )

            if bypass_travelled >= BYPASS_DISTANCE:
                navigation_state = "NAVIGATE"
                print("BYPASS COMPLETE: nrj3o l target")
                event = "BYPASS_COMPLETE"
            elif emergency_detected:
                avoid_target_theta = normalize_angle(
                    odom_theta + avoid_direction * math.radians(45)
                )
                navigation_state = "TURN_AWAY"
                print("EMERGENCY TURN")
                event = "EMERGENCY_TURN"
            else:
                left_speed, right_speed = robot_speeds_to_wheels(
                    BYPASS_LINEAR_SPEED,
                    0.0,
                )

        set_wheel_speeds(
            robot_id,
            left_joint,
            right_joint,
            left_speed,
            right_speed,
        )

        front_color = (
            [1, 0, 0]
            if filtered_front_distance < AVOID_TRIGGER_DISTANCE
            else [0, 1, 0]
        )

        if simulation_step % 4 == 0:
            p.addUserDebugLine(
                front_start,
                front_end,
                front_color,
                lineWidth=3,
                lifeTime=0.10,
            )
            p.addUserDebugLine(
                left_start,
                left_end,
                [0.2, 0.6, 1.0],
                lineWidth=2,
                lifeTime=0.10,
            )
            p.addUserDebugLine(
                right_start,
                right_end,
                [0.2, 0.6, 1.0],
                lineWidth=2,
                lifeTime=0.10,
            )

            real_position, _ = p.getBasePositionAndOrientation(robot_id)
            p.addUserDebugLine(
                [real_position[0], real_position[1], 0.08],
                [goal_x, goal_y, 0.08],
                [0.7, 0.2, 1.0],
                lineWidth=2,
                lifeTime=0.10,
            )

        p.stepSimulation()
        time.sleep(TIME_STEP)

        if simulation_step % LOG_INTERVAL_STEPS == 0 or event:
            logger.log(
                simulation_time_s=simulation_step * TIME_STEP,
                state=navigation_state,
                target=waypoint_name,
                odom_x=odom_x,
                odom_y=odom_y,
                odom_theta_deg=math.degrees(odom_theta),
                front_distance=filtered_front_distance,
                left_distance=left_distance,
                right_distance=right_distance,
                body_clearance=body_clearance,
                event=event,
            )

        if simulation_step % 60 == 0:
            print(
                f"\rState: {navigation_state}"
                f" | Target: {waypoint_name}"
                f" | Goal: {goal_distance:.2f} m"
                f" | Front: {filtered_front_distance:.2f} m"
                f" | Left: {left_distance:.2f} m"
                f" | Right: {right_distance:.2f} m"
                f" | Body: {body_clearance:.2f} m",
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
    if logger is not None:
        logger.close()

    if p.isConnected():
        p.disconnect()
