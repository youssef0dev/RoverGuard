import argparse
import math
import random
import time
from dataclasses import replace
from pathlib import Path

from .astar import AStarPlanner, PathNotFoundError, PlannedPath
from .config import AppConfig
from .controllers import PointController
from .geometry import Point2D, Pose2D, normalize_angle
from .grid_map import OccupancyGrid
from .kinematics import twist_to_wheel_speeds
from .mission import PatrolMission
from .odometry import DifferentialDriveOdometry
from .path_follower import PathFollower
from .safety import SafetyState, SafetySupervisor
from .sensors import LowPassFilter, RangeReading, noisy_clamped_distance, rotate_xy
from .telemetry import TelemetryLogger
from .world_map import WorldMap, load_world_map


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WORLD_MAP = PROJECT_ROOT / "world" / "maps" / "warehouse_v1.json"


def build_grid(config: AppConfig, world_map: WorldMap | None = None) -> OccupancyGrid:
    active_map = world_map or load_world_map(DEFAULT_WORLD_MAP)
    planner = config.planner
    grid = OccupancyGrid(
        min_x_m=active_map.bounds.min_x_m,
        max_x_m=active_map.bounds.max_x_m,
        min_y_m=active_map.bounds.min_y_m,
        max_y_m=active_map.bounds.max_y_m,
        resolution_m=planner.resolution_m,
    )
    inflation = config.robot.footprint_radius_m + planner.safety_margin_m
    for obstacle in active_map.obstacles:
        grid.add_obstacle(obstacle.aabb, inflation)
    return grid


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RoverGuard v0.4 multi-point patrol")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without the PyBullet window.",
    )
    parser.add_argument(
        "--no-realtime",
        action="store_true",
        help="Do not sleep between physics steps (useful for tests).",
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-seconds", type=float, default=None)
    parser.add_argument(
        "--map",
        default=str(DEFAULT_WORLD_MAP),
        help="Path to a RoverGuard world-map JSON file.",
    )
    return parser.parse_args()


def create_obstacles(pybullet, world_map: WorldMap) -> list[int]:
    obstacle_ids: list[int] = []
    for obstacle in world_map.obstacles:
        half_extents = [
            obstacle.size_x_m / 2.0,
            obstacle.size_y_m / 2.0,
            obstacle.height_m / 2.0,
        ]
        collision = pybullet.createCollisionShape(
            pybullet.GEOM_BOX,
            halfExtents=half_extents,
        )
        visual = pybullet.createVisualShape(
            pybullet.GEOM_BOX,
            halfExtents=half_extents,
            rgbaColor=list(obstacle.color_rgba),
        )
        obstacle_id = pybullet.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=[
                obstacle.center.x,
                obstacle.center.y,
                obstacle.height_m / 2.0,
            ],
        )
        obstacle_ids.append(obstacle_id)
        pybullet.addUserDebugText(
            obstacle.name,
            [
                obstacle.center.x,
                obstacle.center.y,
                obstacle.height_m + 0.06,
            ],
            textColorRGB=[0.15, 0.15, 0.15],
            textSize=0.85,
        )
    return obstacle_ids


def create_waypoint_markers(pybullet, waypoints) -> None:
    marker_visual = pybullet.createVisualShape(
        shapeType=pybullet.GEOM_CYLINDER,
        radius=0.045,
        length=0.01,
        rgbaColor=[0.1, 0.9, 0.1, 1.0],
    )
    for name, point in waypoints:
        pybullet.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=marker_visual,
            basePosition=[point.x, point.y, 0.005],
        )
        pybullet.addUserDebugText(
            name,
            [point.x, point.y, 0.07],
            textColorRGB=[0.1, 0.1, 0.1],
            textSize=1.2,
        )


def cast_ray(
    pybullet,
    sensor_position,
    direction,
    max_distance_m: float,
    noise_std_m: float,
    rng: random.Random,
) -> RangeReading:
    ray_start = (
        sensor_position[0] + direction[0] * 0.03,
        sensor_position[1] + direction[1] * 0.03,
        sensor_position[2] + direction[2] * 0.03,
    )
    requested_end = (
        ray_start[0] + direction[0] * max_distance_m,
        ray_start[1] + direction[1] * max_distance_m,
        ray_start[2] + direction[2] * max_distance_m,
    )
    result = pybullet.rayTestBatch([ray_start], [requested_end])[0]
    if result[0] == -1:
        true_distance = max_distance_m
        display_end = requested_end
    else:
        true_distance = result[2] * max_distance_m
        display_end = tuple(result[3])

    measured = noisy_clamped_distance(
        true_distance,
        max_distance_m,
        noise_std_m,
        rng,
    )
    return RangeReading(measured, ray_start, display_end)


def set_wheel_speeds(
    pybullet,
    robot_id: int,
    left_joint: int,
    right_joint: int,
    left_rad_s: float,
    right_rad_s: float,
    motor_force_n: float,
) -> None:
    for joint, speed in (
        (left_joint, left_rad_s),
        (right_joint, right_rad_s),
    ):
        pybullet.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint,
            controlMode=pybullet.VELOCITY_CONTROL,
            targetVelocity=speed,
            force=motor_force_n,
        )


def draw_path(pybullet, path: PlannedPath, previous_ids: list[int]) -> list[int]:
    for item_id in previous_ids:
        pybullet.removeUserDebugItem(item_id)
    ids: list[int] = []
    for first, second in zip(path.points, path.points[1:]):
        ids.append(
            pybullet.addUserDebugLine(
                [first.x, first.y, 0.04],
                [second.x, second.y, 0.04],
                [0.65, 0.1, 0.85],
                lineWidth=4,
            )
        )
    return ids


def run(
    args: argparse.Namespace,
    *,
    mission_waypoints=None,
    max_linear_speed_mps: float | None = None,
    mission_label: str | None = None,
    world_map: WorldMap | None = None,
) -> int:
    try:
        import pybullet as p
        import pybullet_data
    except ImportError as error:
        raise SystemExit(
            "PyBullet ma-minstallach. Run: python -m pip install pybullet==3.2.7"
        ) from error

    config = AppConfig()
    active_map = world_map or load_world_map(
        getattr(args, "map", str(DEFAULT_WORLD_MAP))
    )
    if max_linear_speed_mps is not None:
        if not 0.0 < max_linear_speed_mps <= config.robot.max_linear_speed_mps:
            raise ValueError("Mission speed is outside robot safety limits")
        config = replace(
            config,
            robot=replace(
                config.robot,
                max_linear_speed_mps=max_linear_speed_mps,
            ),
        )
    active_waypoints = (
        tuple(mission_waypoints)
        if mission_waypoints is not None
        else active_map.resolve_route(active_map.default_route)
    )
    if not active_waypoints:
        raise ValueError("Simulation mission cannot be empty")
    if args.max_seconds is not None:
        max_seconds = args.max_seconds
    else:
        max_seconds = config.simulation.max_seconds

    rng = random.Random(args.seed)
    connection_mode = p.DIRECT if args.headless else p.GUI
    connection_id = p.connect(connection_mode)
    if connection_id < 0:
        raise RuntimeError("Ma9drnach nrbtou m3a PyBullet.")

    logger: TelemetryLogger | None = None
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(config.simulation.time_step_s)
        p.loadURDF("plane.urdf")

        obstacle_ids = create_obstacles(p, active_map)
        robot_id = p.loadURDF(
            str(PROJECT_ROOT / "robot" / "roverguard.urdf"),
            basePosition=[
                active_map.robot_start.x,
                active_map.robot_start.y,
                0.15,
            ],
        )

        joints: dict[str, int] = {}
        for joint_index in range(p.getNumJoints(robot_id)):
            info = p.getJointInfo(robot_id, joint_index)
            joints[info[1].decode("utf-8")] = joint_index

        left_joint = joints["left_wheel_joint"]
        right_joint = joints["right_wheel_joint"]
        caster_joint = joints["rear_caster_joint"]
        sensor_joint = joints["front_sensor_joint"]

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

        if not args.headless:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            map_width = active_map.bounds.max_x_m - active_map.bounds.min_x_m
            map_height = active_map.bounds.max_y_m - active_map.bounds.min_y_m
            p.resetDebugVisualizerCamera(
                cameraDistance=max(map_width, map_height) * 0.95,
                cameraYaw=45,
                cameraPitch=-65,
                cameraTargetPosition=[
                    (active_map.bounds.min_x_m + active_map.bounds.max_x_m) / 2.0,
                    (active_map.bounds.min_y_m + active_map.bounds.max_y_m) / 2.0,
                    0,
                ],
            )

        create_waypoint_markers(p, active_map.waypoints)

        for _ in range(240):
            p.stepSimulation()
            if not args.no_realtime:
                time.sleep(config.simulation.time_step_s)

        left_angle = p.getJointState(robot_id, left_joint)[0]
        right_angle = p.getJointState(robot_id, right_joint)[0]
        odometry = DifferentialDriveOdometry(
            config.robot.wheel_radius_m,
            config.robot.wheel_separation_m,
        )
        odometry.reset(left_angle, right_angle)

        controller = PointController(
            max_linear_m_s=config.robot.max_linear_speed_mps,
            max_angular_rad_s=config.robot.max_angular_speed_rps,
            distance_tolerance_m=config.mission.goal_tolerance_m,
        )
        follower = PathFollower(controller)
        mission = PatrolMission(active_waypoints)
        grid = build_grid(config, active_map)
        planner = AStarPlanner(grid, config.planner.allow_diagonal)
        safety = SafetySupervisor(
            config.sensor.stop_distance_m,
            config.sensor.slow_distance_m,
            config.sensor.clear_distance_m,
        )
        front_filter = LowPassFilter(config.sensor.filter_alpha)

        start_position, start_orientation = p.getBasePositionAndOrientation(robot_id)
        start_yaw = p.getEulerFromQuaternion(start_orientation)[2]
        logger = TelemetryLogger(PROJECT_ROOT / "logs")
        path_line_ids: list[int] = []

        def plan_for_current(pose: Pose2D) -> PlannedPath:
            target = mission.current
            if target is None:
                raise RuntimeError("Cannot plan a completed mission")
            path = planner.plan(pose.point, target.point)
            follower.set_path(path.points)
            if not args.headless:
                nonlocal path_line_ids
                path_line_ids = draw_path(p, path, path_line_ids)
            print(
                f"\nPLAN {target.name}: {len(path.raw_cells)} grid cells"
                f" -> {len(path.points)} path points | {path.length_m:.2f} m"
            )
            return path

        current_pose = odometry.pose
        plan_for_current(current_pose)

        route_text = " -> ".join(name for name, _ in active_waypoints)
        print("\nROVERGUARD v0.4 - MULTI-POINT COGNITIVE PATROL")
        print("Map:", active_map.name)
        if mission_label:
            print("Mission:", mission_label)
        print("Route:", route_text)
        print("Log:", logger.path)

        max_steps = int(max_seconds / config.simulation.time_step_s)
        inspection_steps = int(
            config.simulation.inspection_seconds / config.simulation.time_step_s
        )
        inspection_remaining = 0
        mission_success = False

        for step in range(max_steps):
            if not p.isConnected():
                break

            left_angle = p.getJointState(robot_id, left_joint)[0]
            right_angle = p.getJointState(robot_id, right_joint)[0]
            current_pose = odometry.update(left_angle, right_angle)

            if step % config.simulation.localization_interval_steps == 0:
                real_position, real_orientation = p.getBasePositionAndOrientation(robot_id)
                real_yaw = p.getEulerFromQuaternion(real_orientation)[2]
                measured = Pose2D(
                    real_position[0]
                    - start_position[0]
                    + rng.gauss(0.0, config.simulation.localization_position_std_m),
                    real_position[1]
                    - start_position[1]
                    + rng.gauss(0.0, config.simulation.localization_position_std_m),
                    normalize_angle(
                        real_yaw
                        - start_yaw
                        + rng.gauss(
                            0.0,
                            math.radians(
                                config.simulation.localization_angle_std_deg
                            ),
                        )
                    ),
                )
                current_pose = odometry.correct(
                    measured,
                    config.simulation.localization_gain,
                )

            sensor_state = p.getLinkState(
                robot_id,
                sensor_joint,
                computeForwardKinematics=True,
            )
            sensor_position = sensor_state[0]
            rotation = p.getMatrixFromQuaternion(sensor_state[1])
            forward = (rotation[0], rotation[3], rotation[6])
            side_angle = math.radians(config.sensor.side_angle_deg)

            front = cast_ray(
                p,
                sensor_position,
                forward,
                config.sensor.range_m,
                config.sensor.noise_std_m,
                rng,
            )
            left = cast_ray(
                p,
                sensor_position,
                rotate_xy(forward, side_angle),
                config.sensor.range_m,
                config.sensor.noise_std_m,
                rng,
            )
            right = cast_ray(
                p,
                sensor_position,
                rotate_xy(forward, -side_angle),
                config.sensor.range_m,
                config.sensor.noise_std_m,
                rng,
            )
            filtered_front = front_filter.update(front.distance_m)

            if any(p.getContactPoints(robot_id, item_id) for item_id in obstacle_ids):
                set_wheel_speeds(
                    p,
                    robot_id,
                    left_joint,
                    right_joint,
                    0.0,
                    0.0,
                    config.robot.motor_force_n,
                )
                print("\nMISSION FAILED: robot lmes obstacle.")
                return 2

            event = ""
            linear_command = 0.0
            angular_command = 0.0

            if inspection_remaining > 0:
                inspection_remaining -= 1
                if inspection_remaining == 0:
                    reached = mission.mark_current_reached()
                    print(f"\n{reached.name} WSEL")
                    if mission.complete:
                        mission_success = True
                        print("MISSION SUCCESS:", route_text)
                        event = "MISSION_SUCCESS"
                    else:
                        plan_for_current(current_pose)
                        print("NEXT TARGET:", mission.current.name)
                        event = "NEXT_TARGET"
            else:
                command = follower.compute(current_pose)
                if command.reached and follower.finished:
                    inspection_remaining = inspection_steps
                    event = f"INSPECT_{mission.current.name}"
                else:
                    decision = safety.evaluate(filtered_front)
                    linear_command = command.linear_m_s * decision.speed_scale
                    angular_command = command.angular_rad_s
                    if decision.state == SafetyState.STOP:
                        linear_command = 0.0
                        event = "SAFETY_STOP"

            wheel_command = twist_to_wheel_speeds(
                linear_command,
                angular_command,
                config.robot.wheel_radius_m,
                config.robot.wheel_separation_m,
            )
            set_wheel_speeds(
                p,
                robot_id,
                left_joint,
                right_joint,
                wheel_command.left_rad_s,
                wheel_command.right_rad_s,
                config.robot.motor_force_n,
            )

            if not args.headless and step % 4 == 0:
                p.addUserDebugLine(
                    front.ray_start,
                    front.ray_end,
                    [1, 0, 0] if filtered_front < config.sensor.slow_distance_m else [0, 1, 0],
                    lineWidth=3,
                    lifeTime=0.10,
                )
                p.addUserDebugLine(
                    left.ray_start,
                    left.ray_end,
                    [0.2, 0.6, 1.0],
                    lineWidth=2,
                    lifeTime=0.10,
                )
                p.addUserDebugLine(
                    right.ray_start,
                    right.ray_end,
                    [0.2, 0.6, 1.0],
                    lineWidth=2,
                    lifeTime=0.10,
                )

            p.stepSimulation()
            if not args.no_realtime:
                time.sleep(config.simulation.time_step_s)

            if step % config.simulation.log_interval_steps == 0 or event:
                logger.log(
                    simulation_time_s=step * config.simulation.time_step_s,
                    state=safety.state.value,
                    target=mission.current.name if mission.current else "DONE",
                    pose=current_pose,
                    front_distance_m=filtered_front,
                    left_distance_m=left.distance_m,
                    right_distance_m=right.distance_m,
                    linear_command_mps=linear_command,
                    angular_command_rps=angular_command,
                    event=event,
                )

            if step % 120 == 0:
                target_name = mission.current.name if mission.current else "DONE"
                print(
                    f"\rTarget: {target_name} | X: {current_pose.x:.2f}"
                    f" Y: {current_pose.y:.2f} | Front: {filtered_front:.2f} m"
                    f" | Safety: {safety.state.value}",
                    end="",
                )

            if mission_success:
                return 0

        print("\nMISSION FAILED: timeout.")
        return 1

    except PathNotFoundError as error:
        print("\nPLANNER ERROR:", error)
        return 3
    finally:
        if logger is not None:
            logger.close()
        if p.isConnected():
            p.disconnect()


def main() -> None:
    raise SystemExit(run(parse_args()))


if __name__ == "__main__":
    main()

