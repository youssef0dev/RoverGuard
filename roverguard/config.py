from dataclasses import dataclass, field

from .geometry import AABB, Point2D


@dataclass(frozen=True)
class RobotConfig:
    wheel_radius_m: float = 0.05
    wheel_separation_m: float = 0.23
    body_length_m: float = 0.30
    body_width_m: float = 0.26
    motor_force_n: float = 2.4
    max_linear_speed_mps: float = 0.28
    max_angular_speed_rps: float = 1.8

    @property
    def footprint_radius_m(self) -> float:
        """Safe circular approximation of the rectangular robot body."""
        half_length = self.body_length_m / 2.0
        half_width = self.body_width_m / 2.0
        return (half_length**2 + half_width**2) ** 0.5


@dataclass(frozen=True)
class PlannerConfig:
    min_x_m: float = -0.40
    max_x_m: float = 1.60
    min_y_m: float = -0.70
    max_y_m: float = 0.95
    resolution_m: float = 0.05
    safety_margin_m: float = 0.05
    allow_diagonal: bool = True


@dataclass(frozen=True)
class SensorConfig:
    range_m: float = 1.0
    side_angle_deg: float = 45.0
    noise_std_m: float = 0.01
    filter_alpha: float = 0.20
    # The ray starts at the front edge of the chassis. These limits are
    # therefore measured from the sensor, not from the robot centre.
    slow_distance_m: float = 0.18
    stop_distance_m: float = 0.08
    clear_distance_m: float = 0.24


@dataclass(frozen=True)
class CameraConfig:
    width_px: int = 960
    height_px: int = 720
    field_of_view_deg: float = 72.0
    near_plane_m: float = 0.02
    far_plane_m: float = 12.0
    forward_offset_m: float = 0.04
    # Small optical offset from the dedicated camera_head URDF link. The
    # physical height itself belongs in the robot model, not in Python.
    vertical_offset_m: float = 0.02
    pitch_down_deg: float = 8.0
    target_distance_m: float = 3.0


@dataclass(frozen=True)
class SimulationConfig:
    time_step_s: float = 1.0 / 240.0
    max_seconds: float = 360.0
    inspection_seconds: float = 0.50
    localization_interval_steps: int = 24
    localization_gain: float = 0.25
    localization_position_std_m: float = 0.004
    localization_angle_std_deg: float = 0.5
    log_interval_steps: int = 12


@dataclass(frozen=True)
class MissionConfig:
    waypoints: tuple[tuple[str, Point2D], ...] = (
        ("P1", Point2D(1.15, 0.00)),
        ("P2", Point2D(1.15, 0.55)),
        ("P3", Point2D(0.00, 0.55)),
        ("BASE", Point2D(0.00, 0.00)),
    )
    goal_tolerance_m: float = 0.08


@dataclass(frozen=True)
class WorldConfig:
    obstacle_center: Point2D = Point2D(0.70, 0.00)
    obstacle_size_x_m: float = 0.18
    obstacle_size_y_m: float = 0.18

    @property
    def obstacle(self) -> AABB:
        half_x = self.obstacle_size_x_m / 2.0
        half_y = self.obstacle_size_y_m / 2.0
        return AABB(
            self.obstacle_center.x - half_x,
            self.obstacle_center.x + half_x,
            self.obstacle_center.y - half_y,
            self.obstacle_center.y + half_y,
        )


@dataclass(frozen=True)
class AppConfig:
    robot: RobotConfig = field(default_factory=RobotConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)
    world: WorldConfig = field(default_factory=WorldConfig)

