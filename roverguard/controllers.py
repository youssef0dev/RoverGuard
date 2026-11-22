import math
from dataclasses import dataclass

from .geometry import Point2D, Pose2D, normalize_angle


@dataclass(frozen=True)
class VelocityCommand:
    linear_m_s: float
    angular_rad_s: float
    target_distance_m: float
    target_angle_error_rad: float
    reached: bool


class PointController:
    def __init__(
        self,
        max_linear_m_s: float,
        max_angular_rad_s: float,
        distance_tolerance_m: float,
        linear_gain: float = 0.9,
        angular_gain: float = 2.5,
    ):
        self.max_linear_m_s = max_linear_m_s
        self.max_angular_rad_s = max_angular_rad_s
        self.distance_tolerance_m = distance_tolerance_m
        self.linear_gain = linear_gain
        self.angular_gain = angular_gain

    def compute(
        self,
        pose: Pose2D,
        target: Point2D,
        distance_tolerance_m: float | None = None,
    ) -> VelocityCommand:
        delta_x = target.x - pose.x
        delta_y = target.y - pose.y
        distance = math.hypot(delta_x, delta_y)
        target_angle = math.atan2(delta_y, delta_x)
        angle_error = normalize_angle(target_angle - pose.theta)
        tolerance = (
            self.distance_tolerance_m
            if distance_tolerance_m is None
            else distance_tolerance_m
        )
        reached = distance <= tolerance

        if reached:
            return VelocityCommand(0.0, 0.0, distance, angle_error, True)

        linear = min(self.max_linear_m_s, self.linear_gain * distance)
        angular = max(
            -self.max_angular_rad_s,
            min(self.max_angular_rad_s, self.angular_gain * angle_error),
        )

        if abs(angle_error) > math.radians(50):
            linear = 0.0
        else:
            linear *= max(0.0, math.cos(angle_error))

        return VelocityCommand(linear, angular, distance, angle_error, False)

