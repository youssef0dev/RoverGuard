import math

from .geometry import Pose2D, normalize_angle


class DifferentialDriveOdometry:
    def __init__(self, wheel_radius_m: float, wheel_separation_m: float):
        self.wheel_radius_m = wheel_radius_m
        self.wheel_separation_m = wheel_separation_m
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self._previous_left_angle: float | None = None
        self._previous_right_angle: float | None = None

    def reset(
        self,
        left_angle: float,
        right_angle: float,
        pose: Pose2D = Pose2D(0.0, 0.0, 0.0),
    ) -> None:
        self.pose = pose
        self._previous_left_angle = left_angle
        self._previous_right_angle = right_angle

    def update(self, left_angle: float, right_angle: float) -> Pose2D:
        if self._previous_left_angle is None or self._previous_right_angle is None:
            self.reset(left_angle, right_angle)
            return self.pose

        delta_left = (
            left_angle - self._previous_left_angle
        ) * self.wheel_radius_m
        delta_right = (
            right_angle - self._previous_right_angle
        ) * self.wheel_radius_m

        delta_distance = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_separation_m
        middle_theta = self.pose.theta + delta_theta / 2.0

        self.pose = Pose2D(
            self.pose.x + delta_distance * math.cos(middle_theta),
            self.pose.y + delta_distance * math.sin(middle_theta),
            normalize_angle(self.pose.theta + delta_theta),
        )
        self._previous_left_angle = left_angle
        self._previous_right_angle = right_angle
        return self.pose

    def correct(self, measured: Pose2D, gain: float) -> Pose2D:
        theta_error = normalize_angle(measured.theta - self.pose.theta)
        self.pose = Pose2D(
            self.pose.x + gain * (measured.x - self.pose.x),
            self.pose.y + gain * (measured.y - self.pose.y),
            normalize_angle(self.pose.theta + gain * theta_error),
        )
        return self.pose

