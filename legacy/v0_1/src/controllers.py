import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GoToGoalController:
    def __init__(
        self,
        wheel_radius,
        wheel_separation,
        max_linear_speed=0.25,
        max_angular_speed=2.0,
        distance_tolerance=0.03,
        linear_gain=0.9,
        angular_gain=2.5,
    ):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.distance_tolerance = distance_tolerance
        self.linear_gain = linear_gain
        self.angular_gain = angular_gain

    def compute(self, x, y, theta, goal_x, goal_y):
        delta_x = goal_x - x
        delta_y = goal_y - y

        distance_error = math.hypot(delta_x, delta_y)
        target_angle = math.atan2(delta_y, delta_x)
        angle_error = normalize_angle(target_angle - theta)

        reached = distance_error <= self.distance_tolerance

        if reached:
            return 0.0, 0.0, distance_error, angle_error, True

        linear_speed = min(
            self.max_linear_speed,
            self.linear_gain * distance_error,
        )

        angular_speed = self.angular_gain * angle_error
        angular_speed = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular_speed),
        )

        if abs(angle_error) > math.radians(50):
            linear_speed = 0.0
        else:
            linear_speed *= max(0.0, math.cos(angle_error))

        left_linear_speed = (
            linear_speed
            - angular_speed * self.wheel_separation / 2.0
        )

        right_linear_speed = (
            linear_speed
            + angular_speed * self.wheel_separation / 2.0
        )

        left_wheel_speed = left_linear_speed / self.wheel_radius
        right_wheel_speed = right_linear_speed / self.wheel_radius

        return (
            left_wheel_speed,
            right_wheel_speed,
            distance_error,
            angle_error,
            False,
        )
