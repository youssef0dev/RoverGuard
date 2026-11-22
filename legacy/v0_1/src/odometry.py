import math


class DifferentialDriveOdometry:
    def __init__(self, wheel_radius, wheel_separation):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.previous_left_angle = None
        self.previous_right_angle = None

    def reset(self, left_angle, right_angle):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.previous_left_angle = left_angle
        self.previous_right_angle = right_angle

    def update(self, left_angle, right_angle):
        if self.previous_left_angle is None:
            self.reset(left_angle, right_angle)
            return self.x, self.y, self.theta

        delta_left_angle = left_angle - self.previous_left_angle
        delta_right_angle = right_angle - self.previous_right_angle

        delta_left_distance = delta_left_angle * self.wheel_radius
        delta_right_distance = delta_right_angle * self.wheel_radius

        delta_distance = (
            delta_left_distance + delta_right_distance
        ) / 2.0

        delta_theta = (
            delta_right_distance - delta_left_distance
        ) / self.wheel_separation

        middle_theta = self.theta + delta_theta / 2.0

        self.x += delta_distance * math.cos(middle_theta)
        self.y += delta_distance * math.sin(middle_theta)
        self.theta += delta_theta

        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta),
        )

        self.previous_left_angle = left_angle
        self.previous_right_angle = right_angle

        return self.x, self.y, self.theta

    def correct_pose(self, measured_x, measured_y, measured_theta, gain):
        self.x += gain * (measured_x - self.x)
        self.y += gain * (measured_y - self.y)

        theta_error = math.atan2(
            math.sin(measured_theta - self.theta),
            math.cos(measured_theta - self.theta),
        )

        self.theta += gain * theta_error
        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta),
        )

        return self.x, self.y, self.theta
