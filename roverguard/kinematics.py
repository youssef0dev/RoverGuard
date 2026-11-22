from dataclasses import dataclass


@dataclass(frozen=True)
class WheelSpeeds:
    left_rad_s: float
    right_rad_s: float


@dataclass(frozen=True)
class BodyTwist:
    linear_m_s: float
    angular_rad_s: float


def twist_to_wheel_speeds(
    linear_m_s: float,
    angular_rad_s: float,
    wheel_radius_m: float,
    wheel_separation_m: float,
) -> WheelSpeeds:
    left_linear = linear_m_s - angular_rad_s * wheel_separation_m / 2.0
    right_linear = linear_m_s + angular_rad_s * wheel_separation_m / 2.0
    return WheelSpeeds(
        left_rad_s=left_linear / wheel_radius_m,
        right_rad_s=right_linear / wheel_radius_m,
    )


def wheel_speeds_to_twist(
    left_rad_s: float,
    right_rad_s: float,
    wheel_radius_m: float,
    wheel_separation_m: float,
) -> BodyTwist:
    left_linear = left_rad_s * wheel_radius_m
    right_linear = right_rad_s * wheel_radius_m
    return BodyTwist(
        linear_m_s=(left_linear + right_linear) / 2.0,
        angular_rad_s=(right_linear - left_linear) / wheel_separation_m,
    )

