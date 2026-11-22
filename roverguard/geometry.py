import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float

    def distance_to(self, other: "Point2D") -> float:
        return math.hypot(other.x - self.x, other.y - self.y)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float

    @property
    def point(self) -> Point2D:
        return Point2D(self.x, self.y)


@dataclass(frozen=True)
class AABB:
    """Axis-aligned rectangle: min/max X and min/max Y."""

    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def inflated(self, radius: float) -> "AABB":
        return AABB(
            self.min_x - radius,
            self.max_x + radius,
            self.min_y - radius,
            self.max_y + radius,
        )

    def contains(self, point: Point2D) -> bool:
        return (
            self.min_x <= point.x <= self.max_x
            and self.min_y <= point.y <= self.max_y
        )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

