import math
import random
from dataclasses import dataclass


@dataclass(frozen=True)
class RangeReading:
    distance_m: float
    ray_start: tuple[float, float, float]
    ray_end: tuple[float, float, float]


class LowPassFilter:
    def __init__(self, alpha: float):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be inside (0, 1]")
        self.alpha = alpha
        self.value: float | None = None

    def update(self, measurement: float) -> float:
        if self.value is None:
            self.value = measurement
        else:
            self.value = self.alpha * measurement + (1.0 - self.alpha) * self.value
        return self.value


def rotate_xy(direction: tuple[float, float, float], angle_rad: float):
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return (
        cosine * direction[0] - sine * direction[1],
        sine * direction[0] + cosine * direction[1],
        direction[2],
    )


def noisy_clamped_distance(
    true_distance_m: float,
    max_distance_m: float,
    noise_std_m: float,
    rng: random.Random,
) -> float:
    return max(
        0.0,
        min(max_distance_m, true_distance_m + rng.gauss(0.0, noise_std_m)),
    )

