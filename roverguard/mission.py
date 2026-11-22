from dataclasses import dataclass

from .geometry import Point2D


@dataclass(frozen=True)
class MissionTarget:
    name: str
    point: Point2D


class PatrolMission:
    def __init__(self, waypoints: tuple[tuple[str, Point2D], ...]):
        if not waypoints:
            raise ValueError("Mission needs at least one waypoint")
        self.targets = tuple(MissionTarget(name, point) for name, point in waypoints)
        self._index = 0

    @property
    def complete(self) -> bool:
        return self._index >= len(self.targets)

    @property
    def current(self) -> MissionTarget | None:
        if self.complete:
            return None
        return self.targets[self._index]

    def mark_current_reached(self) -> MissionTarget:
        if self.complete:
            raise RuntimeError("Mission already complete")
        reached = self.targets[self._index]
        self._index += 1
        return reached

