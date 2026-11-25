from .controllers import PointController, VelocityCommand
from .geometry import Point2D, Pose2D


class PathFollower:
    """Follow an ordered list of safe A* path points."""

    def __init__(
        self,
        controller: PointController,
        intermediate_tolerance_m: float = 0.04,
    ):
        self.controller = controller
        self.intermediate_tolerance_m = intermediate_tolerance_m
        self._points: tuple[Point2D, ...] = ()
        self._index = 0

    @property
    def finished(self) -> bool:
        return bool(self._points) and self._index >= len(self._points)

    @property
    def current_target(self) -> Point2D | None:
        if not self._points or self.finished:
            return None
        return self._points[self._index]

    def set_path(self, points: tuple[Point2D, ...]) -> None:
        if not points:
            raise ValueError("Path cannot be empty")
        self._points = points
        self._index = 1 if len(points) > 1 else 0

    def compute(self, pose: Pose2D) -> VelocityCommand:
        if not self._points or self.finished:
            return VelocityCommand(0.0, 0.0, 0.0, 0.0, True)

        target = self._points[self._index]
        is_final = self._index == len(self._points) - 1
        tolerance = (
            self.controller.distance_tolerance_m
            if is_final
            else self.intermediate_tolerance_m
        )

        if pose.point.distance_to(target) <= tolerance:
            self._index += 1
            if self.finished:
                return VelocityCommand(0.0, 0.0, 0.0, 0.0, True)
            target = self._points[self._index]

        return self.controller.compute(
            pose,
            target,
            distance_tolerance_m=tolerance,
        )

