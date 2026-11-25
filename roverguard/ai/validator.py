"""Independent safety checks for AI-proposed missions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from ..geometry import Point2D
from .schema import MissionPlan


class MissionValidationError(ValueError):
    """Raised when an AI plan violates the deterministic safety policy."""


@dataclass(frozen=True)
class MissionValidator:
    allowed_waypoints: tuple[str, ...]
    max_robot_speed_mps: float
    min_robot_speed_mps: float = 0.05
    max_mission_waypoints: int = 12

    def validate(self, plan: MissionPlan) -> MissionPlan:
        name = plan.mission_name.strip()
        note = plan.inspection_note.strip()
        if not name:
            raise MissionValidationError("Mission name cannot be empty")
        if len(name) > 80:
            raise MissionValidationError("Mission name is too long")
        if len(note) > 240:
            raise MissionValidationError("Inspection note is too long")

        if not (
            self.min_robot_speed_mps
            <= plan.max_speed_mps
            <= self.max_robot_speed_mps
        ):
            raise MissionValidationError(
                "Unsafe speed: expected "
                f"{self.min_robot_speed_mps:.2f}.."
                f"{self.max_robot_speed_mps:.2f} m/s"
            )

        allowed = set(self.allowed_waypoints)
        normalized = tuple(item.strip().upper() for item in plan.waypoints)
        if not normalized:
            raise MissionValidationError("Mission needs at least one waypoint")
        if len(normalized) > self.max_mission_waypoints:
            raise MissionValidationError("Mission has too many waypoints")

        unknown = [item for item in normalized if item not in allowed]
        if unknown:
            raise MissionValidationError(
                f"Unknown waypoint(s): {', '.join(unknown)}"
            )

        without_base = tuple(item for item in normalized if item != "BASE")
        if len(set(without_base)) != len(without_base):
            raise MissionValidationError("Repeated inspection waypoint")

        if "BASE" in normalized and normalized[-1] != "BASE":
            raise MissionValidationError("BASE can only be the final waypoint")

        return_to_base = plan.return_to_base or normalized[-1] == "BASE"
        route = without_base + (("BASE",) if return_to_base else ())
        if not route:
            raise MissionValidationError("Mission route cannot be empty")

        photos = tuple(item.strip().upper() for item in plan.photo_waypoints)
        if len(set(photos)) != len(photos):
            raise MissionValidationError("Repeated photo waypoint")
        unknown_photos = [item for item in photos if item not in allowed]
        if unknown_photos:
            raise MissionValidationError(
                f"Unknown photo waypoint(s): {', '.join(unknown_photos)}"
            )
        outside_route = [item for item in photos if item not in route]
        if outside_route:
            raise MissionValidationError(
                "Photo waypoint must also be in mission route: "
                + ", ".join(outside_route)
            )

        return MissionPlan(
            mission_name=name,
            waypoints=route,
            photo_waypoints=photos,
            return_to_base=return_to_base,
            max_speed_mps=plan.max_speed_mps,
            inspection_note=note,
        )

    def resolve_waypoints(
        self,
        plan: MissionPlan,
        configured_waypoints: Mapping[str, Point2D],
    ) -> tuple[tuple[str, Point2D], ...]:
        """Translate validated names into deterministic map coordinates."""
        validated = self.validate(plan)
        try:
            return tuple(
                (name, configured_waypoints[name]) for name in validated.waypoints
            )
        except KeyError as error:
            raise MissionValidationError(
                f"Waypoint has no configured coordinate: {error.args[0]}"
            ) from error
