"""Natural-language mission planner with a deterministic validation gate."""

from __future__ import annotations

from typing import Any, Mapping, Protocol

from .schema import MissionPlan, mission_json_schema
from .validator import MissionValidator


class MissionProvider(Protocol):
    def generate_mission(
        self,
        *,
        system_prompt: str,
        user_command: str,
        json_schema: Mapping[str, Any],
    ) -> Mapping[str, Any]: ...


class CognitiveMissionPlanner:
    def __init__(
        self,
        provider: MissionProvider,
        validator: MissionValidator,
    ) -> None:
        self.provider = provider
        self.validator = validator

    def plan(self, user_command: str) -> MissionPlan:
        command = user_command.strip()
        if not command:
            raise ValueError("Mission command cannot be empty")

        allowed = self.validator.allowed_waypoints
        schema = mission_json_schema(
            allowed,
            max_speed_mps=self.validator.max_robot_speed_mps,
            max_waypoints=self.validator.max_mission_waypoints,
        )
        system_prompt = (
            "You are the mission-planning layer of RoverGuard, an autonomous "
            "safety inspection robot. Understand Darija Latin, English, and "
            "French. Convert the operator request into exactly one JSON mission. "
            "Never invent waypoint names. Available waypoints: "
            f"{', '.join(allowed)}. "
            "The robot's deterministic planner handles paths and obstacles. "
            "Choose a max speed between 0.05 and "
            f"{self.validator.max_robot_speed_mps:.2f} m/s. "
            "Set return_to_base true when the request says return, come back, "
            "rje3, or mentions BASE. Do not include explanations outside JSON."
            " Put a waypoint in photo_waypoints only when the operator asks to "
            "take a photo there. Use an empty photo_waypoints list when no photo "
            "is requested. Every photo waypoint must also exist in waypoints."
        )
        payload = self.provider.generate_mission(
            system_prompt=system_prompt,
            user_command=command,
            json_schema=schema,
        )
        plan = MissionPlan.from_mapping(payload)
        return self.validator.validate(plan)
