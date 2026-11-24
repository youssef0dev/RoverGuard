"""Strict data contract between an LLM and the deterministic robot stack."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from typing import Any, Mapping, Sequence


PLAN_FIELDS = {
    "mission_name",
    "waypoints",
    "return_to_base",
    "max_speed_mps",
    "inspection_note",
}


def mission_json_schema(
    allowed_waypoints: Sequence[str],
    max_speed_mps: float = 0.22,
    max_waypoints: int = 12,
) -> dict[str, Any]:
    """Build the JSON schema sent to Ollama structured outputs."""
    names = list(allowed_waypoints)
    if not names:
        raise ValueError("At least one waypoint is required")
    return {
        "type": "object",
        "properties": {
            "mission_name": {
                "type": "string",
                "minLength": 1,
                "maxLength": 80,
            },
            "waypoints": {
                "type": "array",
                "items": {"type": "string", "enum": names},
                "minItems": 1,
                "maxItems": max_waypoints,
            },
            "return_to_base": {"type": "boolean"},
            "max_speed_mps": {
                "type": "number",
                "minimum": 0.05,
                "maximum": max_speed_mps,
            },
            "inspection_note": {
                "type": "string",
                "maxLength": 240,
            },
        },
        "required": sorted(PLAN_FIELDS),
        "additionalProperties": False,
    }


@dataclass(frozen=True)
class MissionPlan:
    mission_name: str
    waypoints: tuple[str, ...]
    return_to_base: bool
    max_speed_mps: float
    inspection_note: str

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "MissionPlan":
        unknown = set(payload) - PLAN_FIELDS
        missing = PLAN_FIELDS - set(payload)
        if unknown:
            raise ValueError(f"Unknown mission fields: {sorted(unknown)}")
        if missing:
            raise ValueError(f"Missing mission fields: {sorted(missing)}")

        mission_name = payload["mission_name"]
        waypoints = payload["waypoints"]
        return_to_base = payload["return_to_base"]
        max_speed = payload["max_speed_mps"]
        inspection_note = payload["inspection_note"]

        if not isinstance(mission_name, str):
            raise ValueError("mission_name must be a string")
        if not isinstance(waypoints, list) or not all(
            isinstance(item, str) for item in waypoints
        ):
            raise ValueError("waypoints must be a list of strings")
        if not isinstance(return_to_base, bool):
            raise ValueError("return_to_base must be a boolean")
        if isinstance(max_speed, bool) or not isinstance(max_speed, (int, float)):
            raise ValueError("max_speed_mps must be a number")
        if not isinstance(inspection_note, str):
            raise ValueError("inspection_note must be a string")

        return cls(
            mission_name=mission_name,
            waypoints=tuple(waypoints),
            return_to_base=return_to_base,
            max_speed_mps=float(max_speed),
            inspection_note=inspection_note,
        )

    @classmethod
    def from_json(cls, text: str) -> "MissionPlan":
        payload = json.loads(text)
        if not isinstance(payload, dict):
            raise ValueError("Mission output must be one JSON object")
        return cls.from_mapping(payload)

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["waypoints"] = list(self.waypoints)
        return data

    def to_json(self, *, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)
