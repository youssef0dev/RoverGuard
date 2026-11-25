"""Strict, data-driven world maps for RoverGuard simulations."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .geometry import AABB, Point2D


class WorldMapError(ValueError):
    """Raised when a map file is incomplete or unsafe to load."""


@dataclass(frozen=True)
class MapBounds:
    min_x_m: float
    max_x_m: float
    min_y_m: float
    max_y_m: float


@dataclass(frozen=True)
class MapObstacle:
    name: str
    center: Point2D
    size_x_m: float
    size_y_m: float
    height_m: float
    color_rgba: tuple[float, float, float, float]

    @property
    def aabb(self) -> AABB:
        return AABB(
            self.center.x - self.size_x_m / 2.0,
            self.center.x + self.size_x_m / 2.0,
            self.center.y - self.size_y_m / 2.0,
            self.center.y + self.size_y_m / 2.0,
        )


@dataclass(frozen=True)
class WorldMap:
    name: str
    bounds: MapBounds
    robot_start: Point2D
    waypoints: tuple[tuple[str, Point2D], ...]
    default_route: tuple[str, ...]
    obstacles: tuple[MapObstacle, ...]

    @property
    def waypoint_lookup(self) -> dict[str, Point2D]:
        return dict(self.waypoints)

    def resolve_route(self, names: tuple[str, ...]) -> tuple[tuple[str, Point2D], ...]:
        lookup = self.waypoint_lookup
        try:
            return tuple((name, lookup[name]) for name in names)
        except KeyError as error:
            raise WorldMapError(f"Unknown route waypoint: {error.args[0]}") from error


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise WorldMapError(f"{label} must be a number")
    return float(value)


def _vector(value: Any, length: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise WorldMapError(f"{label} must contain {length} numbers")
    return tuple(_number(item, label) for item in value)


def load_world_map(path: str | Path) -> WorldMap:
    source = Path(path)
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except OSError as error:
        raise WorldMapError(f"Cannot read map: {source}") from error
    except json.JSONDecodeError as error:
        raise WorldMapError(f"Invalid map JSON: {error}") from error

    if not isinstance(payload, dict):
        raise WorldMapError("Map root must be a JSON object")
    if payload.get("schema_version") != 1:
        raise WorldMapError("Unsupported map schema_version")

    name = payload.get("name")
    if not isinstance(name, str) or not name.strip():
        raise WorldMapError("Map name is required")

    bounds_data = payload.get("bounds")
    if not isinstance(bounds_data, dict):
        raise WorldMapError("Map bounds are required")
    bounds = MapBounds(
        _number(bounds_data.get("min_x_m"), "bounds.min_x_m"),
        _number(bounds_data.get("max_x_m"), "bounds.max_x_m"),
        _number(bounds_data.get("min_y_m"), "bounds.min_y_m"),
        _number(bounds_data.get("max_y_m"), "bounds.max_y_m"),
    )
    if bounds.min_x_m >= bounds.max_x_m or bounds.min_y_m >= bounds.max_y_m:
        raise WorldMapError("Map bounds are reversed or empty")

    start_xy = _vector(payload.get("robot_start"), 2, "robot_start")
    robot_start = Point2D(*start_xy)
    if robot_start != Point2D(0.0, 0.0):
        raise WorldMapError("Current map format requires robot_start [0, 0]")

    waypoint_data = payload.get("waypoints")
    if not isinstance(waypoint_data, dict) or not waypoint_data:
        raise WorldMapError("Map needs at least one waypoint")
    waypoints: list[tuple[str, Point2D]] = []
    for raw_name, raw_point in waypoint_data.items():
        if not isinstance(raw_name, str) or raw_name != raw_name.strip().upper():
            raise WorldMapError("Waypoint names must be uppercase without spaces")
        xy = _vector(raw_point, 2, f"waypoint {raw_name}")
        point = Point2D(*xy)
        if not (
            bounds.min_x_m <= point.x <= bounds.max_x_m
            and bounds.min_y_m <= point.y <= bounds.max_y_m
        ):
            raise WorldMapError(f"Waypoint outside map: {raw_name}")
        waypoints.append((raw_name, point))
    lookup = dict(waypoints)
    if "BASE" not in lookup:
        raise WorldMapError("Map must define BASE")
    if lookup["BASE"] != robot_start:
        raise WorldMapError("BASE must match robot_start")

    route_data = payload.get("default_route")
    if not isinstance(route_data, list) or not route_data:
        raise WorldMapError("default_route is required")
    default_route = tuple(route_data)
    if not all(isinstance(item, str) and item in lookup for item in default_route):
        raise WorldMapError("default_route contains an unknown waypoint")

    obstacle_data = payload.get("obstacles")
    if not isinstance(obstacle_data, list):
        raise WorldMapError("obstacles must be a list")
    obstacles: list[MapObstacle] = []
    names: set[str] = set()
    for index, raw in enumerate(obstacle_data):
        if not isinstance(raw, dict):
            raise WorldMapError(f"Obstacle {index} must be an object")
        obstacle_name = raw.get("name")
        if not isinstance(obstacle_name, str) or not obstacle_name.strip():
            raise WorldMapError(f"Obstacle {index} needs a name")
        if obstacle_name in names:
            raise WorldMapError(f"Repeated obstacle name: {obstacle_name}")
        names.add(obstacle_name)
        center = _vector(raw.get("center"), 2, f"{obstacle_name}.center")
        size = _vector(raw.get("size"), 3, f"{obstacle_name}.size")
        color = _vector(raw.get("color", [0.8, 0.3, 0.1, 1.0]), 4, f"{obstacle_name}.color")
        if any(value <= 0 for value in size):
            raise WorldMapError(f"Obstacle size must be positive: {obstacle_name}")
        if any(value < 0.0 or value > 1.0 for value in color):
            raise WorldMapError(f"Obstacle color must be between 0 and 1: {obstacle_name}")
        obstacles.append(
            MapObstacle(
                obstacle_name,
                Point2D(*center),
                size[0],
                size[1],
                size[2],
                color,  # type: ignore[arg-type]
            )
        )

    return WorldMap(
        name=name.strip(),
        bounds=bounds,
        robot_start=robot_start,
        waypoints=tuple(waypoints),
        default_route=default_route,
        obstacles=tuple(obstacles),
    )
