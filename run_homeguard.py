"""Deterministic semantic-room navigation demo for RoverGuard HomeGuard."""

from __future__ import annotations

import argparse

from roverguard.simulation import HOME_WORLD_MAP, run as run_simulation
from roverguard.world_map import WorldMapError, load_world_map


def build_room_route(world_map, requested_rooms, *, return_to_base: bool = True):
    """Resolve aliases into one deterministic, duplicate-free mission route."""
    resolved = []
    seen: set[str] = set()
    for requested in requested_rooms:
        room = world_map.resolve_room(requested)
        if room.name in seen:
            raise WorldMapError(f"Repeated room request: {room.name}")
        seen.add(room.name)
        resolved.append((room.name, room.inspection_point))
    if not resolved:
        raise WorldMapError("Mission needs at least one room")
    if return_to_base:
        resolved.append(("BASE", world_map.waypoint_lookup["BASE"]))
    return tuple(resolved)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RoverGuard v0.6 semantic home navigation"
    )
    parser.add_argument(
        "--room",
        action="append",
        help=(
            "Room name or configured alias. Repeat --room to visit multiple rooms."
        ),
    )
    parser.add_argument(
        "--photo",
        action="store_true",
        help="Capture one front-camera image after reaching every requested room.",
    )
    parser.add_argument(
        "--no-return",
        action="store_true",
        help="Finish in the last requested room instead of returning to BASE.",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-realtime", action="store_true")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-seconds", type=float, default=None)
    parser.add_argument(
        "--map",
        default=str(HOME_WORLD_MAP),
        help="Path to a semantic RoverGuard home-map JSON file.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    world_map = load_world_map(args.map)
    if not world_map.rooms:
        raise SystemExit("Selected map has no semantic rooms.")

    requested_rooms = args.room
    if not requested_rooms:
        available = ", ".join(room.name for room in world_map.rooms)
        print("\nROVERGUARD v0.6 HOME CONSOLE")
        print("Map:", world_map.name)
        print("Rooms:", available)
        print("Aliases are accepted: cuisine, chambre, salon, couloir...")
        requested_rooms = [input("Room: ").strip()]

    try:
        resolved = build_room_route(
            world_map,
            requested_rooms,
            return_to_base=not args.no_return,
        )
    except WorldMapError as error:
        raise SystemExit(f"MISSION REJECTED: {error}") from error

    for requested, (canonical, _) in zip(requested_rooms, resolved):
        print(f"Resolved '{requested}' -> {canonical}")

    photo_waypoints = tuple(name for name, _ in resolved if name != "BASE")
    if not args.photo:
        photo_waypoints = ()

    simulation_args = argparse.Namespace(
        headless=args.headless,
        no_realtime=args.no_realtime,
        seed=args.seed,
        max_seconds=args.max_seconds,
        map=args.map,
    )
    result = run_simulation(
        simulation_args,
        mission_waypoints=resolved,
        mission_label="HomeGuard semantic room inspection",
        world_map=world_map,
        photo_waypoints=photo_waypoints,
    )
    raise SystemExit(result)


if __name__ == "__main__":
    main()
