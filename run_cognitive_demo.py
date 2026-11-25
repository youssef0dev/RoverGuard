"""Natural-language RoverGuard demo powered by a local Ollama model."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from roverguard.ai import (
    CognitiveMissionPlanner,
    MissionValidationError,
    MissionValidator,
    OllamaProvider,
    OllamaProviderError,
)
from roverguard.cognitive_mission import save_mission_audit
from roverguard.config import AppConfig
from roverguard.simulation import DEFAULT_WORLD_MAP, run as run_simulation
from roverguard.world_map import load_world_map


PROJECT_ROOT = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RoverGuard v0.5 action-aware cognitive mission demo"
    )
    parser.add_argument("--command", help="Natural-language mission command")
    parser.add_argument(
        "--model",
        default="qwen3.5:4b-q4_K_M",
        help="Installed Ollama model name",
    )
    parser.add_argument(
        "--ollama-host",
        default="http://127.0.0.1:11434",
    )
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="Validate and print the plan without starting PyBullet.",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-realtime", action="store_true")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-seconds", type=float, default=None)
    parser.add_argument(
        "--map",
        default=str(DEFAULT_WORLD_MAP),
        help="Path to the JSON world map.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = AppConfig()
    world_map = load_world_map(args.map)
    configured = world_map.waypoint_lookup

    command = args.command
    if command is None:
        available = ", ".join(configured)
        print("\nROVERGUARD v0.5 MISSION CONSOLE")
        print("Map:", world_map.name)
        print("Points:", available)
        print(
            "Example: Go to P3 and take a photo, then P6, then P8 "
            "and take a photo, then return to BASE"
        )
        command = input("Kteb mission: ").strip()

    validator = MissionValidator(
        allowed_waypoints=tuple(configured),
        max_robot_speed_mps=config.robot.max_linear_speed_mps,
        max_mission_waypoints=min(12, len(configured)),
    )
    provider = OllamaProvider(
        model=args.model,
        base_url=args.ollama_host,
    )
    planner = CognitiveMissionPlanner(provider, validator)

    try:
        plan = planner.plan(command)
        resolved_waypoints = validator.resolve_waypoints(plan, configured)
    except (ValueError, MissionValidationError, OllamaProviderError) as error:
        print("\nMISSION REJECTED:", error)
        raise SystemExit(2) from error

    audit_path = save_mission_audit(
        PROJECT_ROOT / "logs",
        user_command=command,
        model=args.model,
        plan=plan,
    )
    print("\nMISSION APPROVED BY SAFETY VALIDATOR")
    print(json.dumps(plan.to_dict(), indent=2, ensure_ascii=False))
    print("Audit:", audit_path)

    if args.plan_only:
        return

    simulation_args = argparse.Namespace(
        headless=args.headless,
        no_realtime=args.no_realtime,
        seed=args.seed,
        max_seconds=args.max_seconds,
        map=args.map,
    )
    result = run_simulation(
        simulation_args,
        mission_waypoints=resolved_waypoints,
        max_linear_speed_mps=plan.max_speed_mps,
        mission_label=plan.mission_name,
        world_map=world_map,
        photo_waypoints=plan.photo_waypoints,
    )
    raise SystemExit(result)


if __name__ == "__main__":
    main()
