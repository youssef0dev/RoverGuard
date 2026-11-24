# Changelog

## 0.3.0

- Added a local Ollama natural-language mission planner.
- Added JSON Schema structured output for AI missions.
- Added an independent deterministic mission validator.
- AI can only select configured waypoints and a bounded speed.
- Added dynamic waypoint routes to the existing PyBullet simulation.
- Added traceable JSON audit logs for approved AI missions.
- Added nine pure unit tests for schema, provider, and AI safety boundaries.
- Added `run_cognitive_demo.py` and AI architecture documentation.

## 0.2.1

- Fixed a path-follower deadlock at intermediate A* corners.
- Intermediate points now use their own 2.5 cm tolerance instead of the
  mission goal tolerance of 8 cm.
- Added a regression test reproducing the exact stop near `(0.29, -0.31)`.

## 0.2.0

- Replaced fixed-distance reactive detour with an inflated 2D map and A*.
- Added safe path simplification and path following.
- Centralized robot, planner, sensor, mission, and simulation configuration.
- Added independent safety states with hysteresis.
- Added deterministic sensor noise and CSV telemetry.
- Added 9 pure-Python unit tests.
- Added hardware-neutral C++17 HAL, wheel PID, command timeout, and E-stop.
- Added GitHub Actions checks for Python and C++.
- Preserved the v0.1 learning scripts under `legacy/v0_1`.

