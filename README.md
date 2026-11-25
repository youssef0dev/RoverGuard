# RoverGuard v0.5

Action-aware, multi-point local-LLM mission planning for a wheeled
differential-drive inspection robot: PyBullet simulation, A* navigation,
independent safety, odometry, telemetry, tests, and a hardware-neutral C++
firmware core.

## Chno kaydir daba?

1. Kaybni 2D occupancy grid.
2. Kaykebber obstacle f map 7sab 7ajm robot + safety margin.
3. A* kay7seb a9sar safe path.
4. Path follower kaytbe3 points dyal path.
5. Safety supervisor y9der yn9es speed wla yw9ef robot.
6. JSON map kat7edded 11 points w several static obstacles.
7. Operator y9der ykhtar route b Darija, English, wla French.
8. Telemetry kattsjel f `logs/`.
9. Local LLM kay7awwel l-command l strict mission JSON.
10. Deterministic validator kayrefed waypoint wla speed ma-amnach.
11. Operator y9der yطلب photo f points mo3ayanin; PNG kattsjel f `captures/`.

## Cognitive local-LLM demo

Requirements:

- Ollama running locally;
- installed model `qwen3.5:4b-q4_K_M`.

Plan and validate without opening PyBullet:

```powershell
python run_cognitive_demo.py `
  --command "Inspect P1 and P3, then return to base" `
  --plan-only
```

Run the approved mission in PyBullet:

```powershell
python run_cognitive_demo.py `
  --command "Inspect P1 and P3, then return to base"
```

Interactive mode:

```powershell
python run_cognitive_demo.py
```

The terminal lists `BASE` and `P1` through `P10`. Example:

```text
Sir P2, P5, P10, w rje3 l BASE b speed 0.15 m/s
```

Photo mission example:

```text
Go to P3 and take a photo, then go to P6, then go to P8 and take a photo, then return to BASE.
```

The approved JSON separates the route from the requested camera actions:

```json
{
  "waypoints": ["P3", "P6", "P8", "BASE"],
  "photo_waypoints": ["P3", "P8"]
}
```

When the robot is stable at each requested point, its simulated front camera
captures a 640x480 RGB PNG under `captures/`. Runtime captures are ignored by
Git by default.

The 3D window is the physics view; mission text stays in the external terminal
so a future voice adapter can feed the same validated command pipeline.

## Data-driven world

The default map is:

```text
world/maps/warehouse_v1.json
```

It stores map bounds, waypoint coordinates, obstacle sizes/colors, robot start,
and the default route. The simulator loads every obstacle into PyBullet and
inflates it in the A* occupancy grid using robot size plus safety margin.

The LLM never controls wheels or motors. It can only propose configured
waypoint names and a bounded speed. Read
[docs/AI_ARCHITECTURE.md](docs/AI_ARCHITECTURE.md).

## Installation (Windows PowerShell)

Python 3.11-3.13 64-bit w virtual environment li deja 3ndk:

```powershell
cd C:\Users\hp\robot-lab\roverguard
& C:\Users\hp\robot-lab\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

## Run simulation

Mn root dyal project:

```powershell
python run_roverguard.py
```

Alternative:

```powershell
python -m roverguard.simulation
```

Test bla GUI w b sor3a:

```powershell
python -m roverguard.simulation --headless --no-realtime
```

## Run pure software tests

Had tests ma-kay7tajoch PyBullet:

```powershell
python -m unittest discover -s tests -v
```

## Expected mission output

```text
PLAN P2: ...
P2 WSEL
NEXT TARGET: P5
P5 WSEL
NEXT TARGET: P10
P10 WSEL
NEXT TARGET: BASE
BASE WSEL
MISSION SUCCESS: P2 -> P5 -> P10 -> BASE
```

## Project structure

```text
roverguard/       Python core: math, planning, safety, simulation, AI boundary
robot/            URDF dyal robot
world/            JSON maps and legacy URDF obstacles
captures/         Runtime RGB photos requested by approved missions
tests/            automatic unit tests
examples/         reproducible natural-language mission prompts
firmware/         hardware-neutral C++ controller/HAL
docs/             architecture w Sim-to-Real explanations
legacy/v0_1/      learning scripts l9dam (reference fa9at)
```

## Important

`legacy/v0_1` kayn ghir bach tchof progress dyalk. Main version hiya:

```powershell
python run_roverguard.py
```

Qra [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) w
[docs/SIM_TO_REAL.md](docs/SIM_TO_REAL.md) 9bel hardware deployment.

