# RoverGuard v0.3

Safe local-LLM mission planning for a wheeled differential-drive inspection
robot: PyBullet simulation, A* navigation, independent safety, odometry,
telemetry, tests, and a hardware-neutral C++ firmware core.

## Chno kaydir daba?

1. Kaybni 2D occupancy grid.
2. Kaykebber obstacle f map 7sab 7ajm robot + safety margin.
3. A* kay7seb a9sar safe path.
4. Path follower kaytbe3 points dyal path.
5. Safety supervisor y9der yn9es speed wla yw9ef robot.
6. Mission katmchi `P1 -> P2 -> P3 -> BASE`.
7. Telemetry kattsjel f `logs/`.
8. Local LLM kay7awwel Darija/English/French l strict mission JSON.
9. Deterministic validator kayrefed waypoint wla speed ma-amnach.

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
PLAN P1: ...
P1 WSEL
NEXT TARGET: P2
P2 WSEL
NEXT TARGET: P3
P3 WSEL
NEXT TARGET: BASE
BASE WSEL
MISSION SUCCESS: P1 -> P2 -> P3 -> BASE
```

## Project structure

```text
roverguard/       Python core: math, planning, safety, simulation, AI boundary
robot/            URDF dyal robot
world/            URDF dyal obstacles
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

