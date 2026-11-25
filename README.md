# RoverGuard v0.6

RoverGuard is an autonomous differential-drive inspection robot built as a
simulation-to-real robotics project. It combines PyBullet physics, A* path
planning, odometry, independent collision safety, local-LLM mission planning,
camera actions, telemetry, automated tests, and a hardware-neutral C++ firmware
core.

Version 0.6 introduces **HomeGuard**: a semantic indoor map where the robot
understands named rooms such as `KITCHEN`, `SALON`, `BEDROOM`, and `BATHROOM`
instead of navigating only to anonymous point numbers.

## Highlights

- Wheeled differential-drive robot described in URDF.
- Data-driven PyBullet warehouse and house environments.
- Inflated 2D occupancy grid that accounts for robot footprint and safety margin.
- A* global path planner with path simplification and closed-loop path following.
- Noisy range sensors, filtering, odometry, and localization correction.
- Independent stop/slow/clear safety supervisor with hysteresis.
- Local Ollama mission planner with strict JSON Schema output.
- Deterministic validator that prevents an LLM from controlling motors directly.
- Robot-mounted simulated RGB camera and waypoint-triggered photo capture.
- Semantic room aliases in English, French, and Darija Latin.
- CSV telemetry, JSON mission audits, Python tests, and a C++17 firmware/HAL core.

## HomeGuard semantic navigation

The semantic house map is stored at:

```text
world/maps/home_v1.json
```

It defines the physical walls and furniture, room boundaries, safe inspection
points, camera headings, and multilingual room aliases.

Run a deterministic room mission without an LLM:

```powershell
python run_homeguard.py --room cuisine --photo
```

Visit several rooms and return to base:

```powershell
python run_homeguard.py `
  --room salon `
  --room kitchen `
  --room bedroom `
  --photo
```

The command resolves aliases to canonical destinations, for example:

```text
cuisine -> KITCHEN
chambre -> BEDROOM
living room -> SALON
couloir -> CORRIDOR
```

## Local-LLM mission demo

Requirements:

- Ollama running locally;
- installed model `qwen3.5:4b-q4_K_M`.

Plan and validate a home mission without opening PyBullet:

```powershell
python run_cognitive_demo.py `
  --map world/maps/home_v1.json `
  --command "Go to the kitchen and take a photo, inspect the bedroom, then return to base" `
  --plan-only
```

Run the approved mission:

```powershell
python run_cognitive_demo.py `
  --map world/maps/home_v1.json `
  --command "Go to the kitchen and take a photo, inspect the bedroom, then return to base"
```

The LLM can only propose configured destinations, camera actions, and a bounded
speed. A deterministic validator rejects invented rooms, unsafe speeds, repeated
destinations, and photo requests outside the approved route.

## Warehouse mission demo

The v0.5 multi-point warehouse remains available:

```powershell
python run_cognitive_demo.py `
  --command "Go to P3 and take a photo, then P6, then P8 and take a photo, then return to BASE"
```

Runtime photos are stored under `captures/` and ignored by Git. Telemetry is
stored under `logs/`.

## Installation on Windows

From the repository root, activate the existing virtual environment and install
the dependencies:

```powershell
cd C:\Users\hp\robot-lab\roverguard
& C:\Users\hp\robot-lab\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

Supported Python versions: 3.11 through 3.13, 64-bit.

## Tests

The pure software test suite does not require the PyBullet GUI:

```powershell
python -m unittest discover -s tests -v
```

The suite verifies kinematics, odometry, safety, A*, path following, camera
encoding, AI mission validation, semantic room parsing, alias resolution, and
reachability between every room.

## Architecture

```text
Operator command
      |
      v
Local LLM mission planner
      |
      v
Strict JSON mission + deterministic validator
      |
      v
Semantic map -> occupancy grid -> A* path
      |
      v
Path follower -> safety supervisor -> wheel commands
      |
      v
PyBullet robot -> telemetry and camera captures
```

The LLM is never allowed to set wheel velocity, motor PWM, coordinates, or
emergency-stop state. Read [docs/AI_ARCHITECTURE.md](docs/AI_ARCHITECTURE.md)
and [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the detailed boundaries.

## Repository structure

```text
roverguard/       Python planning, control, safety, simulation, camera, and AI boundary
robot/            RoverGuard URDF model
world/maps/       Strict JSON warehouse and semantic house maps
tests/            Automated software tests
firmware/         Hardware-neutral C++17 controller and HAL
docs/             Architecture, camera, HomeGuard, and Sim-to-Real documentation
legacy/v0_1/      Early learning scripts preserved for project history
```

## Roadmap

- v0.6: semantic house navigation and room-based inspection.
- v0.7: visual anomaly classification with a local vision-language model.
- v0.8: verification actions, dynamic map updates, and automatic incident reports.
- v1.0: ESP32 hardware abstraction, encoders, real sensors, and Sim-to-Real tests.

## License

MIT License. See [LICENSE](LICENSE).
