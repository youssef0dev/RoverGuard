# RoverGuard safe AI architecture

## Safety boundary

The local LLM is a mission translator, not a motor controller.

```text
Operator language
    -> Ollama structured JSON
    -> MissionPlan strict parser
    -> MissionValidator deterministic policy
    -> configured waypoint coordinates
    -> A* + path follower + SafetySupervisor
    -> wheel commands
```

The model cannot create coordinates, PWM values, joint speeds, shell commands,
or Python code. It can only select configured waypoint names, mark a subset of
the approved route for photos, choose a bounded maximum speed, set a
return-to-base flag, and write a short inspection note.

## Failure behavior

- Ollama offline: mission is rejected before PyBullet starts.
- Invalid JSON: mission is rejected.
- Unknown field or waypoint: mission is rejected.
- Unsafe speed: mission is rejected.
- Obstacle during execution: deterministic safety and A* remain responsible.
- LLM process crashes after approval: the approved mission remains ordinary
  deterministic waypoint navigation.

## Local model

Default model: `qwen3.5:4b-q4_K_M` through the local Ollama HTTP API.
The integration uses JSON Schema structured output and Python's standard
library, so the runtime has no cloud API key and no extra Python SDK dependency.

## Auditability

Every approved request is stored under `logs/ai_mission_*.json` with:

- UTC timestamp;
- model name;
- original operator request;
- approved normalized mission;
- validation status.

These runtime logs are excluded from Git.

## Camera action boundary

The LLM can request a photo only by adding a route point to `photo_waypoints`.
The deterministic validator checks that the point exists and belongs to the
approved route. The simulation captures only after confirmed arrival.

Voice input is the next adapter. It will convert audio to text and feed this
same guarded mission path. Future image analysis will also remain outside the
real-time motor and safety loop.
