# RoverGuard v0.4 multi-point world

## Why the command input stays outside PyBullet

PyBullet is the physics and 3D visualization process. It is not the mission
authority. Operator input enters through a separate adapter and is converted
to the same strict mission contract. Today the adapter is terminal text;
later it can be speech-to-text without changing planning, safety, or motors.

```text
terminal now / voice later
            |
            v
      mission command
            |
            v
      LLM structured plan
            |
            v
   deterministic validator
            |
            v
 A* -> follower -> safety -> wheels
```

## Map file

`world/maps/warehouse_v1.json` is the single source of truth for:

- map bounds;
- robot start and BASE;
- named inspection points;
- the default route;
- static obstacle center, size, height, and display color.

`roverguard/world_map.py` rejects malformed maps before PyBullet starts.

## Static and dynamic obstacles

v0.4 handles several static, known obstacles. A* inflates each obstacle by the
robot footprint radius plus a safety margin, then plans through free cells.
The range sensors and safety supervisor remain active during motion.

A moving or newly discovered obstacle needs an online costmap update and a
re-plan from the current pose. That is intentionally the next milestone; it
should not be hidden inside the LLM.

## Voice boundary

The future voice component has one responsibility:

```text
audio -> text
```

The resulting text must pass through the existing LLM schema and deterministic
validator. Voice never talks directly to wheel motors.
