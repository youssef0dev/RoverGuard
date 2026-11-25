# HomeGuard v0.6

HomeGuard is the semantic indoor-navigation layer of RoverGuard. It turns a
geometric world map into a map that can be addressed with human concepts such
as `KITCHEN`, `BEDROOM`, and `BATHROOM`.

## Why semantic rooms are separate from obstacles

The occupancy grid answers a geometric question: **where can the robot move?**
A semantic room answers a human question: **what is this place called and where
should the robot stop to inspect it?**

Keeping these responsibilities separate prevents a language model from
inventing coordinates. The language model selects a configured canonical room;
the deterministic map supplies its verified inspection point.

## Schema-v2 room contract

Each room contains:

- a canonical uppercase name;
- a human-readable display name;
- rectangular room bounds;
- one collision-checked inspection point;
- a desired future camera heading;
- multilingual aliases.

The loader rejects a room when its inspection point is outside its bounds, the
matching waypoint is missing, an alias is shared by two rooms, or the room is
outside the map.

## Current milestone

Version 0.6 milestone 1 provides semantic navigation and camera capture. The
next milestone will add a local vision-language inspector with a constrained
finding schema. Visual AI will recommend bounded actions, while a deterministic
safety supervisor will remain responsible for stop, reinspection, rerouting,
and return-to-base decisions.
