# RoverGuard mission camera captures

## Operator contract

The operator can request photos at selected route points:

```text
Go to P3 and take a photo, then P6, then P8 and take a photo, then return to BASE.
```

The local model can only produce two relevant lists:

```json
{
  "waypoints": ["P3", "P6", "P8", "BASE"],
  "photo_waypoints": ["P3", "P8"]
}
```

The validator rejects a photo point that is unknown or missing from the route.

## Capture sequence

1. A* plans and the path follower drives to the current target.
2. Arrival is checked with the deterministic goal tolerance.
3. Wheel speed becomes zero during the inspection pause.
4. If the target is in `photo_waypoints`, `camera.py` reads the front-sensor
   link pose.
5. PyBullet computes the view and perspective projection matrices.
6. TinyRenderer produces a 640x480 RGBA frame.
7. Pillow converts it to RGB and saves a timestamped PNG under `captures/`.

## Safety boundary

The LLM never supplies a filesystem path, renderer parameter, link index,
camera pose, or capture function. It requests only an approved waypoint name.

## Later real camera

On physical hardware, the camera adapter can change from PyBullet rendering to
an ESP32-CAM, USB camera, or Raspberry Pi camera. The mission schema and
validator remain the same.
