"""Deterministic robot-mounted RGB camera capture for PyBullet."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from PIL import Image

from .config import CameraConfig


class CameraCaptureError(RuntimeError):
    """Raised when a simulated camera frame cannot be encoded."""


def _pixel_bytes(rgba: Any) -> bytes:
    if hasattr(rgba, "tobytes"):
        return rgba.tobytes()

    try:
        return bytes(rgba)
    except (TypeError, ValueError):
        flat = bytearray()
        for item in rgba:
            if isinstance(item, int):
                flat.append(item)
            else:
                flat.extend(item)
        return bytes(flat)


def capture_robot_camera(
    pybullet,
    *,
    robot_id: int,
    sensor_joint: int,
    waypoint_name: str,
    output_directory: Path,
    config: CameraConfig,
) -> Path:
    """Capture the view in front of the sensor link and save a PNG."""
    link_state = pybullet.getLinkState(
        robot_id,
        sensor_joint,
        computeForwardKinematics=True,
    )
    position = link_state[4] if len(link_state) > 5 else link_state[0]
    orientation = link_state[5] if len(link_state) > 5 else link_state[1]
    rotation = pybullet.getMatrixFromQuaternion(orientation)

    forward = (rotation[0], rotation[3], rotation[6])
    up = (rotation[2], rotation[5], rotation[8])
    eye = [
        position[index]
        + forward[index] * config.forward_offset_m
        + up[index] * config.vertical_offset_m
        for index in range(3)
    ]
    target = [eye[index] + forward[index] * 3.0 for index in range(3)]

    view = pybullet.computeViewMatrix(eye, target, list(up))
    projection = pybullet.computeProjectionMatrixFOV(
        fov=config.field_of_view_deg,
        aspect=config.width_px / config.height_px,
        nearVal=config.near_plane_m,
        farVal=config.far_plane_m,
    )
    image = pybullet.getCameraImage(
        width=config.width_px,
        height=config.height_px,
        viewMatrix=view,
        projectionMatrix=projection,
        shadow=1,
        renderer=pybullet.ER_TINY_RENDERER,
    )
    width, height, rgba = image[0], image[1], image[2]
    raw = _pixel_bytes(rgba)
    expected_size = width * height * 4
    if len(raw) != expected_size:
        raise CameraCaptureError(
            f"Expected {expected_size} RGBA bytes, received {len(raw)}"
        )

    output_directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
    safe_waypoint = "".join(
        character if character.isalnum() else "_"
        for character in waypoint_name.upper()
    )
    path = output_directory / f"{stamp}_{safe_waypoint}.png"
    Image.frombytes("RGBA", (width, height), raw).convert("RGB").save(
        path,
        format="PNG",
    )
    return path
