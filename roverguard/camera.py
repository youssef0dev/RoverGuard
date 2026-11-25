"""Deterministic robot-mounted RGB camera capture for PyBullet."""

from __future__ import annotations

from datetime import datetime, timezone
import math
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw

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
    camera_joint: int,
    waypoint_name: str,
    output_directory: Path,
    config: CameraConfig,
) -> Path:
    """Capture four robot-relative views and save a labelled overview PNG.

    Four full-resolution files are written for FRONT, RIGHT, REAR and LEFT.
    The returned path points to a fifth 2x2 overview image. Keeping this
    function's return type as Path preserves the public simulation API.
    """
    projection = pybullet.computeProjectionMatrixFOV(
        fov=config.field_of_view_deg,
        aspect=config.width_px / config.height_px,
        nearVal=config.near_plane_m,
        farVal=config.far_plane_m,
    )
    output_directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
    safe_waypoint = "".join(
        character if character.isalnum() else "_"
        for character in waypoint_name.upper()
    )
    views = (
        ("FRONT", 0.0),
        ("RIGHT", -90.0),
        ("REAR", 180.0),
        ("LEFT", 90.0),
    )
    captured: list[tuple[str, Image.Image]] = []
    preferred_renderer = getattr(
        pybullet,
        "ER_BULLET_HARDWARE_OPENGL",
        pybullet.ER_TINY_RENDERER,
    )

    try:
        for label, yaw_offset_deg in views:
            pybullet.resetJointState(
                robot_id,
                camera_joint,
                targetValue=math.radians(yaw_offset_deg),
            )
            link_state = pybullet.getLinkState(
                robot_id,
                camera_joint,
                computeForwardKinematics=True,
            )
            position = link_state[4] if len(link_state) > 5 else link_state[0]
            orientation = link_state[5] if len(link_state) > 5 else link_state[1]
            rotation = pybullet.getMatrixFromQuaternion(orientation)
            direction = (rotation[0], rotation[3], rotation[6])
            up = (rotation[2], rotation[5], rotation[8])

            eye = [
                position[index]
                + direction[index] * config.forward_offset_m
                + up[index] * config.vertical_offset_m
                for index in range(3)
            ]
            vertical_drop = (
                math.tan(math.radians(config.pitch_down_deg))
                * config.target_distance_m
            )
            target = [
                eye[index]
                + direction[index] * config.target_distance_m
                - up[index] * vertical_drop
                for index in range(3)
            ]
            view_matrix = pybullet.computeViewMatrix(eye, target, list(up))
            render_arguments = {
                "width": config.width_px,
                "height": config.height_px,
                "viewMatrix": view_matrix,
                "projectionMatrix": projection,
                "shadow": 1,
                "lightDirection": [1.0, 1.0, 2.0],
            }
            try:
                camera_image = pybullet.getCameraImage(
                    **render_arguments,
                    renderer=preferred_renderer,
                )
            except Exception:
                # DIRECT/headless sessions may not expose an OpenGL context.
                camera_image = pybullet.getCameraImage(
                    **render_arguments,
                    renderer=pybullet.ER_TINY_RENDERER,
                )
            width, height, rgba = (
                camera_image[0],
                camera_image[1],
                camera_image[2],
            )
            raw = _pixel_bytes(rgba)
            expected_size = width * height * 4
            if len(raw) != expected_size:
                raise CameraCaptureError(
                    f"Expected {expected_size} RGBA bytes, received {len(raw)}"
                )

            frame = Image.frombytes("RGBA", (width, height), raw).convert("RGB")
            frame_path = output_directory / (
                f"{stamp}_{safe_waypoint}_{label}.png"
            )
            frame.save(frame_path, format="PNG")
            captured.append((label, frame))
    finally:
        pybullet.resetJointState(robot_id, camera_joint, targetValue=0.0)

    frame_width, frame_height = captured[0][1].size
    overview = Image.new(
        "RGB",
        (frame_width * 2, frame_height * 2),
        color=(18, 22, 30),
    )
    draw = ImageDraw.Draw(overview)
    positions = (
        (0, 0),
        (frame_width, 0),
        (0, frame_height),
        (frame_width, frame_height),
    )
    for (label, frame), (x, y) in zip(captured, positions):
        overview.paste(frame, (x, y))
        draw.rectangle((x + 8, y + 8, x + 112, y + 34), fill=(10, 15, 24))
        draw.text((x + 16, y + 14), label, fill=(255, 255, 255))

    overview_path = output_directory / (
        f"{stamp}_{safe_waypoint}_360_OVERVIEW.png"
    )
    overview.save(overview_path, format="PNG")
    return overview_path
