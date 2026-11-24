import csv
from datetime import datetime
from pathlib import Path

from .geometry import Pose2D


class TelemetryLogger:
    FIELDNAMES = (
        "simulation_time_s",
        "state",
        "target",
        "odom_x_m",
        "odom_y_m",
        "odom_theta_deg",
        "front_distance_m",
        "left_distance_m",
        "right_distance_m",
        "linear_command_mps",
        "angular_command_rps",
        "event",
    )

    def __init__(self, directory: Path):
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = directory / f"mission_v03_{stamp}.csv"
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()

    def log(
        self,
        simulation_time_s: float,
        state: str,
        target: str,
        pose: Pose2D,
        front_distance_m: float,
        left_distance_m: float,
        right_distance_m: float,
        linear_command_mps: float,
        angular_command_rps: float,
        event: str = "",
    ) -> None:
        self._writer.writerow(
            {
                "simulation_time_s": f"{simulation_time_s:.3f}",
                "state": state,
                "target": target,
                "odom_x_m": f"{pose.x:.4f}",
                "odom_y_m": f"{pose.y:.4f}",
                "odom_theta_deg": f"{pose.theta * 180.0 / 3.141592653589793:.2f}",
                "front_distance_m": f"{front_distance_m:.4f}",
                "left_distance_m": f"{left_distance_m:.4f}",
                "right_distance_m": f"{right_distance_m:.4f}",
                "linear_command_mps": f"{linear_command_mps:.4f}",
                "angular_command_rps": f"{angular_command_rps:.4f}",
                "event": event,
            }
        )
        if event:
            self._file.flush()

    def close(self) -> None:
        if not self._file.closed:
            self._file.flush()
            self._file.close()

    def __enter__(self) -> "TelemetryLogger":
        return self

    def __exit__(self, *_args) -> None:
        self.close()


