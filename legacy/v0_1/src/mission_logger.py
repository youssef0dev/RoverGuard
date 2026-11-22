import csv
from datetime import datetime
from pathlib import Path


class MissionLogger:
    FIELDNAMES = [
        "simulation_time_s",
        "state",
        "target",
        "odom_x_m",
        "odom_y_m",
        "odom_theta_deg",
        "front_distance_m",
        "left_distance_m",
        "right_distance_m",
        "body_clearance_m",
        "event",
    ]

    def __init__(self, log_directory="logs"):
        directory = Path(log_directory)
        directory.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = directory / f"mission_{timestamp}.csv"

        self.file = self.path.open(
            "w",
            newline="",
            encoding="utf-8",
        )

        self.writer = csv.DictWriter(
            self.file,
            fieldnames=self.FIELDNAMES,
        )

        self.writer.writeheader()
        self.file.flush()

    def log(
        self,
        simulation_time_s,
        state,
        target,
        odom_x,
        odom_y,
        odom_theta_deg,
        front_distance,
        left_distance,
        right_distance,
        body_clearance,
        event="",
    ):
        self.writer.writerow(
            {
                "simulation_time_s": f"{simulation_time_s:.3f}",
                "state": state,
                "target": target,
                "odom_x_m": f"{odom_x:.4f}",
                "odom_y_m": f"{odom_y:.4f}",
                "odom_theta_deg": f"{odom_theta_deg:.2f}",
                "front_distance_m": f"{front_distance:.4f}",
                "left_distance_m": f"{left_distance:.4f}",
                "right_distance_m": f"{right_distance:.4f}",
                "body_clearance_m": f"{body_clearance:.4f}",
                "event": event,
            }
        )

        if event:
            self.file.flush()

    def close(self):
        if not self.file.closed:
            self.file.flush()
            self.file.close()
