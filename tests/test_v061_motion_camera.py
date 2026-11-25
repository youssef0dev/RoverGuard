import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from roverguard.config import AppConfig


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class V061MotionAndCameraTests(unittest.TestCase):
    def test_motion_limits_are_fast_but_bounded(self):
        config = AppConfig()
        self.assertGreater(config.robot.max_linear_speed_mps, 0.22)
        self.assertLessEqual(config.robot.max_linear_speed_mps, 0.30)
        self.assertGreaterEqual(config.simulation.max_seconds, 300.0)

    def test_sensor_thresholds_are_ordered(self):
        sensor = AppConfig().sensor
        self.assertLess(sensor.stop_distance_m, sensor.slow_distance_m)
        self.assertLess(sensor.slow_distance_m, sensor.clear_distance_m)

    def test_urdf_has_dedicated_camera_mast_and_head(self):
        root = ET.parse(PROJECT_ROOT / "robot" / "roverguard.urdf").getroot()
        links = {item.attrib["name"] for item in root.findall("link")}
        joints = {
            item.attrib["name"]: item.attrib["type"]
            for item in root.findall("joint")
        }
        self.assertIn("camera_mast", links)
        self.assertIn("camera_head", links)
        self.assertIn("camera_head_joint", joints)
        self.assertEqual(joints["camera_head_joint"], "continuous")


if __name__ == "__main__":
    unittest.main()
