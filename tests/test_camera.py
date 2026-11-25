import tempfile
import unittest
from pathlib import Path

from PIL import Image

from roverguard.camera import capture_robot_camera
from roverguard.config import CameraConfig


class FakePyBullet:
    ER_TINY_RENDERER = 7
    ER_BULLET_HARDWARE_OPENGL = 8

    def __init__(self):
        self.camera_angles = []

    def resetJointState(self, robot_id, camera_joint, targetValue):
        self.camera_angles.append(targetValue)

    def getLinkState(self, robot_id, sensor_joint, computeForwardKinematics):
        return (
            (0.0, 0.0, 0.1),
            (0.0, 0.0, 0.0, 1.0),
            None,
            None,
            (0.0, 0.0, 0.1),
            (0.0, 0.0, 0.0, 1.0),
        )

    def getMatrixFromQuaternion(self, orientation):
        return (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

    def computeViewMatrix(self, eye, target, up):
        return [1.0] * 16

    def computeProjectionMatrixFOV(self, **kwargs):
        return [1.0] * 16

    def getCameraImage(self, **kwargs):
        self.last_renderer = kwargs["renderer"]
        rgba = [255, 0, 0, 255, 0, 255, 0, 255]
        return (2, 1, rgba, (), ())


class CameraTests(unittest.TestCase):
    def test_four_views_and_overview_are_saved_as_rgb_png(self):
        config = CameraConfig(width_px=2, height_px=1)
        with tempfile.TemporaryDirectory() as directory:
            fake = FakePyBullet()
            path = capture_robot_camera(
                fake,
                robot_id=1,
                camera_joint=5,
                waypoint_name="P3",
                output_directory=Path(directory),
                config=config,
            )

            self.assertTrue(path.exists())
            self.assertIn("P3", path.name)
            self.assertIn("360_OVERVIEW", path.name)
            self.assertEqual(len(list(Path(directory).glob("*.png"))), 5)
            with Image.open(path) as image:
                self.assertEqual(image.mode, "RGB")
                self.assertEqual(image.size, (4, 2))
                self.assertEqual(image.getpixel((0, 0)), (255, 0, 0))
                self.assertEqual(image.getpixel((1, 0)), (0, 255, 0))
            self.assertEqual(fake.last_renderer, fake.ER_BULLET_HARDWARE_OPENGL)
            self.assertEqual(len(fake.camera_angles), 5)
            self.assertAlmostEqual(fake.camera_angles[-1], 0.0)

    def test_camera_uses_small_optical_offset_from_urdf_link(self):
        config = CameraConfig()
        self.assertGreater(config.vertical_offset_m, 0.0)
        self.assertLess(config.vertical_offset_m, 0.10)


if __name__ == "__main__":
    unittest.main()
