import tempfile
import unittest
from pathlib import Path

from PIL import Image

from roverguard.camera import capture_robot_camera
from roverguard.config import CameraConfig


class FakePyBullet:
    ER_TINY_RENDERER = 7

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
        rgba = [255, 0, 0, 255, 0, 255, 0, 255]
        return (2, 1, rgba, (), ())


class CameraTests(unittest.TestCase):
    def test_capture_is_saved_as_rgb_png(self):
        config = CameraConfig(width_px=2, height_px=1)
        with tempfile.TemporaryDirectory() as directory:
            path = capture_robot_camera(
                FakePyBullet(),
                robot_id=1,
                sensor_joint=3,
                waypoint_name="P3",
                output_directory=Path(directory),
                config=config,
            )

            self.assertTrue(path.exists())
            self.assertIn("P3", path.name)
            with Image.open(path) as image:
                self.assertEqual(image.mode, "RGB")
                self.assertEqual(image.size, (2, 1))
                self.assertEqual(image.getpixel((0, 0)), (255, 0, 0))
                self.assertEqual(image.getpixel((1, 0)), (0, 255, 0))


if __name__ == "__main__":
    unittest.main()
