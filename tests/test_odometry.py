import math
import unittest

from roverguard.odometry import DifferentialDriveOdometry


class OdometryTests(unittest.TestCase):
    def test_equal_wheels_move_straight(self):
        odometry = DifferentialDriveOdometry(0.05, 0.23)
        odometry.reset(0.0, 0.0)
        pose = odometry.update(2.0, 2.0)
        self.assertAlmostEqual(pose.x, 0.10, places=6)
        self.assertAlmostEqual(pose.y, 0.0, places=6)
        self.assertAlmostEqual(pose.theta, 0.0, places=6)

    def test_opposite_wheels_rotate(self):
        odometry = DifferentialDriveOdometry(0.05, 0.23)
        odometry.reset(0.0, 0.0)
        pose = odometry.update(-1.0, 1.0)
        self.assertAlmostEqual(pose.x, 0.0, places=6)
        self.assertAlmostEqual(pose.y, 0.0, places=6)
        self.assertAlmostEqual(pose.theta, 0.10 / 0.23, places=6)
        self.assertLess(abs(pose.theta), math.pi)


if __name__ == "__main__":
    unittest.main()

