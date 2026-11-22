import unittest

from roverguard.kinematics import twist_to_wheel_speeds, wheel_speeds_to_twist


class KinematicsTests(unittest.TestCase):
    def test_forward_round_trip(self):
        wheels = twist_to_wheel_speeds(0.20, 0.0, 0.05, 0.23)
        self.assertAlmostEqual(wheels.left_rad_s, 4.0)
        self.assertAlmostEqual(wheels.right_rad_s, 4.0)

        twist = wheel_speeds_to_twist(
            wheels.left_rad_s,
            wheels.right_rad_s,
            0.05,
            0.23,
        )
        self.assertAlmostEqual(twist.linear_m_s, 0.20)
        self.assertAlmostEqual(twist.angular_rad_s, 0.0)

    def test_rotation_has_opposite_wheel_speeds(self):
        wheels = twist_to_wheel_speeds(0.0, 1.0, 0.05, 0.23)
        self.assertLess(wheels.left_rad_s, 0.0)
        self.assertGreater(wheels.right_rad_s, 0.0)


if __name__ == "__main__":
    unittest.main()

