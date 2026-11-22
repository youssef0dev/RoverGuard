import unittest

from roverguard.controllers import PointController
from roverguard.geometry import Point2D, Pose2D
from roverguard.path_follower import PathFollower


class PathFollowerTests(unittest.TestCase):
    def test_intermediate_point_uses_its_own_tighter_tolerance(self):
        controller = PointController(
            max_linear_m_s=0.22,
            max_angular_rad_s=1.8,
            distance_tolerance_m=0.08,
        )
        follower = PathFollower(controller, intermediate_tolerance_m=0.025)
        follower.set_path(
            (
                Point2D(0.0, 0.0),
                Point2D(0.325, -0.375),
                Point2D(1.075, -0.375),
                Point2D(1.15, 0.0),
            )
        )

        # 7.4 cm from the intermediate corner: smaller than the final-goal
        # tolerance (8 cm), but still larger than the corner tolerance (2.5 cm).
        command = follower.compute(Pose2D(0.29, -0.31, -1.0))

        self.assertFalse(command.reached)
        self.assertGreater(abs(command.linear_m_s) + abs(command.angular_rad_s), 0.0)
        self.assertEqual(follower.current_target, Point2D(0.325, -0.375))


if __name__ == "__main__":
    unittest.main()
