import unittest

from roverguard.astar import AStarPlanner, PathNotFoundError
from roverguard.config import AppConfig
from roverguard.geometry import Point2D
from roverguard.simulation import build_grid


class AStarTests(unittest.TestCase):
    def setUp(self):
        self.config = AppConfig()
        self.grid = build_grid(self.config)
        self.planner = AStarPlanner(self.grid)

    def test_path_to_p1_avoids_inflated_obstacle(self):
        start = Point2D(0.0, 0.0)
        goal = Point2D(1.15, 0.0)
        path = self.planner.plan(start, goal)

        self.assertEqual(path.points[0], start)
        self.assertEqual(path.points[-1], goal)
        self.assertGreater(len(path.raw_cells), 2)
        for cell in path.raw_cells:
            self.assertTrue(self.grid.is_free(cell))

    def test_blocked_goal_is_rejected(self):
        with self.assertRaises(PathNotFoundError):
            self.planner.plan(Point2D(0.0, 0.0), Point2D(0.70, 0.0))


if __name__ == "__main__":
    unittest.main()

