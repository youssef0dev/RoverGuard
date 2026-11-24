import unittest

from roverguard.astar import AStarPlanner, PathNotFoundError
from roverguard.config import AppConfig
from roverguard.geometry import Point2D
from roverguard.simulation import DEFAULT_WORLD_MAP, build_grid
from roverguard.world_map import load_world_map


class AStarTests(unittest.TestCase):
    def setUp(self):
        self.config = AppConfig()
        self.world_map = load_world_map(DEFAULT_WORLD_MAP)
        self.grid = build_grid(self.config, self.world_map)
        self.planner = AStarPlanner(self.grid)

    def test_path_to_p10_avoids_multiple_inflated_obstacles(self):
        start = self.world_map.waypoint_lookup["BASE"]
        goal = self.world_map.waypoint_lookup["P10"]
        path = self.planner.plan(start, goal)

        self.assertEqual(path.points[0], start)
        self.assertEqual(path.points[-1], goal)
        self.assertGreater(len(path.raw_cells), 2)
        self.assertGreater(len(path.points), 2)
        for cell in path.raw_cells:
            self.assertTrue(self.grid.is_free(cell))

    def test_blocked_goal_is_rejected(self):
        with self.assertRaises(PathNotFoundError):
            self.planner.plan(Point2D(0.0, 0.0), Point2D(1.50, 0.0))


if __name__ == "__main__":
    unittest.main()

