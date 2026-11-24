import json
import tempfile
import unittest
from pathlib import Path

from roverguard.config import AppConfig
from roverguard.astar import AStarPlanner
from roverguard.simulation import DEFAULT_WORLD_MAP, build_grid
from roverguard.world_map import WorldMapError, load_world_map


class WorldMapTests(unittest.TestCase):
    def test_warehouse_map_has_many_safe_points_and_obstacles(self):
        world_map = load_world_map(DEFAULT_WORLD_MAP)
        grid = build_grid(AppConfig(), world_map)

        self.assertGreaterEqual(len(world_map.waypoints), 10)
        self.assertGreaterEqual(len(world_map.obstacles), 5)
        for _, point in world_map.waypoints:
            self.assertTrue(grid.is_free(grid.world_to_cell(point)))

    def test_unknown_default_route_point_is_rejected(self):
        payload = json.loads(DEFAULT_WORLD_MAP.read_text(encoding="utf-8"))
        payload["default_route"] = ["UNKNOWN"]

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad-map.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaises(WorldMapError):
                load_world_map(path)

    def test_every_waypoint_is_reachable_from_base(self):
        world_map = load_world_map(DEFAULT_WORLD_MAP)
        grid = build_grid(AppConfig(), world_map)
        planner = AStarPlanner(grid)
        base = world_map.waypoint_lookup["BASE"]

        for name, point in world_map.waypoints:
            with self.subTest(waypoint=name):
                path = planner.plan(base, point)
                self.assertEqual(path.points[-1], point)


if __name__ == "__main__":
    unittest.main()
