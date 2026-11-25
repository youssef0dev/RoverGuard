import json
import tempfile
import unittest
from pathlib import Path

from roverguard.astar import AStarPlanner
from roverguard.config import AppConfig
from roverguard.simulation import DEFAULT_WORLD_MAP, HOME_WORLD_MAP, build_grid
from roverguard.world_map import WorldMapError, load_world_map


class SemanticHomeMapTests(unittest.TestCase):
    def setUp(self):
        self.home = load_world_map(HOME_WORLD_MAP)

    def test_home_map_defines_expected_semantic_rooms(self):
        self.assertEqual(self.home.schema_version, 2)
        self.assertEqual(
            set(self.home.room_lookup),
            {"ENTRANCE", "CORRIDOR", "SALON", "KITCHEN", "BEDROOM", "BATHROOM"},
        )
        for room in self.home.rooms:
            self.assertTrue(room.area.contains(room.inspection_point))
            self.assertEqual(
                self.home.waypoint_lookup[room.name],
                room.inspection_point,
            )

    def test_multilingual_room_aliases_resolve_to_canonical_names(self):
        self.assertEqual(self.home.resolve_room("cuisine").name, "KITCHEN")
        self.assertEqual(self.home.resolve_room("chambre").name, "BEDROOM")
        self.assertEqual(self.home.resolve_room("living room").name, "SALON")
        self.assertEqual(self.home.resolve_room("couloir").name, "CORRIDOR")

    def test_every_room_is_reachable_from_every_other_room(self):
        planner = AStarPlanner(build_grid(AppConfig(), self.home))
        points = [self.home.waypoint_lookup["BASE"]]
        points.extend(room.inspection_point for room in self.home.rooms)
        for start in points:
            for goal in points:
                with self.subTest(start=start, goal=goal):
                    path = planner.plan(start, goal)
                    self.assertEqual(path.points[-1], goal)

    def test_shared_room_alias_is_rejected(self):
        payload = json.loads(HOME_WORLD_MAP.read_text(encoding="utf-8"))
        payload["rooms"]["BEDROOM"]["aliases"].append("cuisine")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad-home.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaises(WorldMapError):
                load_world_map(path)

    def test_v1_warehouse_map_remains_backward_compatible(self):
        warehouse = load_world_map(DEFAULT_WORLD_MAP)
        self.assertEqual(warehouse.schema_version, 1)
        self.assertEqual(warehouse.rooms, ())


if __name__ == "__main__":
    unittest.main()
