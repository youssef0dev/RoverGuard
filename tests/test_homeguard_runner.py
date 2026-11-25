import unittest

from roverguard.simulation import HOME_WORLD_MAP
from roverguard.world_map import WorldMapError, load_world_map
from run_homeguard import build_room_route


class HomeGuardRunnerTests(unittest.TestCase):
    def setUp(self):
        self.home = load_world_map(HOME_WORLD_MAP)

    def test_alias_route_is_canonical_and_returns_to_base(self):
        route = build_room_route(self.home, ["cuisine", "chambre"])
        self.assertEqual([name for name, _ in route], ["KITCHEN", "BEDROOM", "BASE"])

    def test_no_return_finishes_in_requested_room(self):
        route = build_room_route(
            self.home,
            ["salon"],
            return_to_base=False,
        )
        self.assertEqual([name for name, _ in route], ["SALON"])

    def test_duplicate_aliases_for_same_room_are_rejected(self):
        with self.assertRaises(WorldMapError):
            build_room_route(self.home, ["KITCHEN", "cuisine"])


if __name__ == "__main__":
    unittest.main()
