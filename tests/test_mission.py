import unittest

from roverguard.config import MissionConfig
from roverguard.mission import PatrolMission


class MissionTests(unittest.TestCase):
    def test_waypoint_order(self):
        mission = PatrolMission(MissionConfig().waypoints)
        visited = []
        while not mission.complete:
            visited.append(mission.mark_current_reached().name)
        self.assertEqual(visited, ["P1", "P2", "P3", "BASE"])


if __name__ == "__main__":
    unittest.main()

