import unittest

from roverguard.ai.planner import CognitiveMissionPlanner
from roverguard.ai.schema import MissionPlan, mission_json_schema
from roverguard.ai.validator import MissionValidationError, MissionValidator


class FakeProvider:
    def __init__(self, payload):
        self.payload = payload
        self.last_schema = None

    def generate_mission(
        self,
        *,
        system_prompt,
        user_command,
        json_schema,
    ):
        self.last_schema = json_schema
        return self.payload


def valid_payload():
    return {
        "mission_name": "Lab inspection",
        "waypoints": ["P1", "P3"],
        "return_to_base": True,
        "max_speed_mps": 0.18,
        "inspection_note": "Check the marked stations.",
    }


class MissionSchemaTests(unittest.TestCase):
    def test_schema_contains_only_configured_waypoints(self):
        schema = mission_json_schema(("P1", "BASE"))
        enum = schema["properties"]["waypoints"]["items"]["enum"]
        self.assertEqual(enum, ["P1", "BASE"])

    def test_unknown_field_is_rejected(self):
        payload = valid_payload()
        payload["motor_pwm"] = 100
        with self.assertRaises(ValueError):
            MissionPlan.from_mapping(payload)


class MissionValidatorTests(unittest.TestCase):
    def setUp(self):
        self.validator = MissionValidator(
            allowed_waypoints=("P1", "P2", "P3", "BASE"),
            max_robot_speed_mps=0.22,
        )

    def test_base_is_appended_once(self):
        plan = self.validator.validate(MissionPlan.from_mapping(valid_payload()))
        self.assertEqual(plan.waypoints, ("P1", "P3", "BASE"))
        self.assertTrue(plan.return_to_base)

    def test_unknown_waypoint_is_rejected(self):
        payload = valid_payload()
        payload["waypoints"] = ["P9"]
        with self.assertRaises(MissionValidationError):
            self.validator.validate(MissionPlan.from_mapping(payload))

    def test_unsafe_speed_is_rejected(self):
        payload = valid_payload()
        payload["max_speed_mps"] = 2.0
        with self.assertRaises(MissionValidationError):
            self.validator.validate(MissionPlan.from_mapping(payload))

    def test_repeated_inspection_is_rejected(self):
        payload = valid_payload()
        payload["waypoints"] = ["P1", "P1"]
        with self.assertRaises(MissionValidationError):
            self.validator.validate(MissionPlan.from_mapping(payload))

    def test_planner_uses_structured_schema_and_validator(self):
        provider = FakeProvider(valid_payload())
        planner = CognitiveMissionPlanner(provider, self.validator)
        plan = planner.plan("Inspect P1 and P3 then return to base")
        self.assertEqual(plan.waypoints[-1], "BASE")
        self.assertFalse(provider.last_schema["additionalProperties"])


if __name__ == "__main__":
    unittest.main()
