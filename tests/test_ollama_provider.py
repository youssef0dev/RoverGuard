import json
import unittest
import urllib.error
from unittest.mock import patch

from roverguard.ai.ollama_provider import OllamaProvider, OllamaProviderError


class FakeHttpResponse:
    def __init__(self, payload):
        self.payload = payload

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        return False

    def read(self):
        return json.dumps(self.payload).encode("utf-8")


class OllamaProviderTests(unittest.TestCase):
    def test_structured_response_is_parsed(self):
        mission = {
            "mission_name": "Inspection",
            "waypoints": ["P1"],
            "return_to_base": True,
            "max_speed_mps": 0.15,
            "inspection_note": "Check P1.",
        }
        api_payload = {"message": {"content": json.dumps(mission)}}
        provider = OllamaProvider(model="test-model")

        with patch(
            "urllib.request.urlopen",
            return_value=FakeHttpResponse(api_payload),
        ):
            result = provider.generate_mission(
                system_prompt="system",
                user_command="inspect P1",
                json_schema={"type": "object"},
            )

        self.assertEqual(result, mission)

    def test_offline_server_has_clear_boundary_error(self):
        provider = OllamaProvider(model="test-model")
        with patch(
            "urllib.request.urlopen",
            side_effect=urllib.error.URLError("offline"),
        ):
            with self.assertRaises(OllamaProviderError):
                provider.generate_mission(
                    system_prompt="system",
                    user_command="inspect P1",
                    json_schema={"type": "object"},
                )


if __name__ == "__main__":
    unittest.main()
