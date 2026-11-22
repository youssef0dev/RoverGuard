import unittest

from roverguard.safety import SafetyState, SafetySupervisor


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.safety = SafetySupervisor(0.12, 0.25, 0.30)

    def test_clear_slow_stop(self):
        self.assertEqual(self.safety.evaluate(0.80).state, SafetyState.CLEAR)
        self.assertEqual(self.safety.evaluate(0.20).state, SafetyState.SLOW)
        self.assertEqual(self.safety.evaluate(0.10).state, SafetyState.STOP)

    def test_hysteresis_requires_clear_distance(self):
        self.safety.evaluate(0.10)
        self.assertEqual(self.safety.evaluate(0.26).state, SafetyState.SLOW)
        self.assertEqual(self.safety.evaluate(0.31).state, SafetyState.CLEAR)


if __name__ == "__main__":
    unittest.main()

