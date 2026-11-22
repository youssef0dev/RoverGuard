from dataclasses import dataclass
from enum import Enum


class SafetyState(str, Enum):
    CLEAR = "CLEAR"
    SLOW = "SLOW"
    STOP = "STOP"


@dataclass(frozen=True)
class SafetyDecision:
    state: SafetyState
    speed_scale: float
    reason: str


class SafetySupervisor:
    """Independent last line of defence; it can always limit the controller."""

    def __init__(
        self,
        stop_distance_m: float,
        slow_distance_m: float,
        clear_distance_m: float,
    ):
        if not stop_distance_m < slow_distance_m < clear_distance_m:
            raise ValueError("Expected stop < slow < clear distances")
        self.stop_distance_m = stop_distance_m
        self.slow_distance_m = slow_distance_m
        self.clear_distance_m = clear_distance_m
        self._state = SafetyState.CLEAR

    @property
    def state(self) -> SafetyState:
        return self._state

    def evaluate(self, front_distance_m: float) -> SafetyDecision:
        # Hysteresis: after STOP/SLOW, the reading must become clearly safe.
        if self._state != SafetyState.CLEAR and front_distance_m < self.clear_distance_m:
            if front_distance_m <= self.stop_distance_m:
                self._state = SafetyState.STOP
                return SafetyDecision(self._state, 0.0, "front_emergency")
            self._state = SafetyState.SLOW
            return SafetyDecision(self._state, 0.35, "waiting_for_clearance")

        if front_distance_m <= self.stop_distance_m:
            self._state = SafetyState.STOP
            return SafetyDecision(self._state, 0.0, "front_emergency")
        if front_distance_m <= self.slow_distance_m:
            self._state = SafetyState.SLOW
            return SafetyDecision(self._state, 0.35, "front_near")

        self._state = SafetyState.CLEAR
        return SafetyDecision(self._state, 1.0, "clear")

