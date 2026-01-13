#!/usr/bin/env python3

from typing import Dict, Any, Tuple
from .conditions_interface import ConditionBase
from automode_interfaces.msg import RobotState

class Condition(ConditionBase):
    """Condition that triggers based on presence/absence of a detected target (red ball)."""

    def __init__(self) -> None:
        self._node = None
        self._sub = None
        self._params: Dict[str, Any] = {}
        self._last_robot_state = None
        self._present_required = True  # True -> trigger when target present, False -> trigger when target absent
        self._magnitude_threshold = 1e-4  # minimal magnitude to consider target present

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "target_presence",
            "type": 6,
            "description": "Triggers based on presence (or absence) of detected target using red_ball_magnitude.",
            "params": [
                {"name": "present", "type": "bool", "required": False, "default": True}
            ]
        }

    def set_params(self, params: Dict[str, Any]) -> None:
        self._params.update(params)
        self._present_required = bool(params.get('present', True))

    def execute_reading(self) -> Tuple[bool, str]:
        if self._last_robot_state is None:
            return False, "Waiting for robot state..."

        mag = float(getattr(self._last_robot_state, 'red_ball_magnitude', 0.0))
        detected = mag > self._magnitude_threshold

        if self._present_required and detected:
            return True, f"Target present (mag={mag:.4f})"
        if (not self._present_required) and (not detected):
            return True, f"Target absent (mag={mag:.4f})"

        return False, f"Condition not met (mag={mag:.4f})"
        
    def setup_communication(self, node) -> None:
        self._node = node   
        self._sub = self._node.create_subscription(
            RobotState, 'robotState', self._robot_state_cb, 10
        )

    def _robot_state_cb(self, msg) -> None:
        self._last_robot_state = msg

    def reset(self) -> None:
        self._last_robot_state = None
        if self._node and self._sub:
            self._node.destroy_subscription(self._sub)
            self._sub = None
