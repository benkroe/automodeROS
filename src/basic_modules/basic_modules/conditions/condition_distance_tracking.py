#!/usr/bin/env python3

import random
from typing import Dict, Any, Tuple
from .conditions_interface import ConditionBase
from automode_interfaces.msg import RobotState

class Condition(ConditionBase):

    def __init__(self) -> None:
        self._node = None
        self._sub = None
        self._last_robot_state = None
        self._probability = 1.0
        self._threshold = 0.5  # magnitude threshold (0..1), higher means closer

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "distance_tracking",
            "type": 8,
            "description": "Triggers when target magnitude (0..1, closer=1) exceeds the threshold; optional probability gate.",
            "params": [
                {"name": "p", "type": "float64", "required": False, "default": 1.0},  # probability gate (0..1)
                {"name": "d", "type": "float64", "required": False, "default": 0.5}   # magnitude threshold (0..1)
            ]
        }

    def setup_communication(self, node) -> None:
        self._node = node
        self._sub = self._node.create_subscription(
            RobotState, 'robotState', self._robot_state_cb, 10
        )

    def set_params(self, params: Dict[str, Any]) -> None:
        try:
            self._probability = float(params.get('p', 1.0))
        except (TypeError, ValueError):
            self._probability = 1.0
        try:
            self._threshold = float(params.get('d', 0.5))
        except (TypeError, ValueError):
            self._threshold = 0.5
        self._threshold = max(0.0, min(1.0, self._threshold))
        self._probability = max(0.0, min(1.0, self._probability))

    def execute_reading(self) -> Tuple[bool, str]:
        if self._last_robot_state is None:
            return False, "Waiting for robot state"

        mag = float(getattr(self._last_robot_state, 'target_magnitude', 0.0))
        close_enough = mag >= self._threshold
        if close_enough and random.random() <= self._probability:
            return True, f"Target magnitude {mag:.3f} >= {self._threshold:.3f}"
        return False, f"Target magnitude {mag:.3f} < {self._threshold:.3f}"

    def _robot_state_cb(self, msg) -> None:
        self._last_robot_state = msg

    def reset(self) -> None:
        self._last_robot_state = None
        if self._node and self._sub:
            self._node.destroy_subscription(self._sub)
            self._sub = None
