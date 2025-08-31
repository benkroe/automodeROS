#!/usr/bin/env python3

from typing import Dict, Any, Tuple
from .conditions_interface import ConditionBase
from automode_interfaces.msg import RobotState
import random

class Condition(ConditionBase):
    def __init__(self) -> None:
        self._node = None
        self._sub = None
        self._params: Dict[str, Any] = {}
        self._last_robot_state = None
        self._threshold = 1  # Default threshold (w parameter)
        self._probability = 1.0  # Default probability (p parameter)

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "inverted_neighbour_count",
            "type": 4,
            "description": "Triggers when neighbour count is below threshold (w) with probability (p)",
            "params": [
                {"name": "p", "type": "float64", "required": False, "default": 1.0},
                {"name": "w", "type": "float64", "required": False, "default": 1}
            ]
        }

    def setup_communication(self, node) -> None:
        self._node = node
        self._sub = self._node.create_subscription(
            RobotState, 'robotState', self._robot_state_cb, 10
        )

    def set_params(self, params: Dict[str, Any]) -> None:
        self._params.update(params)
        probability_param = params.get('p', 1.0)
        try:
            self._probability = float(probability_param)
        except (ValueError, TypeError):
            self._probability = 1.0
        threshold_param = params.get('w', 1)
        try:
            self._threshold = int(threshold_param)
        except (ValueError, TypeError):
            self._threshold = 1

    def _robot_state_cb(self, msg) -> None:
        self._last_robot_state = msg

    def execute_reading(self) -> Tuple[bool, str]:
        if self._last_robot_state is None:
            return False, "Waiting for robot state..."
        neighbour_count = getattr(self._last_robot_state, 'neighbour_count', 0)
        if random.random() > self._probability:
            return False, f"Skipped check (probability: {self._probability:.2f})"
        if neighbour_count < self._threshold:
            return True, f"Few neighbours: {neighbour_count} < {self._threshold} (p: {self._probability:.2f})"
        else:
            return False, f"Too many neighbours: {neighbour_count} >= {self._threshold} (p: {self._probability:.2f})"

    def reset(self) -> None:
        self._last_robot_state = None
        if self._node and self._sub:
            try:
                self._node.destroy_subscription(self._sub)
            except Exception:
                pass