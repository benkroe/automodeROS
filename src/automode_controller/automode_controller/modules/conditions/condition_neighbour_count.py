#!/usr/bin/env python3

from typing import Dict, Any, Tuple
from .conditions_interface import ConditionBase  # Following black_floor pattern
from automode_interfaces.msg import RobotState

class Condition(ConditionBase):
    def __init__(self) -> None:
        self._node = None
        self._sub = None
        self._params: Dict[str, Any] = {}
        self._last_robot_state = None
        self._threshold = 1  # Default threshold

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "neighbour_count",
            "type": 3,
            "description": "Triggers when neighbour count reaches or exceeds threshold (p)",
            "params": [
                {"name": "p", "type": "float64", "required": False, "default": 1, # new threshold (fload converted then to int)
                 "name": "w", "type": "float64", "required": False, "default": 1} # don't know what that is stick only to it because of orgiginal
            ]
        }

    def setup_communication(self, node) -> None:
        self._node = node
        self._sub = self._node.create_subscription(
            RobotState, 'robotState', self._robot_state_cb, 10
        )

    def set_params(self, params: Dict[str, Any]) -> None:
        self._params.update(params)
        # Handle both string and int parameters from command line
        threshold_param = params.get('p', 1)
        try:
            self._threshold = int(threshold_param)
        except (ValueError, TypeError):
            self._threshold = 1  # Fallback to default

    def _robot_state_cb(self, msg) -> None:
        self._last_robot_state = msg

    def execute_reading(self) -> Tuple[bool, str]:
        """Check if neighbour count condition is met."""
        if self._last_robot_state is None:
            return False, "Waiting for robot state..."
        
        # Get neighbour count from robot state (with safe fallback)
        neighbour_count = getattr(self._last_robot_state, 'neighbour_count', 0)

        
        # Check if threshold is met or exceeded
        if neighbour_count >= self._threshold:
            return True, f"Neighbours detected: {neighbour_count} >= {self._threshold}"
        else:
            return False, f"Not enough neighbours: {neighbour_count} < {self._threshold}"

    def reset(self) -> None:
        self._last_robot_state = None
        if self._node and self._sub:
            try:
                self._node.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None