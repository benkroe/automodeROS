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

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "white_floor",
            "type": 1,
            "description": "Triggers when white floor is detected with probability (p).",
            "params": [
                {"name": "p", "type": "float64", "required": False, "default": 1.0}
            ]
        }

    def set_params(self, params: Dict[str, Any]) -> None:
        self._params.update(params)

    def execute_reading(self) -> Tuple[bool, str]:
        if self._last_robot_state is None:
            return False, "Waiting for robot state..."

        p = self._params.get('p', 1.0)
        if random.random() <= p:
            detected_color = getattr(self._last_robot_state, "floor_color", None)
            if detected_color == "white":
                return True, "White floor detected!"
            else:
                return False, f"White floor not detected (current: {detected_color})"
        else:
            return False, "Probability condition not met"
        
    def setup_communication(self, node) -> None:
        self._node = node   
        self._sub = self._node.create_subscription(
            RobotState, 'robotState', self._robot_state_cb, 10
        )

    def _robot_state_cb(self, msg) -> None:
        self._last_robot_state = msg

    def reset(self) -> None:
        self._last_robot_state = None
        self._sub = None
