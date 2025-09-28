#!/usr/bin/env python3

from typing import Dict, Any, Tuple
from .conditions_interface import ConditionBase
from automode_interfaces.msg import RobotState
import random

class Condition(ConditionBase):
    COLOR_MAP = {
        1.0: "black",
        2.0: "gray",
        3.0: "white"
    }

    def __init__(self) -> None:
        self._node = None
        self._sub = None
        self._params: Dict[str, Any] = {}
        self._last_robot_state = None
        self._target_color_float = 2.0  # Default

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "floor_color",
            "type": 0,
            "description": "Triggers when specified floor color (float: 1=black, 2=gray, 3=white) is detected with probability (p).",
            "params": [
                {"name": "color", "type": "float64", "required": True, "default": 1.0},
                {"name": "p", "type": "float64", "required": False, "default": 1}
            ]
        }

    def set_params(self, params: Dict[str, Any]) -> None:
        self._params.update(params)
        self._target_color_float = float(params.get('color', 2.0))

    def execute_reading(self) -> Tuple[bool, str]:
        if self._last_robot_state is None:
            return False, "Waiting for robot state..."

        p = self._params.get('p', 1.0)
        if random.random() <= p:
            detected_color = getattr(self._last_robot_state, "floor_color", None)
            expected_color = self.COLOR_MAP.get(self._target_color_float, None)
            if expected_color is None:
                return False, f"Unknown color mapping for value {self._target_color_float}"
            if detected_color == expected_color:
                return True, f"Floor color {expected_color} detected!"
            else:
                return False, f"Floor color {expected_color} not detected (current: {detected_color})"
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
        if self._node and self._sub:
            self._node.destroy_subscription(self._sub)
            self._sub = None