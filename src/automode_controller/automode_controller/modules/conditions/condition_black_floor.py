#!/usr/bin/env python3

from typing import Dict, Any, Tuple
from .conditions_interface import ConditionBase
from automode_interfaces.msg import RobotState

class Condition(ConditionBase):
    def __init__(self) -> None:
        self._node = None
        self._sub = None
        self._params: Dict[str, Any] = {}
        self._last_robot_state = None
        self._detect_black = False  # True = trigger when black detected, False = trigger when no black

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "black_floor",
            "description": "Triggers when black floor is detected (or not detected)",
            "params": [
                {"name": "detect_black", "type": "bool", "required": False, "default": True, 
                 "description": "True = trigger when black detected, False = trigger when no black"}
            ]
        }

    def setup_communication(self, node) -> None:
        self._node = node
        self._sub = self._node.create_subscription(
            RobotState, 'robotState', self._robot_state_cb, 10
        )

    def set_params(self, params: Dict[str, Any]) -> None:
        self._params.update(params)
        self._detect_black = params.get('detect_black', True)

    def _robot_state_cb(self, msg) -> None:
        self._last_robot_state = msg

    def execute_step(self) -> Tuple[bool, str]:
        if self._last_robot_state is None:
            return False, "Waiting for robot state..."
        
        ground_black_floor = self._last_robot_state.ground_black_floor
        
        if self._detect_black:
            if ground_black_floor:
                return True, "Black floor detected!"
            else:
                return False, "No black floor detected yet"
        else:
            if not ground_black_floor:
                return True, "No black floor (condition met)"
            else:
                return False, "Black floor detected, waiting for no black floor"

    def reset(self) -> None:
        self._last_robot_state = None
        if self._node and self._sub:
            self._node.destroy_subscription(self._sub)
            self._sub = None