#!/usr/bin/env python3

from typing import Optional, Dict, Any, Tuple
from .behavior_interface import BehaviorBase
from automode_interfaces.msg import RobotState

import random
import time

class Behavior(BehaviorBase):
    def __init__(self) -> None:
        self._node = None
        self._pub = None
        self._sub = None
        self._params: Dict[str, Any] = {}
        self._last_robot_state: Optional[RobotState] = None
        self._Float32MultiArray = None
        self._turning_time = time.time()

        self._obstacle_threshold = 9.99  # Proximity magnitude threshold for obstacle detection
        self._forward_speed = 0.3       # Forward movement speed
        self._turn_speed = 4.0         # Turning speed (will calibrate later)

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "exploration",
            "type": 0,
            "description": "Walks around random. Goes in one direction until obtacle is seen turns around a random angle and goes forward again.",
            "params": [
                {"name": "rwm", "type": "int", "required": False, "default": 100}
                 ]
        }

    def set_params(self, params: Dict[str, Any]) -> None:
        if not isinstance(params, dict):
            raise TypeError("params must be a dict")
        self._params.update(params)

    def _robot_state_cb(self, msg) -> None:
        self._last_robot_state = msg

    def setup_communication(self, node) -> None:
        self._node = node
        try:
            from std_msgs.msg import Float32MultiArray, String  # type: ignore
        except ImportError:
            if self._node is not None:
                self._node.get_logger().warning('Required std_msgs not available; setup_communication aborted')
            return
        self._Float32MultiArray = Float32MultiArray
        self._pub = self._node.create_publisher(Float32MultiArray, 'wheels_speed', 10)
        self._sub = self._node.create_subscription(RobotState, 'robotState', self._robot_state_cb, 10)

    def execute_step(self) -> Tuple[bool, str, bool]:
        # If currently turning, keep publishing turn speeds
        if self._turning_time > time.time():
            msg = self._Float32MultiArray()
            if hasattr(self, "_last_turn_direction"):
                msg.data = self._last_turn_direction
                self._pub.publish(msg)
            return True, "turning", False

        proximity_magnitude = 0.0
        proximity_angle = 0.0

        if self._last_robot_state:
            proximity_magnitude = self._last_robot_state.proximity_magnitude
            proximity_angle = self._last_robot_state.proximity_angle

        msg = self._Float32MultiArray()

        # Only turn if obstacle is close AND roughly in front (angle near 0)
        if proximity_magnitude < self._obstacle_threshold and proximity_magnitude != 0.0:
            self._pub.publish(msg)
            # Obstacle detected, initiate turn
            if proximity_angle > 0:
                turn_direction = [self._turn_speed, -self._turn_speed]  # Turn left
            else:
                turn_direction = [-self._turn_speed, self._turn_speed]  # Turn right

            self._last_turn_direction = turn_direction
            rwm = int(self._params.get("rwm", 100))
            self._turning_time = time.time() + (random.uniform(0, rwm))/30
            msg.data = turn_direction
            self._pub.publish(msg)
        else:
            msg.data = [self._forward_speed, self._forward_speed]
            self._pub.publish(msg)

        return True, "published wheel speeds", False
    
    def reset(self) -> None:
        """Reset the behavior state."""
        self._turning_time = time.time()
        self._last_robot_state = None
        self._params = {}
        self._pub = None
        self._sub = None
        self._Float32MultiArray = None