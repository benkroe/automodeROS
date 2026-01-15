#!/usr/bin/env python3

from typing import Optional, Dict, Any, Tuple
from .behavior_interface import BehaviorBase
from automode_interfaces.msg import RobotState

import random
import time
import math

class Behavior(BehaviorBase):
    """Collective behavior with internal FSM: free (explore with obstacle avoidance) -> recruited (stay) -> leaving (move away)"""

    def __init__(self) -> None:
        self._node = None
        self._pub = None
        self._sub = None
        self._params: Dict[str, Any] = {}
        self._last_robot_state: Optional[RobotState] = None
        self._Float32MultiArray = None
        
        # FSM state
        self._state = "free"  # free, recruited, leaving
        self._turning_time = time.time()
        self._last_turn_direction = [0.0, 0.0]
        
        # Behavior parameters
        self._proximity_range = 0.5     # proximity_magnitude threshold for "within range"
        self._recruitment_prob = 0.1   # probability j to transition free -> recruited
        self._leaving_prob = 0.05      # probability q to transition recruited -> leaving
        
        # Movement speeds
        self._forward_speed = 0.3
        self._turn_speed = 1.0
        self._obstacle_threshold = 9.99
        self._leaving_speed = -0.5     # negative = backward/away

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "recruitment",
            "type": 9,
            "description": "FSM behavior: free (explore+obstacle avoidance) -> recruited (stay) -> leaving (move away)",
            "params": [
                {"name": "r", "type": "float64", "required": False, "default": 0.5},   # proximity range threshold
                {"name": "j", "type": "float64", "required": False, "default": 0.1},   # recruitment probability
                {"name": "q", "type": "float64", "required": False, "default": 0.05},  # leaving probability
                {"name": "rwm", "type": "int", "required": False, "default": 100}      # random walk modulation
            ]
        }

    def set_params(self, params: Dict[str, Any]) -> None:
        if not isinstance(params, dict):
            raise TypeError("params must be a dict")
        self._params.update(params)
        
        try:
            self._proximity_range = float(params.get("r", 0.5))
        except (TypeError, ValueError):
            self._proximity_range = 0.5
        
        try:
            self._recruitment_prob = float(params.get("j", 0.1))
        except (TypeError, ValueError):
            self._recruitment_prob = 0.1
        
        try:
            self._leaving_prob = float(params.get("q", 0.05))
        except (TypeError, ValueError):
            self._leaving_prob = 0.05

    def _robot_state_cb(self, msg) -> None:
        self._last_robot_state = msg

    def setup_communication(self, node) -> None:
        self._node = node
        try:
            from std_msgs.msg import Float32MultiArray
        except ImportError:
            if self._node is not None:
                self._node.get_logger().warning('Required std_msgs not available; setup_communication aborted')
            return
        self._Float32MultiArray = Float32MultiArray
        self._pub = self._node.create_publisher(Float32MultiArray, 'wheels_speed', 10)
        self._sub = self._node.create_subscription(RobotState, 'robotState', self._robot_state_cb, 10)

    def execute_step(self) -> Tuple[bool, str, bool]:
        if self._node is None or self._pub is None:
            return False, "recruitment behavior not initialized", False

        proximity_magnitude = 0.0
        proximity_angle = 0.0

        if self._last_robot_state:
            proximity_magnitude = self._last_robot_state.proximity_magnitude
            proximity_angle = math.degrees(self._last_robot_state.proximity_angle)

        within_range = proximity_magnitude > 0.0 and proximity_magnitude <= self._proximity_range

        # FSM transitions
        if self._state == "free":
            if within_range and random.random() < self._recruitment_prob:
                self._state = "recruited"
                self._node.get_logger().info(f"[Recruitment] Transition: free -> recruited")
        
        elif self._state == "recruited":
            if not within_range:
                self._state = "free"
                self._node.get_logger().info(f"[Recruitment] Transition: recruited -> free")
            elif within_range and random.random() < self._leaving_prob:
                self._state = "leaving"
                self._node.get_logger().info(f"[Recruitment] Transition: recruited -> leaving")
        
        elif self._state == "leaving":
            if not within_range:
                self._state = "free"
                self._node.get_logger().info(f"[Recruitment] Transition: leaving -> free")

        # Execute behavior based on current state
        msg = self._Float32MultiArray()

        if self._state == "free":
            # Random walk with obstacle avoidance
            if self._turning_time > time.time():
                msg.data = self._last_turn_direction
                self._pub.publish(msg)
            elif proximity_magnitude < self._obstacle_threshold and proximity_magnitude != 0.0:
                # Obstacle detected, turn
                if proximity_angle < 0:
                    turn_direction = [self._turn_speed, -self._turn_speed]  # Turn left
                else:
                    turn_direction = [-self._turn_speed, self._turn_speed]   # Turn right

                self._last_turn_direction = turn_direction
                rwm = int(self._params.get("rwm", 100))
                self._turning_time = time.time() + (random.uniform(0, rwm)) / 30
                msg.data = turn_direction
                self._pub.publish(msg)
            else:
                # Move forward
                msg.data = [self._forward_speed, self._forward_speed]
                self._pub.publish(msg)

        elif self._state == "recruited":
            # Stay in place (stop)
            msg.data = [0.0, 0.0]
            self._pub.publish(msg)

        elif self._state == "leaving":
            # Move away (turn away and move backward)
            if proximity_angle < 0:
                # Robot on left, turn right and back away
                msg.data = [self._leaving_speed, -self._turn_speed]
            else:
                # Robot on right, turn left and back away
                msg.data = [-self._turn_speed, self._leaving_speed]
            self._pub.publish(msg)

        return True, f"state={self._state}, proximity={proximity_magnitude:.3f}", False

    def reset(self) -> None:
        """Reset behavior state."""
        self._state = "free"
        self._turning_time = time.time()
        self._last_robot_state = None
        self._params = {}
        self._pub = None
        self._sub = None
        self._Float32MultiArray = None
