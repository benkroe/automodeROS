#!/usr/bin/env python3

from typing import Optional, Dict, Any, Tuple
from .behavior_interface import BehaviorBase
from automode_interfaces.msg import RobotState

import math

class Behavior(BehaviorBase):
    def __init__(self) -> None:
        self._node = None
        self._pub = None
        self._sub = None
        self._params: Dict[str, Any] = {}
        self._last_robot_state: Optional[RobotState] = None
        self._Float32MultiArray = None

        # Default parameters
        self._repulsion_gain = 4.0        # rep parameter (typical range [1, 5])
        self._neighbor_threshold = 0.1    # minimum proximity to consider neighbors
        self._wheel_speed_limit = 10.0    # max wheel speed (saturation safeguard)

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "repulsion",
            "type": 5,
            "description": "Moves away from neighboring robots using range-and-bearing repulsion",
            "params": [
                {
                    "name": "rep",
                    "type": "float",
                    "required": False,
                    "default": 4.0,
                }
            ]
        }

    def set_params(self, params: Dict[str, Any]) -> None:
        if not isinstance(params, dict):
            raise TypeError("params must be a dict")
        self._params.update(params)
        if "rep" in params:
            self._repulsion_gain = float(params["rep"])

    def _robot_state_cb(self, msg) -> None:
        self._last_robot_state = msg

    def setup_communication(self, node) -> None:
        self._node = node
        try:
            from std_msgs.msg import Float32MultiArray  # type: ignore
        except ImportError:
            if self._node is not None:
                self._node.get_logger().warning(
                    'Required std_msgs not available; setup_communication aborted'
                )
            return
        self._Float32MultiArray = Float32MultiArray
        self._pub = self._node.create_publisher(Float32MultiArray, 'wheels_speed', 10)
        self._sub = self._node.create_subscription(
            RobotState, 'robotState', self._robot_state_cb, 10
        )

    def execute_step(self) -> Tuple[bool, str, bool]:
        """
        Execute repulsion behavior - move away from neighboring robots.

        Returns:
            (success, message, goal_reached) 
            goal_reached always False for pure repulsion
        """
        if self._last_robot_state is None:
            return False, "No robot state data available", False

        if self._pub is None or self._Float32MultiArray is None:
            return False, "Communication not set up", False

        prox_mag = self._last_robot_state.proximity_magnitude
        prox_ang = self._last_robot_state.proximity_angle
        neighbor_count = self._last_robot_state.neighbour_count

        # If no neighbors or too weak signal → stop
        if neighbor_count == 0 or prox_mag <= self._neighbor_threshold:
            msg = self._Float32MultiArray()
            msg.data = [0.0, 0.0]
            self._pub.publish(msg)
            return True, f"No neighbors detected, stopped", False

        # REPULSION: Add 180° to move away from neighbors
        escape_angle = (prox_ang + 180) % 360

        # Repulsion vector scaled by rep
        vector_length = self._repulsion_gain * prox_mag
        angle_rad = math.radians(escape_angle)

        # Map vector into differential drive wheel speeds
        # Standard ARGoS-style mapping: 
        #   left  = v·cos(θ) - v·sin(θ)
        #   right = v·cos(θ) + v·sin(θ)
        v_cos = vector_length * math.cos(angle_rad)
        v_sin = vector_length * math.sin(angle_rad)
        left_wheel_speed = v_cos - v_sin
        right_wheel_speed = v_cos + v_sin

        # Saturate speeds to limits
        left_wheel_speed = max(min(left_wheel_speed, self._wheel_speed_limit), -self._wheel_speed_limit)
        right_wheel_speed = max(min(right_wheel_speed, self._wheel_speed_limit), -self._wheel_speed_limit)

        # Publish
        msg = self._Float32MultiArray()
        msg.data = [left_wheel_speed, right_wheel_speed]
        self._pub.publish(msg)

        return True, (
            f"Repulsion active (neighbors: {neighbor_count}, "
            f"prox: {prox_mag:.3f}, neighbor_angle: {prox_ang:.1f}°, "
            f"escape_angle: {escape_angle:.1f}°, "
            f"rep: {self._repulsion_gain:.1f}), "
            f"wheels: [{left_wheel_speed:.2f}, {right_wheel_speed:.2f}]"
        ), False

    def reset(self) -> None:
        """Reset the behavior state and clean up resources."""
        self._last_robot_state = None
        self._params = {}
        self._repulsion_gain = 4.0

        if self._node is not None:
            try:
                if self._pub is not None:
                    self._node.destroy_publisher(self._pub)
                if self._sub is not None:
                    self._node.destroy_subscription(self._sub)
            except Exception:
                if self._node is not None:
                    self._node.get_logger().warning('Error during behavior reset cleanup')

        self._pub = None
        self._sub = None
        self._Float32MultiArray = None