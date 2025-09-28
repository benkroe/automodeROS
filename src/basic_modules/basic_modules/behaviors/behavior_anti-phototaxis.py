#!/usr/bin/env python3

from typing import Optional, Dict, Any, Tuple
from .behavior_interface import BehaviorBase
from automode_interfaces.msg import RobotState

import time

class Behavior(BehaviorBase):
    def __init__(self) -> None:
        self._node = None
        self._pub = None
        self._sub = None
        self._params: Dict[str, Any] = {}
        self._last_robot_state: Optional[RobotState] = None
        self._Float32MultiArray = None

        self._forward_speed = 1.0       # Forward movement speed
        self._turn_speed = 5.0          # Turning speed (will calibrate later)
        self._light_threshold = 0.0    # Light detection threshold

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "anti_phototaxis",
            "type": 3,
            "description": "If it sees light, it walks away from this direction",
            "params": []
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
        """
        Execute anti-phototaxis behavior - move away from light source.
        
        Returns:
            (success, message, goal_reached)
        """
        # Check if we have robotState
        if self._last_robot_state is None:
                return True, "No robot state data available", False

        # Check if publisher is ready
        if self._pub is None or self._Float32MultiArray is None:
            return False, "Communication not set up", False

        light_magnitude = self._last_robot_state.light_magnitude
        light_angle = self._last_robot_state.light_angle

        # If no light detected, stop movement
        if light_magnitude <= self._light_threshold:
            msg = self._Float32MultiArray()
            msg.data = [0.0, 0.0] 
            self._pub.publish(msg)
            return True, f"No light detected (magnitude: {light_magnitude:.3f}), stopped", False

        base_speed = self._forward_speed
        escape_angle = (light_angle + 180) % 360

        # Use same turning logic as phototaxis (smooth differential steering)
        if escape_angle <= 180:
            turn_angle = escape_angle
        else:
            turn_angle = escape_angle - 360

        # Apply smooth differential steering (same as phototaxis)
        left_wheel_speed = base_speed - self._turn_speed * (turn_angle / 180.0)
        right_wheel_speed = base_speed + self._turn_speed * (turn_angle / 180.0)

        # Publish wheel speeds
        msg = self._Float32MultiArray()
        msg.data = [left_wheel_speed, right_wheel_speed]
        self._pub.publish(msg)

        # Goal is reached when we've escaped from strong light
        goal_reached = light_magnitude < 0.0

        return True, f"Escaping from light (mag: {light_magnitude:.3f}, light_angle: {light_angle:.1f}°, escape_angle: {escape_angle:.1f}°, turn: {turn_angle:.1f}°), wheels: [{left_wheel_speed:.2f}, {right_wheel_speed:.2f}]", goal_reached
    
    def reset(self) -> None:
        """Reset the behavior state and clean up resources."""
        # Reset behavior-specific state variables
        self._last_robot_state = None
        self._params = {}
        
        # Clean up ROS2 resources if node exists
        if self._node is not None:
            try:
                if self._pub is not None:
                    try:
                        self._node.destroy_publisher(self._pub)
                    except Exception:
                        pass
                    self._pub = None
                    
                if self._sub is not None:
                    try:
                        self._node.destroy_subscription(self._sub)
                    except Exception:
                        pass
                    self._sub = None
            except Exception:
                # Log any unexpected errors during cleanup
                if self._node is not None:
                    self._node.get_logger().warning('Error during behavior reset cleanup')
        else:
            # If no node, just clear the publishers/subscribers
            self._pub = None
            self._sub = None
        
        # Reset message type reference
        self._Float32MultiArray = None