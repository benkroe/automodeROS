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
        self._turn_speed = 1.0          # Turning speed
        self._light_threshold = 0.0     # Light detection threshold
        self._obstacle_threshold = 70   # Proximity threshold to consider obstacle too close
        self._avoidance_turn_speed = 2.0  # Turn speed when avoiding obstacles

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "phototaxis",
            "type": 2,
            "description": "If it sees light, it walks in this direction",
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
        Execute phototaxis behavior with obstacle avoidance.
        
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
        proximity_magnitude = self._last_robot_state.proximity_magnitude
        proximity_angle = self._last_robot_state.proximity_angle

        # OBSTACLE AVOIDANCE TAKES COMPLETE PRIORITY
        if proximity_magnitude > self._obstacle_threshold:
            # Forget about the light, only focus on avoiding obstacle
            # Normalize proximity angle to [-180, 180]
            if proximity_angle > 180:
                proximity_angle -= 360
            
            # Turn away from obstacle direction
            if proximity_angle < 0:  # Obstacle on left, turn right
                left_wheel_speed = -self._avoidance_turn_speed
                right_wheel_speed = self._avoidance_turn_speed
            else:  # Obstacle on right, turn left
                left_wheel_speed = self._avoidance_turn_speed
                right_wheel_speed = -self._avoidance_turn_speed

            status_msg = f"AVOIDING OBSTACLE (prox_mag: {proximity_magnitude:.3f}, prox_angle: {proximity_angle:.1f}°)"
        else:
            # No obstacle, apply normal phototaxis behavior
            left_wheel_speed = self._forward_speed
            right_wheel_speed = self._forward_speed
            
            if light_magnitude > self._light_threshold:
                # Convert light angle to turning direction
                if light_angle > 180:
                    turn_angle = light_angle - 360
                else:
                    turn_angle = light_angle
                
                # Apply differential steering toward light
                turn_factor = self._turn_speed * (turn_angle / 180.0)
                left_wheel_speed -= turn_factor
                right_wheel_speed += turn_factor
                
                status_msg = f"Moving toward light (light_mag: {light_magnitude:.3f}, light_angle: {light_angle:.1f}°)"
            else:
                status_msg = f"No light detected, moving forward"

        # Ensure wheel speeds don't go negative
        left_wheel_speed = max(0.0, left_wheel_speed)
        right_wheel_speed = max(0.0, right_wheel_speed)

        # Publish wheel speeds
        msg = self._Float32MultiArray()
        msg.data = [left_wheel_speed, right_wheel_speed]
        self._pub.publish(msg)

        # Behavior never finishes - always returns False for goal_reached
        return True, f"{status_msg}, wheels: [{left_wheel_speed:.2f}, {right_wheel_speed:.2f}]", False
    
    def reset(self) -> None:
        """Reset the behavior state."""
        self._last_robot_state = None
        self._params = {}
        self._pub = None
        self._sub = None
        self._Float32MultiArray = None