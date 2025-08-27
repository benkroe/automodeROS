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
        self._last_robot_state: Optional[str] = None
        self._Float32MultiArray = None
        self._turning_time = time.time()

        self._obstacle_threshold = 0.5  # Proximity magnitude threshold for obstacle detection
        self._forward_speed = 1.0       # Forward movement speed
        self._turn_speed = 1.0          # Turning speed (will calibrate later)

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "exploration",
            "type": 0,
            "description": "Walks around random. Goes in one direction until obtacle is seen turns around a random angle and goes forward again.",
            "params": [
                {"name": "Max angle", "type": "int", "required": False, "default": 100}
                 ]
        }

    def set_params(self, params: Dict[str, Any]) -> None:
        if not isinstance(params, dict):
            raise TypeError("params must be a dict")
        self._params.update(params)

    def _robot_state_cb(self, msg) -> None:
        try:
            if hasattr(msg, "data"):
                self._last_robot_state = str(msg.data)
            else:
                self._last_robot_state = repr(msg)
            if self._node is not None:
                self._node.get_logger().debug(f'behavior_stop robotState: {self._last_robot_state}')
        except Exception:
            if self._node is not None:
                self._node.get_logger().warning('behavior_stop robotState callback error')

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
        # If currently turning just do nothing
        if self._turning_time > time.time():
            return True, "turning", False

        if hasattr(self, '_last_robot_state_parsed') and self._last_robot_state_parsed:
            proximity_magnitude = getattr(self._last_robot_state_parsed, 'proximity_magnitude', 0.0)
            proximity_angle = getattr(self._last_robot_state_parsed, 'proximity_angle', 0.0)
        
        msg = self._Float32MultiArray()

        # Check if obstacle ahead and we need to turn
        if proximity_magnitude < self._obstacle_threshold:
            # Obstacle detected, initiate turn
            if proximity_angle < 180:
                msg.data = [-self._turn_speed, self._turn_speed]  # Turn left
            else:
                msg.data = [self._turn_speed, -self._turn_speed]  # Turn right
            self._turning_time = time.time() + random.uniform(0, getattr(self._last_robot_state_parsed, 'turn_duration')*5)  
            

            # Debug log
            if self._node is not None:
                self._node.get_logger().debug(
                    f"Turning: angle={proximity_angle}, magnitude={proximity_magnitude}, "
                    f"turn_duration={getattr(self._last_robot_state_parsed, 'turn_duration', 'N/A')}, "
                    f"turning_time={self._turning_time}"
                )


            self._pub.publish(msg)
        else:
            msg.data = [self._forward_speed, self._forward_speed]
            self._pub.publish(msg)

        return True, "published wheel speeds", False
    
    def reset(self) -> None:
        """Reset the behavior state and clean up resources."""
        # Reset behavior-specific state variables
        self._turning_time = time.time()  # Reset to current time (no turning)
        self._last_robot_state = None
        self._params = {}
        
        # Reset parsed robot state if it exists
        if hasattr(self, '_last_robot_state_parsed'):
            self._last_robot_state_parsed = None
        
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