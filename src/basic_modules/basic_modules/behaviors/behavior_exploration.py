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

        self._obstacle_threshold = 70  # Proximity magnitude threshold for obstacle detection
        self._forward_speed = 1.5       # Forward movement speed
        self._turn_speed = 0.6          # Turning speed (will calibrate later)

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
        if proximity_magnitude > self._obstacle_threshold:
            msg.data = [-1.5, -1.5]
            self._pub.publish(msg)
            time.sleep(0.2)
            # Obstacle detected, initiate turn
            if proximity_angle > 0:
                turn_direction = [self._turn_speed, -self._turn_speed]  # Turn left
            else:
                turn_direction = [-self._turn_speed, self._turn_speed]  # Turn right

            self._last_turn_direction = turn_direction
            rwm = int(self._params.get("rwm", 100))
            self._turning_time = time.time() + (random.uniform(0, rwm))/10
            msg.data = turn_direction
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
        self._last_robot_state = None
        
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