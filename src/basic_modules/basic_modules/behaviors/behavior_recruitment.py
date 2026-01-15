#!/usr/bin/env python3

from typing import Optional, Dict, Any, Tuple
from .behavior_interface import BehaviorBase
from automode_interfaces.msg import RobotState

import random
import time
import math

class Behavior(BehaviorBase):
    """Density-based recruitment and leaving behavior.
    
    Agents aggregate at targets through positive feedback:
    - Increasing local density raises recruitment probability
    - Increasing local density suppresses leaving probability
    """

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
        self._leaving_turn_time = None  # Timer for 180s-degree turn when leaving
        
        # Recruitment parameters (density-based)
        self._k_j = 0.5          # sigmoid steepness for recruitment
        self._theta_j = 2.0      # social activation threshold (neighbor count)
        self._q_max = 0.0001        # maximum leaving probability
        self._k_l = 0.8          # social stabilization strength
        self._target_approach_threshold = 0.55  # target_magnitude threshold to stop and recruit (close enough)
        
        # Movement speeds
        self._forward_speed = 0.3
        self._turn_speed = 1.0
        self._approach_turn_speed = 0.5  # slower turning when approaching target
        self._obstacle_threshold = 9.99
        self._leaving_speed = -0.5     # negative = backward/away

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "recruitment",
            "type": 9,
            "description": "Density-based recruitment: agents aggregate at targets through neighbor-dependent probabilities",
            "params": [
                {"name": "kj", "type": "float64", "required": False, "default": 0.5},    # sigmoid steepness
                {"name": "tj", "type": "float64", "required": False, "default": 2.0},    # social activation threshold
                {"name": "qmax", "type": "float64", "required": False, "default": 0.1},  # max leaving probability
                {"name": "kl", "type": "float64", "required": False, "default": 0.5},    # social stabilization strength
                {"name": "rwm", "type": "int", "required": False, "default": 100}        # random walk modulation
            ]
        }

    def set_params(self, params: Dict[str, Any]) -> None:
        if not isinstance(params, dict):
            raise TypeError("params must be a dict")
        self._params.update(params)
        
        try:
            self._k_j = float(params.get("kj", 0.5))
        except (TypeError, ValueError):
            self._k_j = 0.5
        
        try:
            self._theta_j = float(params.get("tj", 2.0))
        except (TypeError, ValueError):
            self._theta_j = 2.0
        
        try:
            self._q_max = float(params.get("qmax", 0.1))
        except (TypeError, ValueError):
            self._q_max = 0.1
        
        try:
            self._k_l = float(params.get("kl", 0.5))
        except (TypeError, ValueError):
            self._k_l = 0.5

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

    def _sigmoid(self, n: int) -> float:
        """Sigmoid recruitment probability: p_join(n) = 1 / (1 + e^(-k_j(n - θ_j)))"""
        exponent = self._k_j * (n - self._theta_j)
        # Clamp exponent to avoid overflow
        exponent = max(-500, min(500, exponent))
        return 1.0 / (1.0 + math.exp(-exponent))
    
    def _exponential_leaving(self, n: int) -> float:
        """Exponential leaving probability: p_leave(n) = q_max * e^(-k_l * n)"""
        return self._q_max * math.exp(-self._k_l * n)

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

        target_magnitude = 0.0
        target_position = 0.0
        neighbour_count = 0
        target_visible = False

        if self._last_robot_state:
            target_magnitude = self._last_robot_state.target_magnitude
            target_position = self._last_robot_state.target_position
            neighbour_count = self._last_robot_state.neighbour_count
            target_visible = target_magnitude > 0.0  # target visible if magnitude > 0

        # FSM transitions
        if self._state == "free":
            # FREE → RECRUITED: recruit if target is close enough
            if target_visible and target_magnitude >= self._target_approach_threshold:
                # Close enough to target, compute recruitment probability
                p_join = self._sigmoid(neighbour_count)
                if random.random() < p_join:
                    self._state = "recruited"
                    self._node.get_logger().info(
                        f"[Recruitment] FREE → RECRUITED: target close (mag={target_magnitude:.3f}), n={neighbour_count}, p_join={p_join:.3f}"
                    )
        
        elif self._state == "recruited":
            # RECRUITED → LEAVING: leave based on density
            if not target_visible:
                self._state = "free"
                self._node.get_logger().info(f"[Recruitment] RECRUITED → FREE: target lost")
            else:
                # Compute leaving probability (exponentially suppressed by neighbors)
                p_leave = self._exponential_leaving(neighbour_count)
                if random.random() < p_leave:
                    self._state = "leaving"
                    self._leaving_turn_time = time.time() + 1.5  # Turn for ~1.5 seconds (approx 180 degrees)
                    self._node.get_logger().info(
                        f"[Recruitment] RECRUITED → LEAVING: n={neighbour_count}, p_leave={p_leave:.3f}"
                    )
        
        elif self._state == "leaving":
            # LEAVING → FREE: return to exploring when target no longer visible
            if not target_visible:
                self._state = "free"
                self._node.get_logger().info(f"[Recruitment] LEAVING → FREE: target no longer visible")

        # Execute behavior based on current state
        msg = self._Float32MultiArray()

        if self._state == "free":
            proximity_magnitude = self._last_robot_state.proximity_magnitude if self._last_robot_state else 0.0
            proximity_angle = math.degrees(self._last_robot_state.proximity_angle) if self._last_robot_state else 0.0
            
            # If target is visible, move toward it
            if target_visible:
                # Proportional turning control based on target position
                # target_position: -1 (left) to +1 (right), 0 (center)
                # Scale turn gain to keep both wheels moving forward
                turn_gain = self._forward_speed * 0.5 * target_position  # Max ±0.15 when forward_speed=0.3
                
                # Move forward while turning toward target
                # Negative target_position (left) → turn left (left slower, right faster)
                # Positive target_position (right) → turn right (left faster, right slower)
                left_speed = self._forward_speed + turn_gain
                right_speed = self._forward_speed - turn_gain
                
                msg.data = [left_speed, right_speed]
                self._pub.publish(msg)
            else:
                # No target visible: random walk with obstacle avoidance
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
            # Remain at target (stop for simplicity; can add cohesion motion later)
            msg.data = [0.0, 0.0]
            self._pub.publish(msg)

        elif self._state == "leaving":
            # Move away from target: first turn 180 degrees, then move forward
            if self._leaving_turn_time is not None and time.time() < self._leaving_turn_time:
                # Still turning around
                msg.data = [self._turn_speed, -self._turn_speed]  # Turn in place
                self._pub.publish(msg)
            else:
                # Done turning, move forward (away from target)
                msg.data = [self._forward_speed, self._forward_speed]
                self._pub.publish(msg)

        return True, f"state={self._state}, target_mag={target_magnitude:.3f}, n={neighbour_count}", False

    def reset(self) -> None:
        """Reset behavior state."""
        self._state = "free"
        self._turning_time = time.time()
        self._leaving_turn_time = None
        self._last_robot_state = None
        self._params = {}
        self._pub = None
        self._sub = None
        self._Float32MultiArray = None
