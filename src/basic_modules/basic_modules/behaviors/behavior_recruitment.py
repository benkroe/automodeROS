#!/usr/bin/env python3

from typing import Optional, Dict, Any, Tuple
from .behavior_interface import BehaviorBase
from automode_interfaces.msg import RobotState

import random
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
        self._turning_time = 0.0  # Will be set when node is available
        self._last_turn_direction = [0.0, 0.0]
        self._leaving_turn_time = None  # Timer for 180s-degree turn when leaving
        
        # Recruitment parameters (density-based)
        self._sigma = 0.5         # Gaussian spread (std dev, curve width)
        self._mu = 2.0            # Optimal neighbor count (peak probability)
        self._qmax_r = 0.1        # Maximum recruitment probability (peak of bell curve)
        self._qmax_l = 0.0001     # Maximum leaving probability
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
            "description": "Density-based recruitment with optimal group size: agents aggregate at targets with probability peaking at μ neighbors, self-regulating group size",
            "params": [
                {"name": "sigma", "type": "float64", "required": False, "default": 0.5},  # Gaussian spread (std dev, controls curve width)
                {"name": "mu", "type": "float64", "required": False, "default": 2.0},     # Optimal neighbor count (peak probability)
                {"name": "qmax_r", "type": "float64", "required": False, "default": 0.1}, # Maximum recruitment probability (peak of bell curve)
                {"name": "qmax_l", "type": "float64", "required": False, "default": 0.0001}, # Maximum leaving probability
                {"name": "kl", "type": "float64", "required": False, "default": 0.5},    # social stabilization strength
                {"name": "rwm", "type": "int", "required": False, "default": 100}        # random walk modulation
            ]
        }

    def set_params(self, params: Dict[str, Any]) -> None:
        if not isinstance(params, dict):
            raise TypeError("params must be a dict")
        self._params.update(params)
        
        try:
            self._sigma = float(params.get("sigma", 0.5))
        except (TypeError, ValueError):
            self._sigma = 0.5
        
        try:
            self._mu = float(params.get("mu", 2.0))
        except (TypeError, ValueError):
            self._mu = 2.0
        
        try:
            self._qmax_r = float(params.get("qmax_r", 0.1))
        except (TypeError, ValueError):
            self._qmax_r = 0.1
        
        try:
            self._qmax_l = float(params.get("qmax_l", 0.0001))
        except (TypeError, ValueError):
            self._qmax_l = 0.0001
        
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
        """Gaussian bell curve recruitment probability: p_join(n) = qmax_r * exp( - (n - μ)^2 / (2 σ^2) )"""
        return self._qmax_r * math.exp( - ((n - self._mu) ** 2) / (2 * self._sigma ** 2) )
    
    def _exponential_leaving(self, n: int) -> float:
        """Exponential leaving probability: p_leave(n) = qmax_l * e^(-k_l * n)"""
        return self._qmax_l * math.exp(-self._k_l * n)

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

        current_time = self._node.get_clock().now().nanoseconds / 1e9

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
                    self._leaving_turn_time = current_time + 1.5  # Turn for 1.5 seconds (approx 180 degrees)
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
                turn_gain = self._forward_speed * 0.5 * target_position  
                
                # Move forward while turning toward target
                # Negative target_position (left) → turn left (left slower, right faster)
                # Positive target_position (right) → turn right (left faster, right slower)
                left_speed = self._forward_speed + turn_gain
                right_speed = self._forward_speed - turn_gain
                
                msg.data = [left_speed, right_speed]
                self._pub.publish(msg)
            else:
                # No target visible: random walk with obstacle avoidance
                if self._turning_time > current_time:
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
                    self._turning_time = current_time + (random.uniform(0, rwm)) / 30
                    msg.data = turn_direction
                    self._pub.publish(msg)
                else:
                    # Move forward
                    msg.data = [self._forward_speed, self._forward_speed]
                    self._pub.publish(msg)

        elif self._state == "recruited":
            msg.data = [0.0, 0.0]
            self._pub.publish(msg)

        elif self._state == "leaving":
            # Move away from target: first turn 180 degrees, then move forward
            if self._leaving_turn_time is not None and current_time < self._leaving_turn_time:
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
        current_time = self._node.get_clock().now().nanoseconds / 1e9 if self._node else 0.0
        self._turning_time = current_time
        self._leaving_turn_time = None
        self._last_robot_state = None
        self._params = {}
        self._pub = None
        self._sub = None
        self._Float32MultiArray = None
