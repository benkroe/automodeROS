#!/usr/bin/env python3

from typing import Optional, Dict, Any, Tuple
from .behavior_interface import BehaviorBase
from automode_interfaces.msg import RobotState

class Behavior(BehaviorBase):
    """Follow the detected target (red ball) with a simple P controller."""

    def __init__(self) -> None:
        self._node = None
        self._pub = None
        self._sub = None
        self._params: Dict[str, Any] = {}
        self._last_robot_state: Optional[RobotState] = None
        self._Float32MultiArray = None

        # Controller gains / limits
        self._kp_lin = 0.8          # P gain on target magnitude -> forward speed
        self._kp_ang = 0.2          # P gain on horizontal error (-1..1)
        self._base_speed = 0.7        # nominal forward speed baseline
        self._min_speed = 0.0
        self._max_speed = 1.0
        self._obstacle_threshold = 3.0  # if proximity magnitude above -> stop (reuse existing convention)

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "follow_target",
            "type": 6,
            "description": "Follow detected target using red_ball_position with a simple P controller.",
            "params": []
        }

    def set_params(self, params: Dict[str, Any]) -> None:
        self._params.update(params)

    def _robot_state_cb(self, msg) -> None:
        self._last_robot_state = msg

    def setup_communication(self, node) -> None:
        self._node = node
        try:
            from std_msgs.msg import Float32MultiArray  # type: ignore
        except ImportError:
            if self._node is not None:
                self._node.get_logger().warning('Required std_msgs not available; setup_communication aborted')
            return
        self._Float32MultiArray = Float32MultiArray
        self._pub = self._node.create_publisher(Float32MultiArray, 'wheels_speed', 10)
        self._sub = self._node.create_subscription(RobotState, 'robotState', self._robot_state_cb, 10)

    def execute_step(self) -> Tuple[bool, str, bool]:
        if self._last_robot_state is None:
            return True, "Waiting for robot state", False
        if self._pub is None or self._Float32MultiArray is None:
            return False, "Communication not set up", False

        rs = self._last_robot_state

        # Safety: stop if obstacle magnitude high (reuse existing convention)
        if rs.proximity_magnitude > self._obstacle_threshold:
            msg = self._Float32MultiArray()
            msg.data = [0.0, 0.0]
            self._pub.publish(msg)
            return True, f"Obstacle detected (prox_mag={rs.proximity_magnitude:.3f}) -> stop", False

        # If no target detected, stop
        if rs.red_ball_magnitude <= 0.0:
            msg = self._Float32MultiArray()
            msg.data = [0.0, 0.0]
            self._pub.publish(msg)
            return True, "No target detected -> stop", False

        # P-control on horizontal error
        error = float(rs.red_ball_position)  # -1 (left) to 1 (right)
        w = self._kp_ang * error

        v_raw = self._kp_lin * max(0.0, rs.red_ball_magnitude) + self._base_speed
        turn_scaler = max(0.2, 1.0 - 0.5 * abs(error))  
        v = v_raw * turn_scaler
        v = max(self._min_speed, min(self._max_speed, v))

        # Convert to wheel speeds (differential drive, units normalized)
        left = v + w
        right = v - w   

        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        msg = self._Float32MultiArray()
        msg.data = [left, right]
        self._pub.publish(msg)

        return True, f"Follow target: v={v:.2f}, w={w:.2f}, err={error:.3f}, wheels=({left:.2f},{right:.2f})", False

    def reset(self) -> None:
        self._last_robot_state = None
        self._params = {}
        self._pub = None
        self._sub = None
        self._Float32MultiArray = None
