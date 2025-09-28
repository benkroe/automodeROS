#!/usr/bin/env python3

from typing import Optional, Dict, Any, Tuple
from .behavior_interface import BehaviorBase
from automode_interfaces.msg import RobotState

class Behavior(BehaviorBase):
    def __init__(self) -> None:
        self._node = None
        self._pub = None
        self._sub = None
        self._params: Dict[str, Any] = {}
        self._last_robot_state: Optional[str] = None
        self._Float32MultiArray = None

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "stop",
            "type": 1,
            "description": "Publish zero wheel speeds on 'wheels_speed'. Controller should call execute_step() periodically.",
            "params": []
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
        # self._node.get_logger().debug(f'Random number: {self._last_robot_state.random_number}')
        if self._node is None or self._pub is None:
            return False, "behavior_stop not initialised; call setup_communication(node) first"
        try:
            msg = self._Float32MultiArray()
            msg.data = [0.0, 0.0]
            self._pub.publish(msg)
            return True, "published [0.0, 0.0] to wheels_speed", False
        except Exception as e:
            if self._node is not None:
                self._node.get_logger().error(f'behavior_stop publish failed: {e}')
            return False, f'publish failed: {e}', False

    def reset(self) -> None:
        """Reset the behavior state."""
        self._last_robot_state = None
        self._params = {}
        self._pub = None
        self._sub = None
        self._Float32MultiArray = None