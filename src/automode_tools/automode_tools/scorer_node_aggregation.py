#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from automode_interfaces.msg import RobotState
from nav_msgs.msg import Odometry
from typing import Dict, Optional


class AggregationScorer(Node):
    def __init__(self):
        super().__init__('aggregation_scorer')
        self.split_axis = self.declare_parameter('split_axis', 'x').get_parameter_value().string_value
        self.split_value = self.declare_parameter('split_value', 0.0).get_parameter_value().double_value
        self.robot_namespaces = ['tb1', 'tb2', 'tb3', 'tb4']
        self.robot_states: Dict[str, Dict[str, Optional[float]]] = {}
        for ns in self.robot_namespaces:
            self.robot_states[ns] = {'x': None, 'y': None, 'floor_color': None}
            self.create_subscription(
                RobotState,
                f'/{ns}/robotState',
                lambda msg, ns=ns: self.robot_state_callback(msg, ns),
                10
            )
            self.create_subscription(
                Odometry,
                f'/{ns}/odom',
                lambda msg, ns=ns: self.odom_callback(msg, ns),
                10
            )
        self.timer = self.create_timer(1.0, self.display_scores)

    def robot_state_callback(self, msg: RobotState, ns: str):
        self.robot_states[ns]['floor_color'] = msg.floor_color

    def odom_callback(self, msg: Odometry, ns: str):
        self.robot_states[ns]['x'] = msg.pose.pose.position.x
        self.robot_states[ns]['y'] = msg.pose.pose.position.y

    def display_scores(self):
        total = len(self.robot_namespaces)
        upper = 0
        lower = 0
        for ns in self.robot_namespaces:
            state = self.robot_states[ns]
            axis_value = state[self.split_axis] if self.split_axis in state else None
            if state['floor_color'] == 'black' and axis_value is not None:
                if axis_value >= self.split_value:
                    upper += 1
                else:
                    lower += 1
        score = max(upper / total, lower / total) if total > 0 else 0.0
        self.get_logger().info(
            f'AGGREGATION_SCORE: {score:.3f} upper={upper} lower={lower} total={total} '
            f'axis={self.split_axis} split={self.split_value:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AggregationScorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
