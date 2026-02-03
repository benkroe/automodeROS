#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from automode_interfaces.msg import RobotState
from typing import Dict

class ForagingScorer(Node):
    def __init__(self):
        super().__init__('foraging_scorer')
        self.robot_namespaces = ['tb1', 'tb2', 'tb3', 'tb4']
        self.robot_states: Dict[str, Dict] = {}
        for ns in self.robot_namespaces:
            self.robot_states[ns] = {'has_marker': False, 'score': 0}
            self.create_subscription(
                RobotState,
                f'/{ns}/robotState',
                lambda msg, ns=ns: self.robot_state_callback(msg, ns),
                10
            )
        self.timer = self.create_timer(1.0, self.display_scores)

    def robot_state_callback(self, msg: RobotState, ns: str):
        state = self.robot_states[ns]
        color = msg.floor_color
        if color == 'black':
            if not state['has_marker']:
                state['has_marker'] = True
                self.get_logger().info(f'{ns}: Picked up marker')
        elif color == 'white':
            if state['has_marker']:
                state['score'] += 1
                state['has_marker'] = False
                self.get_logger().info(f'{ns}: Deposited marker, score now {state["score"]}')

    def display_scores(self):
        scores = {ns: self.robot_states[ns]['score'] for ns in self.robot_namespaces}
        self.get_logger().info(f'Scores: {scores}')

def main(args=None):
    rclpy.init(args=args)
    node = ForagingScorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()