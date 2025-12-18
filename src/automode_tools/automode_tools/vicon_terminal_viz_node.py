#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vicon_receiver.msg import PositionList
import signal
import sys
import threading

class ViconTerminalVizNode(Node):
    def __init__(self):
        super().__init__('vicon_terminal_viz_node')
        self.create_subscription(PositionList, '/vicon/default/data', self.callback, 10)
        self._running = True
        self._executor = None
        self._input_thread = None
        self.get_logger().info('Vicon Terminal Viz Node started. Press Ctrl+C to exit.')

    def start_interactive_mode(self, executor):
        self._executor = executor
        self._input_thread = threading.Thread(target=self._wait_for_exit, daemon=True)
        self._input_thread.start()

    def _wait_for_exit(self):
        try:
            while self._running:
                signal.pause()
        except KeyboardInterrupt:
            self._running = False
            rclpy.shutdown()

    def callback(self, msg):
        print("\033c", end="")  # Clear terminal
        map_width = 40
        map_height = 20
        x_min, x_max = -2000, 1000
        y_min, y_max = -500, 2500
        grid = [['.' for _ in range(map_width)] for _ in range(map_height)]
        for pos in msg.positions:
            gx = int((pos.x_trans - x_min) / (x_max - x_min) * (map_width - 1))
            gy = int((pos.y_trans - y_min) / (y_max - y_min) * (map_height - 1))
            gy = map_height - 1 - gy
            if 0 <= gx < map_width and 0 <= gy < map_height:
                grid[gy][gx] = 'O'
        print("2D Arena Map (O = robot, . = empty)")
        for row in grid:
            print(''.join(row))

def main(args=None):
    rclpy.init(args=args)
    node = ViconTerminalVizNode()
    executor = rclpy.get_global_executor()
    node.start_interactive_mode(executor)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
