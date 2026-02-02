#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import os
import signal
import time

class TimerShutdownNode(Node):
    def __init__(self):
        super().__init__('timer_shutdown')
        self.subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )
        self.start_time = time.time()
        self.get_logger().info('Timer shutdown node started, monitoring sim time for 10 minutes')

    def clock_callback(self, msg):
        if msg.clock.sec >= 300:  # 5 minutes = 300 seconds
            end_time = time.time()
            real_time_elapsed = end_time - self.start_time
            sim_time_elapsed = 300.0
            rtf = sim_time_elapsed / real_time_elapsed
            self.get_logger().info(f'Simulation time reached 10 minutes. Real time elapsed: {real_time_elapsed:.2f}s, RTF: {rtf:.2f}')
            # Try to stop the launch process, then shutdown this node
            try:
                os.kill(os.getppid(), signal.SIGINT)
            except Exception as e:
                self.get_logger().warning(f'Failed to signal parent process: {e}')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TimerShutdownNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()