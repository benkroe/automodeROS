#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from automode_interfaces.msg import RobotState
from rclpy.executors import ExternalShutdownException


class TurtleBot4ReferenceNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_reference_node')
        # Publisher for cmd_vel (movement)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publisher for RobotState
        self._robot_state_pub = self.create_publisher(RobotState, 'robotState', 10)
        # Subscriber for wheels_speed (from automode)
        self.create_subscription(Float32MultiArray, 'wheels_speed', self._wheels_speed_cb, 10)
        # Timer to publish RobotState periodically
        self.create_timer(0.1, self._publish_robot_state)  # 10 Hz

        # Example state variables (expand as needed)
        self.robot_id = 1
        self.neighbour_count = 0
        self.ground_black_floor = False
        self.proximity_magnitude = 0.0
        self.proximity_angle = 0.0
        self.light_magnitude = 0.0
        self.light_angle = 0.0

    def _wheels_speed_cb(self, msg: Float32MultiArray):
        # Convert wheel speeds to Twist for /cmd_vel
        # Assuming msg.data = [left_wheel, right_wheel]
        if len(msg.data) != 2:
            self.get_logger().warning("wheels_speed message must have 2 elements")
            return
        left, right = msg.data
        twist = Twist()
        # Simple differential drive conversion (tune these factors for your robot)
        twist.linear.x = (left + right) / 2.0
        twist.angular.z = (right - left) / 0.3  # 0.3 is wheel_base (meters), adjust as needed
        self._cmd_vel_pub.publish(twist)

    def _publish_robot_state(self):
        # Fill RobotState message (expand with real sensor data)
        msg = RobotState()
        msg.robot_id = self.robot_id
        msg.neighbour_count = self.neighbour_count
        msg.ground_black_floor = self.ground_black_floor
        msg.proximity_magnitude = self.proximity_magnitude
        msg.proximity_angle = self.proximity_angle
        msg.light_magnitude = self.light_magnitude
        msg.light_angle = self.light_angle
        
        self._robot_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4ReferenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Shutting down due to interrupt or external shutdown')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()