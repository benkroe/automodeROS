#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
from automode_interfaces.msg import RobotState
from rclpy.executors import ExternalShutdownException
#from rclpy.qos import qos_profile_sensor_data


# Use of lidar
from sensor_msgs.msg import LaserScan
import math
import numpy as np  
PROXIMITY_MAX_RANGE = 1.0 # for creating the virtual proximity sensors, max range of them


class TurtleBot4ReferenceNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_reference_node')
        # Publisher for cmd_vel (movement)
        self._cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        # Publisher for RobotState
        self._robot_state_pub = self.create_publisher(RobotState, 'robotState', 10)
        # Subscriber for wheels_speed (from automode)
        self.create_subscription(Float32MultiArray, 'wheels_speed', self._wheels_speed_cb, 10)
        # Subscribe for lidar scan (turtlebot4)
        self.create_subscription(LaserScan, 'scan', self._lidar_scan_cb, 10)
        # Subscribe to raw IR and light sensor topics
        self.create_subscription(Float32MultiArray, 'ir_intensities', self._ir_cb, 10)
        self.create_subscription(Float32, 'light_sensor_front_left', self._light_fl_cb, 10)
        self.create_subscription(Float32, 'light_sensor_front_right', self._light_fr_cb, 10)
        self.create_subscription(Float32, 'light_sensor_back', self._light_back_cb, 10)
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

        
        # Store latest sensor values
        self.latest_ir = None
        self.latest_light_fl = None
        self.latest_light_fr = None
        self.latest_light_back = None


    def _ir_cb(self, msg):
        self.latest_ir = msg.data

    def _light_fl_cb(self, msg):
        self.latest_light_fl = msg.data

    def _light_fr_cb(self, msg):
        self.latest_light_fr = msg.data

    def _light_back_cb(self, msg):
        self.latest_light_back = msg.data
    
    def compute_proximity(self):
        # Example: Use max IR value as magnitude, index as angle
        if self.latest_ir:
            mag = max(self.latest_ir)
            idx = self.latest_ir.index(mag)
            angle = idx * (360 / len(self.latest_ir))
            return mag, angle
        return 0.0, 0.0
    
    def compute_light(self):
        # Example: Vector sum based on sensor positions
        if None not in (self.latest_light_fl, self.latest_light_fr, self.latest_light_back):
            angles = [60, -60, 180]  # degrees
            values = [self.latest_light_fl, self.latest_light_fr, self.latest_light_back]
            x = sum(v * math.cos(math.radians(a)) for v, a in zip(values, angles))
            y = sum(v * math.sin(math.radians(a)) for v, a in zip(values, angles))
            mag = math.sqrt(x**2 + y**2)
            angle = (math.degrees(math.atan2(y, x))) % 360
            return mag, angle
        return 0.0, 0.0

    def _wheels_speed_cb(self, msg: Float32MultiArray):
        # Convert wheel speeds to TwistStamped for /cmd_vel
        # Assuming msg.data = [left_wheel, right_wheel]
        if len(msg.data) != 2:
            self.get_logger().warning("wheels_speed message must have 2 elements")
            return
        left, right = msg.data
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"
        twist_stamped.twist.linear.x = (left + right) / 2.0
        twist_stamped.twist.angular.z = (right - left) / 0.3  # 0.3 is wheel_base (meters), adjust as needed
        self._cmd_vel_pub.publish(twist_stamped)

    def _publish_robot_state(self):
        # Fill RobotState message (expand with real sensor data)
        msg = RobotState()
        msg.robot_id = self.robot_id
        msg.neighbour_count = self.neighbour_count
        msg.ground_black_floor = self.ground_black_floor
        msg.proximity_magnitude, msg.proximity_angle = self.compute_proximity()
        msg.light_magnitude, msg.light_angle = self.compute_light()

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