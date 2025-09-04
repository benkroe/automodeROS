#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
from automode_interfaces.msg import RobotState
from rclpy.executors import ExternalShutdownException

# Use of lidar
from sensor_msgs.msg import LaserScan
import math
import numpy as np  
PROXIMITY_MAX_RANGE = 0.5 # for creating the virtual proximity sensors, max range of them


class TurtleBot4ReferenceNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_reference_node')
        # Publisher for cmd_vel (movement)
        self._cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        # Publisher for RobotState
        self._robot_state_pub = self.create_publisher(RobotState, 'robotState', 10)
        # Subscriber for wheels_speed (from automode)
        self.create_subscription(Float32MultiArray, 'wheels_speed', self._wheels_speed_cb, 10)
        # Subscribe for lidar scan (turtlebot4)
        self.create_subscription(LaserScan, '/scan', self._lidar_scan_cb, 10)
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


    def _lidar_scan_cb(self, msg):
        # Divide scan into 5 sectors: left, front-left, front, front-right, right
        num_ranges = len(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, num_ranges)
        ranges = np.array(msg.ranges)

            # Debug: print lidar angle info
        self.get_logger().info(
            f"Lidar angle_min={math.degrees(msg.angle_min):.1f}°, angle_max={math.degrees(msg.angle_max):.1f}°, num_ranges={num_ranges}"
        )

        sector_angles = [math.pi/2, math.pi/4, 0, -math.pi/4, -math.pi/2]
        sector_width = math.pi/8  # 22.5° width per sector

        sector_ranges = []
        for sa in sector_angles:
            mask = np.abs(angles - sa) < sector_width
            sector = ranges[mask]
            # Only consider readings within the proximity sensor's max range
            sector = sector[(sector > msg.range_min) & (sector < PROXIMITY_MAX_RANGE)]
            if len(sector) > 0:
                sector_ranges.append(np.min(sector))
            else:
                sector_ranges.append(PROXIMITY_MAX_RANGE)

        # Compute proximity vector (closer = stronger)
        vectors = []
        for r, a in zip(sector_ranges, sector_angles):
            strength = max(0, PROXIMITY_MAX_RANGE - r) / PROXIMITY_MAX_RANGE  # 0 (far) to 1 (close)
            vectors.append(np.array([strength * math.cos(a), strength * math.sin(a)]))

        prox_vec = np.sum(vectors, axis=0)
        prox_mag = np.linalg.norm(prox_vec)

        prox_ang = math.atan2(prox_vec[1], prox_vec[0])  # radians

        self.proximity_magnitude = float(prox_mag)
        if prox_mag == 0.0:
            self.proximity_angle = 0.0
        else:
            self.proximity_angle = (math.degrees(prox_ang) - 90) % 360

        # Debug log for proximity sensors
    
        # lidar_preview = np.array2string(ranges[:5], precision=2, separator=',') + " ... " + np.array2string(ranges[-5:], precision=2, separator=',')
        # self.get_logger().info(
        #     f"Lidar (first/last 5): {lidar_preview}\n"
        #     f"Proximity sensors (m): left={sector_ranges[0]:.2f}, front-left={sector_ranges[1]:.2f}, "
        #     f"front={sector_ranges[2]:.2f}, front-right={sector_ranges[3]:.2f}, right={sector_ranges[4]:.2f} | "
        #     f"Vector mag={self.proximity_magnitude:.2f}, angle={self.proximity_angle:.1f}°"
        # )

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