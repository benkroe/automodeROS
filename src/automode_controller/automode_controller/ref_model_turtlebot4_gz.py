#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
from automode_interfaces.msg import RobotState
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from std_msgs.msg import String


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
        # self.create_subscription(LaserScan, 'scan', self._lidar_scan_cb, 10)
        # Subscribe to raw IR and light sensor topics and ground
        self.create_subscription(Float32MultiArray, 'ir_intensities', self._ir_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, 'light_sensor_front_left', self._light_fl_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, 'light_sensor_front_right', self._light_fr_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, 'light_sensor_back', self._light_back_cb, qos_profile_sensor_data)
        self.create_subscription(String, 'ground_sensor_center', self._ground_sensor_cb, qos_profile_sensor_data)
        # self.create_subscription(String, 'random_number', self._random_number_cb, qos_profile_sensor_data)


        # Timer to publish RobotState periodically
        self.create_timer(0.05, self._publish_robot_state)  # 20 Hz

        # Example state variables (expand as needed)
        self.robot_id = 1
        self.neighbour_count = 0
        self.floor_color = "gray"
        self.proximity_magnitude = 0.0
        self.proximity_angle = 0.0
        self.light_magnitude = 0.0
        self.light_angle = 0.0
        # self.random_number = 4

        
        # Store latest sensor values
        self.latest_ir = None
        self.latest_light_fl = None
        self.latest_light_fr = None
        self.latest_light_back = None

        self.latest_wheels_speed = [0.0, 0.0]  
        self.latest_cmd_vel = (0.0, 0.0) 
        self.latest_floor_color = "gray"

        # self.latest_random_number = 4

        # prox smoothening
        self.proximity_mag_history = []
        self.proximity_angle_history = []
        self.proximity_smoothing_window = 1  # Number of samples for moving average
        self.proximity_angle_last = 0.0
        self.proximity_mag_last = 0.0
        self.proximity_hysteresis = 2.0  # Minimum change to update

        # neighbour count and direction
        self.create_subscription(String, 'neighbours_info', self._neighbours_cb, qos_profile_sensor_data)
        self.latest_neighbour_count = 0
        self.latest_attraction_angle = 0.0

    # def _neigbours_cb(self, msg):
    #     self.latest_random_number = msg.data

    def _ground_sensor_cb(self, msg):
        self.latest_floor_color = msg.data

    def _ir_cb(self, msg):
        self.latest_ir = msg.data

    def _light_fl_cb(self, msg):
        self.latest_light_fl = msg.data

    def _light_fr_cb(self, msg):
        self.latest_light_fr = msg.data

    def _light_back_cb(self, msg):
        self.latest_light_back = msg.data

    def _neighbours_cb(self, msg: String):
        try:
            angle_str, count_str = msg.data.split(',')
            self.latest_attraction_angle = float(angle_str)
            self.latest_neighbour_count = int(count_str)
        except Exception as e:
            self.get_logger().warning(f"Failed to parse neighbours_info: {msg.data} ({e})")
    
    def compute_proximity(self):
        if self.latest_ir:
            angles = np.array([0, -25, 25, -50, 65, -75, 75])  # match your sensor order!
            weights = np.array(self.latest_ir)
            total = np.sum(weights)
            if total > 0:
                avg_angle = np.sum(angles * weights) / total
                mag = total / len(self.latest_ir)
            else:
                avg_angle = 0.0
                mag = 0.0

            # Smoothing (moving average)
            self.proximity_mag_history.append(mag)
            self.proximity_angle_history.append(avg_angle)
            if len(self.proximity_mag_history) > self.proximity_smoothing_window:
                self.proximity_mag_history.pop(0)
                self.proximity_angle_history.pop(0)
            mag_smoothed = np.mean(self.proximity_mag_history)
            angle_smoothed = np.mean(self.proximity_angle_history)

            # Hysteresis: only update if change is significant
            if abs(mag_smoothed - self.proximity_mag_last) > self.proximity_hysteresis:
                self.proximity_mag_last = mag_smoothed
            if abs(angle_smoothed - self.proximity_angle_last) > self.proximity_hysteresis:
                self.proximity_angle_last = angle_smoothed

            return self.proximity_mag_last, self.proximity_angle_last
        return 0.0, 0.0
    
    def _ground_sensor_cb(self, msg: String):
        self.latest_ground_sensor = msg.data

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
        self.latest_wheels_speed = [left, right]
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"
        twist_stamped.twist.linear.x = (left + right) / 2.0
        twist_stamped.twist.angular.z = (right - left) / 0.3  # 0.3 is wheel_base (meters), adjust as needed
        self.latest_cmd_vel = (twist_stamped.twist.linear.x, twist_stamped.twist.angular.z)
        self._cmd_vel_pub.publish(twist_stamped)

    def _publish_robot_state(self):
        # Fill RobotState message (expand with real sensor data)
        msg = RobotState()
        msg.robot_id = self.robot_id
        msg.neighbour_count = self.latest_neighbour_count
        msg.attraction_angle = self.latest_attraction_angle
        msg.floor_color = self.latest_floor_color
        msg.proximity_magnitude, msg.proximity_angle = self.compute_proximity()
        msg.light_magnitude, msg.light_angle = self.compute_light()
        # msg.random_number = self.latest_random_number
        

        # Log a single summary message including wheel speeds
        self.get_logger().info(
            f"RobotState: id={msg.robot_id}, neighbours={msg.neighbour_count}, "
            f"attraction_angle={msg.attraction_angle:.2f}, floor_color={msg.floor_color}, "
            f"proximity(mag={msg.proximity_magnitude:.2f}, ang={msg.proximity_angle:.1f}), "
            f"light(mag={msg.light_magnitude:.2f}, ang={msg.light_angle:.1f}), "
            f"ground={self.latest_ground_sensor}, "
            f"wheels(received=[{self.latest_wheels_speed[0]:.2f}, {self.latest_wheels_speed[1]:.2f}], "
            f"published=[{self.latest_cmd_vel[0]:.2f}, {self.latest_cmd_vel[1]:.2f}])"
        )

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