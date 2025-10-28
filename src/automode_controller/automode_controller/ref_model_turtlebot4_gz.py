#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from automode_interfaces.msg import RobotState
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray, Float32, String
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math
import numpy as np

PROXIMITY_MAX_RANGE = 2.0  # meters (reduced for IR realism)

class TurtleBot4ReferenceNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_reference_node')

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._robot_state_pub = self.create_publisher(RobotState, 'robotState', 10)

        # Subscriber for wheel commands
        self.create_subscription(Float32MultiArray, 'wheels_speed', self._wheels_speed_cb, 10)

        # Track latest IR readings
        self.latest_ir_vectors = {}

        # Subscribed IR topics (only the ones that work)
        ir_topics = {
            '/Turtlebot4/ir_intensity_front_center_right/point_cloud': 'front_center_right',
            '/Turtlebot4/ir_intensity_front_right/point_cloud': 'front_right',
            '/Turtlebot4/ir_intensity_left/point_cloud': 'left',
            '/Turtlebot4/ir_intensity_right/point_cloud': 'right',
            '/Turtlebot4/ir_intensity_side_left/point_cloud': 'side_left'
        }

        for topic, name in ir_topics.items():
            self.create_subscription(PointCloud2, topic, lambda msg, n=name: self._process_point_cloud(msg, n), 10)

        # Optional: Light, ground, and neighbour sensors
        self.create_subscription(Float32, 'light_sensor_front_left', self._light_fl_cb, 10)
        self.create_subscription(Float32, 'light_sensor_front_right', self._light_fr_cb, 10)
        self.create_subscription(Float32, 'light_sensor_back', self._light_back_cb, 10)
        self.create_subscription(String, 'ground_sensor_center', self._ground_sensor_cb, 10)
        self.create_subscription(String, 'neighbours_info', self._neighbours_cb, 10)

        # State variables
        self.robot_id = 1
        self.latest_floor_color = "gray"
        self.latest_light_fl = None
        self.latest_light_fr = None
        self.latest_light_back = None
        self.latest_neighbour_count = 0
        self.latest_attraction_angle = 0.0
        self.latest_wheels_speed = [0.0, 0.0]
        self.latest_cmd_vel = (0.0, 0.0)

        # Periodic publication (20 Hz)
        self.create_timer(0.05, self._publish_robot_state)

    # ----------------------------
    # IR point cloud processing
    # ----------------------------
    def _process_point_cloud(self, msg: PointCloud2, sensor_name: str):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        distances, angles = [], []

        for x, y, z in points:
            dist = math.sqrt(x**2 + y**2)
            if 0.01 < dist < PROXIMITY_MAX_RANGE:
                distances.append(dist)
                angles.append(math.degrees(math.atan2(y, x)))

        if distances:
            min_idx = np.argmin(distances)
            dist, angle = distances[min_idx], angles[min_idx]
            self.latest_ir_vectors[sensor_name] = [dist, angle]
        else:
            self.latest_ir_vectors[sensor_name] = [0.0, 0.0]

    # ----------------------------
    # Compute proximity vector
    # ----------------------------
    def compute_proximity(self):
        if not self.latest_ir_vectors:
            return 0.0, 0.0

        x_total, y_total = 0.0, 0.0
        for dist, angle_deg in self.latest_ir_vectors.values():
            angle_rad = math.radians(angle_deg)
            x_total += (PROXIMITY_MAX_RANGE - dist) * math.cos(angle_rad)
            y_total += (PROXIMITY_MAX_RANGE - dist) * math.sin(angle_rad)

        mag = math.sqrt(x_total**2 + y_total**2)
        angle = (math.degrees(math.atan2(y_total, x_total))) % 360
        return mag, angle

    # ----------------------------
    # Light, ground, neighbours
    # ----------------------------
    def _light_fl_cb(self, msg: Float32): self.latest_light_fl = msg.data
    def _light_fr_cb(self, msg: Float32): self.latest_light_fr = msg.data
    def _light_back_cb(self, msg: Float32): self.latest_light_back = msg.data
    def _ground_sensor_cb(self, msg: String): self.latest_floor_color = msg.data

    def _neighbours_cb(self, msg: String):
        try:
            angle_str, count_str = msg.data.split(',')
            self.latest_attraction_angle = float(angle_str)
            self.latest_neighbour_count = int(count_str)
        except Exception:
            pass

    # ----------------------------
    # Wheel speed → cmd_vel
    # ----------------------------
    def _wheels_speed_cb(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            return
        left, right = msg.data
        twist = Twist()
        twist.linear.x = (left + right) / 2.0
        twist.angular.z = (right - left) 
        self._cmd_vel_pub.publish(twist)
        self.latest_wheels_speed = [left, right]
        self.latest_cmd_vel = (twist.linear.x, twist.angular.z)

    # ----------------------------
    # Periodic state publisher
    # ----------------------------
    def _publish_robot_state(self):
        msg = RobotState()
        msg.robot_id = self.robot_id
        msg.floor_color = self.latest_floor_color
        msg.neighbour_count = self.latest_neighbour_count
        msg.attraction_angle = self.latest_attraction_angle
        msg.proximity_magnitude, msg.proximity_angle = self.compute_proximity()
        msg.light_magnitude, msg.light_angle = 0.0, 0.0  # simplified

        self.get_logger().info(
            f"[RobotState] ID={msg.robot_id}, "
            f"Floor='{msg.floor_color}', "
            f"Neighbours={msg.neighbour_count}, "
            f"AttractionAngle={msg.attraction_angle:.1f}, "
            f"Proximity=(Mag={msg.proximity_magnitude:.3f}, Angle={msg.proximity_angle:.1f}), "
            f"Light=(Mag={msg.light_magnitude:.3f}, Angle={msg.light_angle:.1f})"
            f"CmdVel= {self.latest_cmd_vel}"
        )

        self._robot_state_pub.publish(msg)

    # ----------------------------
    # Main entry point
    # ----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4ReferenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Shutting down TurtleBot4ReferenceNode')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()