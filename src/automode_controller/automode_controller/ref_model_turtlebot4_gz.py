# ...existing code...
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from automode_interfaces.msg import RobotState
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray, Float32, String
import math
import numpy as np

PROXIMITY_MAX_RANGE = 2.0  # meters (reduced for IR realism)

# New tunable IR parameters (matches new ir_intensity input semantics)
IR_MIN_RANGE = 0.02  # meters
IR_MAX_RANGE = 0.20  # meters

class TurtleBot4ReferenceNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_reference_node')

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._robot_state_pub = self.create_publisher(RobotState, 'robotState', 10)

        # Subscriber for wheel commands
        self.create_subscription(Float32MultiArray, 'wheels_speed', self._wheels_speed_cb, 10)

        # Track latest IR readings (keeps [estimated_distance_m, angle_deg] per sensor)
        self.latest_ir_vectors = {}

        # New: subscribe to consolidated ir_intensity topic (Float32MultiArray)
        # Index map:
        # 0 -> front_center_left
        # 1 -> front_center_right
        # 2 -> front_left
        # 3 -> front_right
        # 4 -> left
        # 5 -> right
        # 6 -> side_left
        self._ir_index_map = [
            'front_center_left',
            'front_center_right',
            'front_left',
            'front_right',
            'left',
            'right',
            'side_left'
        ]

        # Fixed sensor mounting angles (degrees, x-forward, +y left)
        self._sensor_angle_map = {
            'front_center_left':  10.0,
            'front_center_right': -10.0,
            'front_left':         30.0,
            'front_right':       -30.0,
            'left':               90.0,
            'right':             -90.0,
            'side_left':         135.0
        }

        # subscribe to consolidated ir_intensity only (compatibility pointcloud code removed)
        self.create_subscription(Float32MultiArray, '/Turtlebot4/ir_intensity', self._ir_intensity_cb, 10)

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

        # Initialize entries so compute_proximity has consistent keys
        for name in self._ir_index_map:
            angle = self._sensor_angle_map.get(name, 0.0)
            # store as [distance_m, angle_deg]; default to PROXIMITY_MAX_RANGE -> treated as "no detection"
            self.latest_ir_vectors[name] = [PROXIMITY_MAX_RANGE, angle]

        # Periodic publication (20 Hz)
        self.create_timer(0.05, self._publish_robot_state)

    # ----------------------------
    # ir_intensity array callback
    # ----------------------------
    def _ir_intensity_cb(self, msg: Float32MultiArray):
        # msg.data elements are normalized intensities in [0,1] for each sensor index.
        n = min(len(msg.data), len(self._ir_index_map))
        for i in range(n):
            name = self._ir_index_map[i]
            intensity = float(msg.data[i]) if msg.data[i] is not None else 0.0
            # clamp intensity
            intensity = max(0.0, min(1.0, intensity))
            # convert intensity -> estimated distance (meters)
            # treat zero intensity as "no detection" -> set to PROXIMITY_MAX_RANGE
            if intensity <= 0.0:
                est_dist = PROXIMITY_MAX_RANGE
            else:
                est_dist = IR_MIN_RANGE + (1.0 - intensity) * (IR_MAX_RANGE - IR_MIN_RANGE)
            angle = self._sensor_angle_map.get(name, 0.0)
            self.latest_ir_vectors[name] = [est_dist, angle]
            angle = self._sensor_angle_map.get(name, 0.0)
            self.latest_ir_vectors[name] = [est_dist, angle]

        # For any missing indices, set to max-range (no detection)
        for i in range(n, len(self._ir_index_map)):
            name = self._ir_index_map[i]
            angle = self._sensor_angle_map.get(name, 0.0)
            self.latest_ir_vectors[name] = [PROXIMITY_MAX_RANGE, angle]

    # ----------------------------
    # Compute proximity vector (uses estimated distances stored in latest_ir_vectors)
    # ----------------------------
    def compute_proximity(self):
        if not self.latest_ir_vectors:
            return 0.0, 0.0

        x_total, y_total = 0.0, 0.0
        for dist, angle_deg in self.latest_ir_vectors.values():
            # larger weight for closer obstacles: (PROXIMITY_MAX_RANGE - dist)
            weight = max(0.0, PROXIMITY_MAX_RANGE - dist)
            angle_rad = math.radians(angle_deg)
            x_total += weight * math.cos(angle_rad)
            y_total += weight * math.sin(angle_rad)

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
# ...existing code...