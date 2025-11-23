# ...existing code...
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from automode_interfaces.msg import RobotState
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray, Float32, String
from irobot_create_msgs.msg import IrIntensityVector
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Illuminance
from irobot_create_msgs.msg import IrIntensityVector

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

        # subscribe to cliff intensity sensors, for ground color detection
        self.create_subscription(IrIntensityVector, '/Turtlebot4/cliff_intensity', self._cliff_intensity_cb, 10)

        # Optional: Light, ground, and neighbour sensors

        self.create_subscription(Illuminance, '/Turtlebot4/light_front_left', self._light_fl_cb, 10)
        self.create_subscription(Illuminance, '/Turtlebot4/light_front_right', self._light_fr_cb, 10)

        #self.create_subscription(String, 'ground_sensor_center', self._ground_sensor_cb, 10)
        self.create_subscription(String, 'neighbours_info', self._neighbours_cb, 10)

        # State variables
        self.robot_id = 1
        self.latest_floor_color = "gray"
        self.count_floor_color = 0
        self.latest_light_fl = None
        self.latest_light_fr = None
        self.latest_light_back = None
        self.latest_neighbour_count = 0
        self.latest_attraction_angle = 0.0
        self.latest_wheels_speed = [0.0, 0.0]
        self.latest_cmd_vel = (0.0, 0.0)

        # black: (969, 982), grey: (887, 901), white: (962, 975)
        self._floor_color_refs = {
            'black': (969, 969),
            'gray':  (887, 887),
            'white': (962, 962)
        }

        # Initialize entries so compute_proximity has consistent keys
        for name in self._ir_index_map:
            angle = self._sensor_angle_map.get(name, 0.0)
            # store as [distance_m, angle_deg]; default to PROXIMITY_MAX_RANGE -> treated as "no detection"
            self.latest_ir_vectors[name] = [PROXIMITY_MAX_RANGE, angle]

        # Periodic publication (20 Hz)
        self.create_timer(0.05, self._publish_robot_state)


    # ----------------------------
    # cliff_intensity callback -> compute floor colour
    # ----------------------------
    def _cliff_intensity_cb(self, msg: IrIntensityVector):
        # Use the two front cliff sensors (index 0: front_left, 1: front_right)
        try:
            readings = msg.readings
        except Exception:
            return

        if not readings or len(readings) < 2:
            return

        # Extract raw integer values (safety with getattr)
        v0 = int(getattr(readings[0], 'value', 0))
        v1 = int(getattr(readings[1], 'value', 0))

        # Choose the colour with minimal squared error to the reference pair
        best_color = None
        best_err = float('inf')
        for color, (r0, r1) in self._floor_color_refs.items():
            err = (v0 - r0) ** 2 + (v1 - r1) ** 2
            if err < best_err:
                best_err = err
                best_color = color

        if best_color is not None:
            self.count_floor_color += 1
            if self.count_floor_color > 10:
                self.latest_floor_color = best_color
                self.count_floor_color = 0
            # optional debug log
            self.get_logger().debug(f"Cliff intensity -> v0={v0} v1={v1} -> colour={best_color} (err={best_err})")



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
    # Compute light vector (uses latest_light_fl and latest_light_fr)
    # ----------------------------
    def compute_light_vector(self):
        if self.latest_light_fl is None or self.latest_light_fr is None:
            return 0.0, 0.0

        # Simple model: each sensor contributes a vector
        fl_intensity = float(self.latest_light_fl)
        fr_intensity = float(self.latest_light_fr)

        # Weights proportional to intensity
        x_total = fl_intensity * math.cos(math.radians(45.0)) + fr_intensity * math.cos(math.radians(-45.0))
        y_total = fl_intensity * math.sin(math.radians(45.0)) + fr_intensity * math.sin(math.radians(-45.0))

        mag = math.sqrt(x_total**2 + y_total**2)
        angle = (math.degrees(math.atan2(y_total, x_total))) % 360
        return mag, angle

    # ----------------------------
    # Light, ground, neighbours
    # ----------------------------
    def _light_fl_cb(self, msg: Illuminance): self.latest_light_fl = msg.illuminance
    def _light_fr_cb(self, msg: Illuminance): self.latest_light_fr = msg.illuminance

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
        msg.light_magnitude, msg.light_angle = self.compute_light_vector()

        # self.get_logger().info(
        #     f"[RobotState] ID={msg.robot_id}, "
        #     f"Floor='{msg.floor_color}', "
        #     f"Neighbours={msg.neighbour_count}, "
        #     f"AttractionAngle={msg.attraction_angle:.1f}, "
        #     f"Proximity=(Mag={msg.proximity_magnitude:.3f}, Angle={msg.proximity_angle:.1f}), "
        #     f"Light=(Mag={msg.light_magnitude:.3f}, Angle={msg.light_angle:.1f})"
        #     f"CmdVel= {self.latest_cmd_vel}"
        # )

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