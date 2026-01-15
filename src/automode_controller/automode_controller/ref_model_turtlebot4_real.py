import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from automode_interfaces.msg import RobotState
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray, Float32, String
from irobot_create_msgs.msg import IrIntensityVector
from sensor_msgs.msg import Illuminance
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from vicon_receiver.msg import Position
from vicon_receiver.msg import PositionList


import math
import numpy as np

ROBOT_ID = 1  # Default robot ID (change if used)
SUBJECT_NAME = f"turtlebot4_11"

PROXIMITY_MAX_RANGE = 2.0  # meters (reduced for IR realism)

# New tunable IR parameters (matches new ir_intensity input semantics)
IR_MIN_RANGE = 0.02  # meters
IR_MAX_RANGE = 0.20  # meters
IR_DETECTION_THRESHOLD = 0.05
IR_INTENSITY_MAX = 1725.0 
NEIGHBOUR_DETECTION_RANGE=1000

# Arena definition
# Arena points
# x=663, y=1596 -- door -right
# x=-1368, y=2036 -- door -left
# x=-1569, y=-418 --window -left
# x=470, y=-124 --window -right (kind of)
class Arena:
    def __init__(self):
        # Arena edge points (approximate polygon)
        self.arena_points = [
            (663, 1596),    # door - right
            (-1368, 2036),  # door - left
            (-1569, -418),  # window - left
            (470, -124)     # window - right
        ]
        # Define color regions (example: white, black, gray)
        # For simplicity, use bounding boxes for each region
        self.white_box = (0, 600, 0, 1600)    # Example: right upper area
        self.black_box = (-1600, -1300, -500, 2100)  # Example: left side
        # The rest is gray

    def get_color(self, x, y):
        # White region (example: right upper area)
        xw_min, xw_max, yw_min, yw_max = self.white_box
        if xw_min <= x <= xw_max and yw_min <= y <= yw_max:
            return "white"
        # Black region (example: left side)
        xb_min, xb_max, yb_min, yb_max = self.black_box
        if xb_min <= x <= xb_max and yb_min <= y <= yb_max:
            return "black"
        # Otherwise gray
        return "gray"
    


class TurtleBot4ReferenceNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_reference_node_gz')

        self.arena = Arena()
        self.robots = {} 

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.light_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )




        # Publishers
        self._cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 5)
        self._robot_state_pub = self.create_publisher(RobotState, 'robotState', 10)

        # Subscriber for wheel commands
        self.create_subscription(Float32MultiArray, 'wheels_speed', self._wheels_speed_cb, 10)    
        # subscribe to consolidated ir_intensity only (compatibility pointcloud code removed)
        self.create_subscription(IrIntensityVector, 'ir_intensity', self._ir_intensities_cb, self.qos)

        # subscribtion for position from tracking system. for now only turt11
        self.vicon_sub = self.create_subscription(PositionList, '/vicon/default/data', self.vicon_callback, 10)

        # Subscribe to the three light sensor topics (Float32)
        self.create_subscription(Float32, 'light_sensor_front_left', self._light_fl_cb, self.light_qos)
        self.create_subscription(Float32, 'light_sensor_front_right', self._light_fr_cb, self.light_qos)
        self.create_subscription(Float32, 'light_sensor_back', self._light_back_cb, self.light_qos)


        # Track latest IR readings (keeps [estimated_distance_m, angle] per sensor)
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
            'side_left',
            'left',
            'front_left',
            'front',
            'front_right',
            'right',
            'side_right'
        ]

        # Fixed sensor mounting angles (degrees, x-forward, +y left)
        self._sensor_angle_map = {
            'side_left':            math.radians(-65.3),
            'left':                 math.radians(-38.0),
            'front_left':         math.radians(-20.0),
            'front':                math.radians(-3.0),
            'front_right':          math.radians(14.25),
            'right':              math.radians(34.0),
            'side_right':          math.radians(65.3)
        }


        self._light_sensor_angle_map = {
            'light_sensor_front_left': math.radians(-45.0),
            'light_sensor_front_right': math.radians(45.0),
            'light_sensor_back': math.radians(180.0)
        }


        # State variables
        self.robot_id = ROBOT_ID
        self.latest_floor_color = "gray"
        self.count_floor_color = 0
        self.latest_light_fl = 0
        self.latest_light_fr = 0
        self.latest_light_back = 0
        self.latest_neighbour_count = 0
        self.latest_attraction_angle = 0.0
        self.latest_wheels_speed = [0.0, 0.0]
        self.latest_cmd_vel = (0.0, 0.0)


        # Initialize entries so compute_proximity has consistent keys
        for name in self._ir_index_map:
            angle = self._sensor_angle_map.get(name, 0.0)
            # store as [distance_m, angle]; default to PROXIMITY_MAX_RANGE -> treated as "no detection"
            self.latest_ir_vectors[name] = [PROXIMITY_MAX_RANGE, angle]

        # Periodic publication (20 Hz)
        self.create_timer(0.05, self._publish_robot_state)


    def vicon_callback(self, msg):
        # msg is PositionList
        for pos in msg.positions:
            x = pos.x_trans
            y = pos.y_trans
            theta = pos.z_rot_euler  # Or use quaternion if you prefer
            self.robots[pos.subject_name] = (x, y, theta)
            # If this is our robot, update floor color
            if pos.subject_name == SUBJECT_NAME:  # Or use self.subject_name if configurable
                self.latest_floor_color = self.arena.get_color(x, y)

    def _ir_intensities_cb(self, msg: IrIntensityVector):
        self._publish_robot_state()
        readings = getattr(msg, 'readings', None)
        if not readings:
            return

        # Extract normalized intensities [0..1] from readings
        intensities = []
        for r in readings:
            val = getattr(r, 'value', None)
            if val is None:
                val = getattr(r, 'intensity', 0.0)
            try:
                # Normalize if value is integer (0..1725), else use as-is
                f = float(val)
                if isinstance(val, int) or (isinstance(f, float) and f > 1.0):
                    f = f / IR_INTENSITY_MAX
            except Exception:
                f = 0.0
            intensities.append(max(0.0, min(1.0, f)))

        # Map intensities -> estimated distance per configured index map
        n = min(len(intensities), len(self._ir_index_map))
        for i in range(n):
            name = self._ir_index_map[i]
            intensity = intensities[i]

            if intensity <= IR_DETECTION_THRESHOLD:
                est_dist = PROXIMITY_MAX_RANGE
            else:
                est_dist = IR_MIN_RANGE + (1.0 - intensity) * (IR_MAX_RANGE - IR_MIN_RANGE)
            angle = self._sensor_angle_map.get(name, 0.0)
            self.latest_ir_vectors[name] = [est_dist, angle]

        # For any missing indices, set to max-range (no detection)
        for i in range(n, len(self._ir_index_map)):
            name = self._ir_index_map[i]
            angle = self._sensor_angle_map.get(name, 0.0)
            self.latest_ir_vectors[name] = [PROXIMITY_MAX_RANGE, angle]

    def compute_proximity(self):
        if not self.latest_ir_vectors:
            return 0.0, 0.0

        x_total, y_total = 0.0, 0.0
        for dist, angle in self.latest_ir_vectors.values():
            # larger weight for closer obstacles: (PROXIMITY_MAX_RANGE - dist)
            weight = max(0.0, PROXIMITY_MAX_RANGE - dist)
            angle_rad = math.radians(angle)
            x_total += weight * math.cos(angle_rad)
            y_total += weight * math.sin(angle_rad)

        mag = math.sqrt(x_total**2 + y_total**2)
        angle_rad = math.atan2(y_total, x_total)

        # Normalize proximity magnitude to [0, 1]
        mag = max(0.0, min(1.0, mag / (len(self.latest_ir_vectors) * PROXIMITY_MAX_RANGE)))
        return mag, angle_rad

    def compute_neighbours(self):
        my_pose = self.robots.get(SUBJECT_NAME, None)
        if my_pose is None:
            return 0, 0.0
        my_x, my_y, my_theta = my_pose
        count = 0
        sum_dx = 0.0
        sum_dy = 0.0
        for name, (x, y, theta) in self.robots.items():
            if name == SUBJECT_NAME:
                continue
            dx = x - my_x
            dy = y - my_y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist <= NEIGHBOUR_DETECTION_RANGE:
                count += 1
                sum_dx += dx
                sum_dy += dy
        if count > 0:
            angle = math.atan2(sum_dy, sum_dx) - my_theta
        else:
            angle = 0.0
        return count, angle

    def compute_light_vector(self):
        # Collect the latest sensor readings
        sensor_values = {
            'light_sensor_front_left': getattr(self, 'latest_light_fl', 0.0),
            'light_sensor_front_right': getattr(self, 'latest_light_fr', 0.0),
            'light_sensor_back': getattr(self, 'latest_light_back', 0.0)
        }

        x_total, y_total = 0.0, 0.0
        for name, intensity in sensor_values.items():
            angle = self._light_sensor_angle_map.get(name, 0.0)
            x_total += intensity * math.cos(angle)
            y_total += intensity * math.sin(angle)

        # Compute raw magnitude and angle
        mag_raw = math.sqrt(x_total**2 + y_total**2)
        angle_rad = math.atan2(y_total, x_total)

        # Normalize magnitude to [0, 1] using max possible sum of sensor values
        max_possible = sum(abs(v) for v in sensor_values.values()) or 1.0
        mag = max(0.0, min(1.0, mag_raw / max_possible))

        return mag, angle_rad

    def _light_fl_cb(self, msg: Float32):
        self.latest_light_fl = msg.data

    def _light_fr_cb(self, msg: Float32):
        self.latest_light_fr = msg.data

    def _light_back_cb(self, msg: Float32):
        self.latest_light_back = msg.data

    def _neighbours_cb(self, msg: String):
        try:
            angle_str, count_str = msg.data.split(',')
            self.latest_attraction_angle = math.radians(float(angle_str))  # Convert to radians
            self.latest_neighbour_count = int(count_str)
        except Exception:
            pass

    def _wheels_speed_cb(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            return
        left, right = msg.data

        twist = Twist()
        twist.linear.x = (left + right) / 2.0
        twist.angular.z = (right - left) / 0.3

        scale = max(abs(twist.linear.x), abs(twist.angular.z))
        if scale > 1.0:
            twist.linear.x /= scale
            twist.angular.z /= scale
            
        # self._cmd_vel_pub.publish(twist)
        # new with TwistStamped
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = twist
        #self._cmd_vel_pub.publish(stamped)


        self.latest_wheels_speed = [left, right]
        self.latest_cmd_vel = (twist.linear.x, twist.angular.z)

    def _publish_robot_state(self):
        msg = RobotState()
        msg.robot_id = self.robot_id
        msg.floor_color = self.latest_floor_color
        count, angle = self.compute_neighbours()
        msg.neighbour_count = count
        msg.attraction_angle = angle
        msg.proximity_magnitude, msg.proximity_angle = self.compute_proximity()
        msg.light_magnitude, msg.light_angle = self.compute_light_vector()
        msg.target_magnitude = 0.0
        msg.target_position = 0.0

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