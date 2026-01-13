import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from automode_interfaces.msg import RobotState
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from cv_bridge import CvBridge
import cv2
import numpy as np

import math

ROBOT_ID = 1  # Default robot ID (change if used)

ROBOT_NAMES = ["tb1", "tb2", "tb3", "tb4"]
WHEELBASE = 0.3  # meters (distance between left and right wheels)

PROXIMITY_MAX_RANGE = 2.0  # meters (reduced for IR realism)
PROXIMITY_RANGE_MIN = 0.01  # meters
PROXIMITY_RANGE_MAX = 0.10  # meters
PROXIMITY_INF_THRESHOLD = 10.0  # Treat anything >= this as inf/no detection

# Neighbor detection configuration
NEIGHBOR_DETECTION_RANGE = 2.0  # meters

# Ground sensor configuration (mission world)
GROUND_SENSOR_BASE_FRAME = 'mission'
WHITE_X_MIN = -3
WHITE_X_MAX = 3
WHITE_Y_MIN = 1
WHITE_Y_MAX = 3
BLACK1_X = -1.5
BLACK1_Y = -1.0
BLACK1_RADIUS = 0.4
BLACK2_X = 1.5
BLACK2_Y = -1.0
BLACK2_RADIUS = 0.4

# Light sensor configuration (mission world)
LIGHT_SOURCE_X = 0.0
LIGHT_SOURCE_Y = 3.5
LIGHT_SENSOR_OFFSET = 0.1  # meters from robot center
LIGHT_SENSOR_FL_ANGLE = math.radians(45.0)   # front left
LIGHT_SENSOR_FR_ANGLE = math.radians(-45.0)  # front right
LIGHT_SENSOR_BACK_ANGLE = math.radians(180.0)  # back 

class TurtleBot4ReferenceNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_reference_node_gz')

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        self._cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 5)
        self._robot_state_pub = self.create_publisher(RobotState, 'robotState', 10)

        # TF2 buffer and listener for ground sensor
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.namespace = self.get_namespace().strip('/')
        self.other_robot_positions = {}
        self.self_robot_name = self.namespace if self.namespace else f"tb{ROBOT_ID}"
        self.cached_robot_yaw = None
        
        # Subscribe to ground truth odometry
        self.ground_truth_position = None
        self.ground_truth_orientation = None
        self.create_subscription(Odometry, 'ground_truth_odom', self._ground_truth_odom_cb, 10)

        # Subscribe to ground truth odometry for all robots (tb1-tb4)
        for robot_name in ROBOT_NAMES:
            topic = f'/{robot_name}/ground_truth_odom'
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, rn=robot_name: self._multi_robot_odom_cb(msg, rn),
                10
            )
        
        # Subscriber for wheel commands
        self.create_subscription(Float32MultiArray, 'wheels_speed', self._wheels_speed_cb, 10)
        
        # Subscribe to 7 proximity sensor topics
        for i in range(1, 8):
            topic = f'proximity/proximity_{i}'
            self.create_subscription(LaserScan, topic, 
                                   lambda msg, sensor_id=i: self._proximity_cb(msg, sensor_id), 
                                   self.qos)

        # Ground sensor computation timer (0.2s = 5 Hz)
        self.create_timer(0.2, self._update_ground_sensor)

        # Light sensor computation timer (10 Hz, same as external node)
        self.create_timer(0.01, self._update_light_sensors)

        # Track latest IR readings (keeps [estimated_distance_m, angle] per sensor)
        self.latest_ir_vectors = {}

        # Fixed sensor mounting angles (degrees from xacro definitions)
        # Proximity sensors are numbered 1-7 with specific angles
        self._sensor_angle_map = {
            1: math.radians(65.3),  # left
            2: math.radians(38.0),  # lmid left
            3: math.radians(20.0),  # middle left
            4: math.radians(3.0),   # center
            5: math.radians(-14.25),  # middle right
            6: math.radians(-34.0),   # mid right
            7: math.radians(-65.3)    # right
        }

        self.latest_light_fl = None
        self.latest_light_fr = None
        self.latest_light_back = None


        # State variables
        self.robot_id = ROBOT_ID
        self.latest_floor_color = "gray"
        self.count_floor_color = 0
        self.latest_light_fl = None
        self.latest_light_fr = None
        self.latest_light_back = None
        self.latest_neighbour_count = 0
        self.latest_attraction_angle = 0.0
        self.latest_wheels_speed = [0.0, 0.0]
        self.latest_cmd_vel = (0.0, 0.0)
        
        # Red ball detection variables
        self.bridge = CvBridge()
        self.latest_camera_image = None
        self.red_ball_magnitude = 0.0  # Amount of red pixels (normalized)
        self.red_ball_position = 0.0   # Position: -1.0 (left) to 1.0 (right), 0.0 (center)


        # Initialize entries so compute_proximity has consistent keys
        for sensor_id in range(1, 8):
            angle = self._sensor_angle_map.get(sensor_id, 0.0)
            self.latest_ir_vectors[sensor_id] = [PROXIMITY_MAX_RANGE, angle]

        # Subscribe to camera for red ball detection
        self.create_subscription(Image, '/rgbd_camera/image', self._camera_cb, 10)
        
        # Red ball detection timer (every 0.5 seconds)
        self.create_timer(0.5, self._process_red_ball_detection)

        # Periodic publication (20 Hz)
        self.create_timer(0.05, self._publish_robot_state)


    def _ground_truth_odom_cb(self, msg: Odometry):
        self.ground_truth_position = msg.pose.pose.position
        self.ground_truth_orientation = msg.pose.pose.orientation
        self.cached_robot_yaw = self._compute_yaw_from_quaternion(self.ground_truth_orientation)

    def _multi_robot_odom_cb(self, msg: Odometry, robot_name: str):
        # Store positions from other robots for neighbour calculations
        if robot_name == self.self_robot_name:
            return

        position = msg.pose.pose.position
        self.other_robot_positions[robot_name] = (position.x, position.y)

    def _update_ground_sensor(self):
        if self.ground_truth_position is None:
            self.get_logger().info("Waiting for ground truth position from /world/pose")
            return

        x = self.ground_truth_position.x
        y = self.ground_truth_position.y

        # Mission world logic
        if (WHITE_X_MIN <= x <= WHITE_X_MAX and
            WHITE_Y_MIN <= y <= WHITE_Y_MAX):
            color = "white"
        elif ((x - BLACK1_X)**2 + (y - BLACK1_Y)**2 <= BLACK1_RADIUS**2):
            color = "black"
        elif ((x - BLACK2_X)**2 + (y - BLACK2_Y)**2 <= BLACK2_RADIUS**2):
            color = "black"
        else:
            color = "gray"

        self.latest_floor_color = color

    def _update_light_sensors(self):

        if self.ground_truth_position is None or self.cached_robot_yaw is None:
            return

        robot_yaw = self.cached_robot_yaw

        robot_x = self.ground_truth_position.x
        robot_y = self.ground_truth_position.y

        dx = LIGHT_SOURCE_X - robot_x
        dy = LIGHT_SOURCE_Y - robot_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance == 0:
            self.latest_light_fl = 0.0
            self.latest_light_fr = 0.0
            self.latest_light_back = 0.0
            return

        # Angle from robot to light in world frame
        angle_to_light_world = math.atan2(dy, dx)
        
        angle_to_light_robot = angle_to_light_world - robot_yaw

        while angle_to_light_robot > math.pi:
            angle_to_light_robot -= 2 * math.pi
        while angle_to_light_robot < -math.pi:
            angle_to_light_robot += 2 * math.pi

        # Base intensity from distance (inverse square law)
        base_intensity = 1.0 / (distance ** 2)

        sensors = [
            ('fl', LIGHT_SENSOR_FL_ANGLE),
            ('fr', LIGHT_SENSOR_FR_ANGLE),
            ('back', LIGHT_SENSOR_BACK_ANGLE)
        ]

        for sensor_name, sensor_facing_angle in sensors:
            # Angular difference between sensor facing direction and light direction
            angle_diff = angle_to_light_robot - sensor_facing_angle
            # Normalize to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            directional_factor = max(0.0, math.cos(angle_diff))
            
            sensor_value = base_intensity * directional_factor

            if sensor_name == 'fl':
                self.latest_light_fl = sensor_value
            elif sensor_name == 'fr':
                self.latest_light_fr = sensor_value
            elif sensor_name == 'back':
                self.latest_light_back = sensor_value

    def _compute_yaw_from_quaternion(self, quaternion):
        q = quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _proximity_cb(self, msg: LaserScan, sensor_id: int):
        if not msg.ranges or len(msg.ranges) == 0:
            # No ranges available, treat as no detection
            angle = self._sensor_angle_map.get(sensor_id, 0.0)
            self.latest_ir_vectors[sensor_id] = [PROXIMITY_MAX_RANGE, angle]
            return
        
        # Find minimum valid range (ignore inf)
        min_range = None
        for r in msg.ranges:
            if math.isfinite(r) and r < PROXIMITY_INF_THRESHOLD:
                if min_range is None or r < min_range:
                    min_range = r
        
        # If no valid range found, treat as no detection
        if min_range is None:
            min_range = PROXIMITY_MAX_RANGE
        
        angle = self._sensor_angle_map.get(sensor_id, 0.0)
        self.latest_ir_vectors[sensor_id] = [min_range, angle]

    def _compute_proximity(self):
        if not self.latest_ir_vectors:
            return 0.0, 0.0

        x_total, y_total = 0.0, 0.0
        for dist, angle in self.latest_ir_vectors.values():
            # larger weight for closer obstacles: (PROXIMITY_MAX_RANGE - dist)
            weight = max(0.0, PROXIMITY_MAX_RANGE - dist)
            # angle is already in radians from _sensor_angle_map
            x_total += weight * math.cos(angle)
            y_total += weight * math.sin(angle)

        mag = math.sqrt(x_total**2 + y_total**2)
        angle_rad = math.atan2(y_total, x_total)

        # Normalize proximity magnitude to [0, 1]
        mag = max(0.0, min(1.0, mag / (len(self.latest_ir_vectors) * PROXIMITY_MAX_RANGE)))
        return mag, angle_rad

    def _compute_light_vector(self):
        # Use all three sensors if available
        if self.latest_light_fl is None or self.latest_light_fr is None or self.latest_light_back is None:
            return 0.0, 0.0

        # Sensor arrangement: front_left (+45°), front_right (-45°), back (180°)
        fl = float(self.latest_light_fl)
        fr = float(self.latest_light_fr)
        back = float(self.latest_light_back)

        x_total = fl * math.cos(LIGHT_SENSOR_FL_ANGLE) + fr * math.cos(LIGHT_SENSOR_FR_ANGLE) + back * math.cos(LIGHT_SENSOR_BACK_ANGLE)
        y_total = fl * math.sin(LIGHT_SENSOR_FL_ANGLE) + fr * math.sin(LIGHT_SENSOR_FR_ANGLE) + back * math.sin(LIGHT_SENSOR_BACK_ANGLE)

        # Magnitude is the sum of sensor readings
        mag = fl + fr + back
        
        # Angle is determined by the weighted vector sum
        angle_rad = math.atan2(y_total, x_total)
        return mag, angle_rad

    def _neighbours_cb(self, msg=None):
        # Compute neighbour stats from stored positions instead of subscribing to an aggregated message
        if self.ground_truth_position is None:
            self.latest_neighbour_count = 0
            self.latest_attraction_angle = 0.0
            return

        robot_yaw = self.cached_robot_yaw

        total_x, total_y = 0.0, 0.0
        count = 0

        for robot_name, (x, y) in self.other_robot_positions.items():
            if robot_name == self.self_robot_name:
                continue

            dx = x - self.ground_truth_position.x
            dy = y - self.ground_truth_position.y

            if dx == 0.0 and dy == 0.0:
                continue

            distance = math.sqrt(dx**2 + dy**2)
            if distance > NEIGHBOR_DETECTION_RANGE:
                continue

            total_x += dx
            total_y += dy
            count += 1

        self.latest_neighbour_count = count

        if count > 0:
            angle_world = math.atan2(total_y, total_x)
            if robot_yaw is not None:
                angle_robot = angle_world - robot_yaw
                while angle_robot > math.pi:
                    angle_robot -= 2 * math.pi
                while angle_robot < -math.pi:
                    angle_robot += 2 * math.pi
            else:
                angle_robot = angle_world
            self.latest_attraction_angle = angle_robot
        else:
            self.latest_attraction_angle = 0.0

    def _camera_cb(self, msg: Image):
        self.latest_camera_image = msg

    def _process_red_ball_detection(self):
        # Only every 0.5 seconds
        if self.latest_camera_image is None:
            self.get_logger().info("[Red Ball] Waiting for camera image")
            return
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_camera_image, desired_encoding='bgr8')
            
            # Convert BGR to HSV for better red detection
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define red color range in HSV
            # Red wraps around in HSV, so we need two ranges
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            
            # Create masks for red color
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            
            # Count red pixels
            red_pixel_count = cv2.countNonZero(mask)
            total_pixels = cv_image.shape[0] * cv_image.shape[1]
            
            # Calculate magnitude (normalized by total pixels)
            self.red_ball_magnitude = red_pixel_count / total_pixels
            
            # Calculate position (center of red pixels)
            if red_pixel_count > 0:
                # Find moments to get center of mass
                moments = cv2.moments(mask)
                if moments['m00'] > 0:
                    cx = moments['m10'] / moments['m00']
                    image_width = cv_image.shape[1]
                    
                    # Normalize position: -1.0 (left) to 1.0 (right), 0.0 (center)
                    self.red_ball_position = (cx - image_width / 2.0) / (image_width / 2.0)
                else:
                    self.red_ball_position = 0.0
            else:
                self.red_ball_position = 0.0
            
            # Log the values for debugging
            self.get_logger().info(
                f"[Red Ball Detection] Magnitude: {self.red_ball_magnitude:.4f} "
                f"(red pixels: {red_pixel_count}/{total_pixels}), "
                f"Position: {self.red_ball_position:.3f} "
                f"({'LEFT' if self.red_ball_position < -0.1 else 'RIGHT' if self.red_ball_position > 0.1 else 'CENTER'})"
            )
            
        except Exception as e:
            self.get_logger().error(f"[Red Ball] Error processing image: {str(e)}")

    def _wheels_speed_cb(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            return
        left, right = msg.data

        twist = Twist()
        twist.linear.x = (left + right) / 2.0
        twist.angular.z = (right - left) / WHEELBASE

        scale = max(abs(twist.linear.x), abs(twist.angular.z))
        if scale > 1.0:
            twist.linear.x /= scale
            twist.angular.z /= scale
            
        #self._cmd_vel_pub.publish(twist)
        # new with TwistStamped
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = twist
        self._cmd_vel_pub.publish(stamped)


        self.latest_wheels_speed = [left, right]
        self.latest_cmd_vel = (twist.linear.x, twist.angular.z)

    def _publish_robot_state(self):
        self._neighbours_cb()

        msg = RobotState()
        msg.robot_id = self.robot_id
        msg.floor_color = self.latest_floor_color
        msg.neighbour_count = self.latest_neighbour_count
        msg.attraction_angle = self.latest_attraction_angle
        msg.proximity_magnitude, msg.proximity_angle = self._compute_proximity()
        msg.light_magnitude, msg.light_angle = self._compute_light_vector()
        msg.red_ball_magnitude = float(self.red_ball_magnitude)
        msg.red_ball_position = float(self.red_ball_position)

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