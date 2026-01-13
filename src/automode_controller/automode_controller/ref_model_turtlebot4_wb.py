import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from automode_interfaces.msg import RobotState
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray, Float32, String
from irobot_create_msgs.msg import IrIntensityVector
from sensor_msgs.msg import Illuminance, Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

ROBOT_ID = 1  # Default robot ID (change if used)

PROXIMITY_MAX_RANGE = 2.0  # meters (reduced for IR realism)

# New tunable IR parameters (matches new ir_intensity input semantics)
IR_MIN_RANGE = 0.02  # meters
IR_MAX_RANGE = 0.20  # meters

class TurtleBot4ReferenceNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_reference_node_wb')

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 5)
        self._robot_state_pub = self.create_publisher(RobotState, 'robotState', 10)

        # Subscriber for wheel commands
        self.create_subscription(Float32MultiArray, 'wheels_speed', self._wheels_speed_cb, 10)    
        # subscribe to consolidated ir_intensity only (compatibility pointcloud code removed)
        self.create_subscription(IrIntensityVector, 'ir_intensity', self._ir_intensity_cb, self.qos)

        # subscribe to cliff intensity sensors, for ground color detection
        self.create_subscription(IrIntensityVector, 'cliff_intensity', self._cliff_intensity_cb, self.qos)

        # Optional: Light, ground, and neighbour senso

        self.create_subscription(Illuminance, 'light_front_left', self._light_fl_cb, self.qos)
        self.create_subscription(Illuminance, 'light_front_right', self._light_fr_cb, self.qos)

        #self.create_subscription(String, 'ground_sensor_center', self._ground_sensor_cb)
        self.create_subscription(String, 'neighbours_info', self._neighbours_cb, self.qos)

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
            'front_center_left':  math.radians(0.0),     
            'front_center_right': math.radians(-25.0),   
            'front_left':         math.radians(25.0),    
            'front_right':        math.radians(-50.0),   
            'left':               math.radians(50.0),    
            'right':              math.radians(-75.0),  
            'side_left':          math.radians(75.0)     
        }



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

        # black: (969, 982), grey: (887, 901), white: (962, 975)
        self._floor_color_refs = {
            'black': (969, 969),
            'gray':  (887, 887),
            'white': (962, 962)
        }

        # Initialize entries so compute_proximity has consistent keys
        for name in self._ir_index_map:
            angle = self._sensor_angle_map.get(name, 0.0)
            # store as [distance_m, angle]; default to PROXIMITY_MAX_RANGE -> treated as "no detection"
            self.latest_ir_vectors[name] = [PROXIMITY_MAX_RANGE, angle]

        # Subscribe to camera for red ball detection
        self.create_subscription(Image, '/rgbd_camera/image', self._camera_cb, 10)
        
        # Red ball detection timer (every 0.5 seconds)
        self.create_timer(0.5, self._process_red_ball_detection)

        # Periodic publication (20 Hz)
        self.create_timer(0.05, self._publish_robot_state)


    def _cliff_intensity_cb(self, msg: IrIntensityVector):
        # Use the two front cliff sensors (index 0: front_left, 1: front_right)
        try:
            readings = msg.readings
        except Exception:
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
            # self.get_logger().debug(f"Cliff intensity -> v0={v0} v1={v1} -> colour={best_color} (err={best_err})")

    def _ir_intensity_cb(self, msg: IrIntensityVector):
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
                f = float(val)
            except Exception:
                f = 0.0
            intensities.append(max(0.0, min(1.0, f)))

        # Map intensities -> estimated distance per configured index map
        n = min(len(intensities), len(self._ir_index_map))
        for i in range(n):
            name = self._ir_index_map[i]
            intensity = intensities[i]
            if intensity <= 0.0:
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
        angle_rad = math.atan2(x_total, y_total)  
        return mag, angle_rad

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
        angle_rad = math.atan2(y_total, x_total)  
        return mag, angle_rad

    def _light_fl_cb(self, msg: Illuminance): self.latest_light_fl = msg.illuminance
    def _light_fr_cb(self, msg: Illuminance): self.latest_light_fr = msg.illuminance

    def _neighbours_cb(self, msg: String):
        try:
            angle_str, count_str = msg.data.split(',')
            self.latest_attraction_angle = math.radians(float(angle_str))  # Convert to radians
            self.latest_neighbour_count = int(count_str)
        except Exception:
            pass

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
        twist.angular.z = (right - left) / 0.3

        scale = max(abs(twist.linear.x), abs(twist.angular.z))
        if scale > 1.0:
            twist.linear.x /= scale
            twist.angular.z /= scale
            
        self._cmd_vel_pub.publish(twist)
        # new with TwistStamped
        # stamped = TwistStamped()
        # stamped.header.stamp = self.get_clock().now().to_msg()
        # stamped.twist = twist
        # self._cmd_vel_pub.publish(stamped)


        self.latest_wheels_speed = [left, right]
        self.latest_cmd_vel = (twist.linear.x, twist.angular.z)

    def _publish_robot_state(self):
        msg = RobotState()
        msg.robot_id = self.robot_id
        msg.floor_color = self.latest_floor_color
        msg.neighbour_count = self.latest_neighbour_count
        msg.attraction_angle = self.latest_attraction_angle
        msg.proximity_magnitude, msg.proximity_angle = self.compute_proximity()
        msg.light_magnitude, msg.light_angle = self.compute_light_vector()
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
