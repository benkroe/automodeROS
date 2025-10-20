#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from typing import Dict, Any
import time

from std_msgs.msg import Float32, Float32MultiArray, String
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from automode_interfaces.msg import RobotState
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from webots_ros2_msgs.msg import StringStamped, FloatStamped
from std_msgs.msg import Int32
from sensor_msgs.msg import Illuminance
from geometry_msgs.msg import Vector3Stamped


TYPE_MAP = {
    'std_msgs/msg/Float32': Float32,
    'std_msgs/msg/Float32MultiArray': Float32MultiArray,
    'std_msgs/msg/String': String,
    'std_msgs/msg/Int32': Int32,
    'sensor_msgs/msg/Range': Range,
    'sensor_msgs/msg/Illuminance': Illuminance,
    'sensor_msgs/msg/LaserScan': LaserScan,
    'geometry_msgs/msg/Twist': Twist,
    'geometry_msgs/msg/Vector3Stamped': Vector3Stamped,
    'nav_msgs/msg/Odometry': Odometry,
    'tf2_msgs/msg/TFMessage': TFMessage,
    'automode_interfaces/msg/RobotState': RobotState,
    'webots_ros2_msgs/msg/StringStamped': StringStamped,
    'webots_ros2_msgs/msg/FloatStamped': FloatStamped,
}

REF_TOPICS = [
    # Existing topics
    'wheels_speed', '/wheels_speed',
    'ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7',
    'ls0','ls1','ls2','ls3','ls4','ls5','ls6','ls7',
    
    # Robot state and control
    'robotState', '/robotState',
    '/e_puck/robotState',
    '/e_puck/wheels_speed',
    
    # Receiver related
    'receiver/data', '/receiver/data',
    '/e_puck/receiver/data',
    '/e_puck/receiver/emitter_direction',
    '/e_puck/receiver/signal_strength',
    
    # Behavior and condition info
    '/behaviors/list',
    '/conditions/list',
    '/e_puck/behaviors/list',
    '/e_puck/conditions/list',
    
    # Navigation related
    '/odom',
    '/tf',
    '/tf_static',
    
    # Additional sensor data
    '/scan',
    '/tof',
    
    # LED status (optional but might be useful)
    '/led0', '/led1', '/led2', '/led3', '/led4',
    '/led5', '/led6', '/led7', '/led8', '/led9',
    '/pi_puck_led_0', '/pi_puck_led_1', '/pi_puck_led_2',
]


class EpuckTopicInspector(Node):
    def __init__(self):
        # Call parent class constructor first
        super().__init__('epuck_topic_inspector')
        
        # Initialize instance attributes
        self._subscriptions = []  # Use underscore for "private" attribute
        self._last = {}  # Change to underscore for consistency
        
        # Start node
        self.get_logger().info('epuck_topic_inspector starting')
        self.create_timer(0.5, self._print_status)
        self._create_subscriptions_for_ref_topics()


    def _create_subscriptions_for_ref_topics(self):
        topics_and_types = dict(self.get_topic_names_and_types())

        for topic in REF_TOPICS:
            candidates = [topic]
            if not topic.startswith('/'):
                candidates.insert(0, '/' + topic)

            found = False
            for tname in candidates:
                if tname in topics_and_types:
                    types = topics_and_types[tname]
                    # Try to subscribe to all supported message types
                    for typ in types:
                        if typ in TYPE_MAP:
                            msg_cls = TYPE_MAP[typ]
                            self._subscribe_safe(tname, msg_cls)
                            found = True
                    if not found and types:
                        self.get_logger().warning(f"Topic {tname} has no supported types in {types}")
                        found = True
            if not found:
                self.last[topic] = {'present': False, 'type': None, 'value': None, 'ts': None}

    def _subscribe_safe(self, topic_name: str, msg_cls):
        # Use reliable QoS for all sensor messages
        if msg_cls in (Range, Illuminance, LaserScan):
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
            self.get_logger().debug(f"Using RELIABLE QoS for sensor topic {topic_name}")
        else:
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
            self.get_logger().debug(f"Using default RELIABLE QoS for {topic_name}")

        try:
            sub = self.create_subscription(msg_cls, topic_name, cb, qos)
            self._subscriptions.append(sub)  # Updated reference
            self.get_logger().info(f"Subscribed to {topic_name} as {msg_cls.__name__}")
            
            self._last[topic_name] = {  # Updated reference
                'present': True,
                'type': f"{msg_cls.__module__}/{msg_cls.__name__}",
                'value': None,
                'ts': None,
                'sub': sub
            }
            return sub
        except Exception as e:
            self.get_logger().error(f"Failed to subscribe to {topic_name}: {e}")
            return None

    def _generic_cb(self, topic: str, msg):
        self.get_logger().debug(f"Callback received for {topic}: {type(msg).__name__}")

        try:
            if isinstance(msg, RobotState):
                val = {
                    'robot_id': getattr(msg, 'robot_id', None),
                    'proximity_magnitude': getattr(msg, 'proximity_magnitude', None),
                    'proximity_angle': getattr(msg, 'proximity_angle', None),
                    'light_magnitude': getattr(msg, 'light_magnitude', None),
                    'light_angle': getattr(msg, 'light_angle', None),
                    'neighbour_count': getattr(msg, 'neighbour_count', None),
                    'attraction_angle': getattr(msg, 'attraction_angle', None),
                    'floor_color': getattr(msg, 'floor_color', None),
                    'stamp': getattr(msg, 'stamp', None),
                }
            elif isinstance(msg, Odometry):
                val = {
                    'pos': (msg.pose.pose.position.x, msg.pose.pose.position.y),
                    'orient': msg.pose.pose.orientation.z,
                    'vel': (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
                }
            elif isinstance(msg, LaserScan):
                val = {
                    'range_min': min(r for r in msg.ranges if r > 0),
                    'range_max': max(msg.ranges),
                    'samples': len(msg.ranges)
                }
            elif isinstance(msg, Int32):
                val = int(msg.data)
            elif isinstance(msg, Illuminance):
                val = float(msg.illuminance)
            elif isinstance(msg, Vector3Stamped):
                val = {
                    'x': msg.vector.x,
                    'y': msg.vector.y,
                    'z': msg.vector.z
                }
            elif isinstance(msg, StringStamped):
                val = msg.data
            elif isinstance(msg, FloatStamped):
                val = float(msg.data)
            elif isinstance(msg, TFMessage):
                val = f"{len(msg.transforms)} transforms"
            elif isinstance(msg, Float32):
                val = float(msg.data)
            elif isinstance(msg, Float32MultiArray):
                val = list(msg.data) if msg.data is not None else []
            elif isinstance(msg, Range):
                # Add more Range message details
                val = {
                    'range': float(msg.range),
                    'min_range': float(msg.min_range),
                    'max_range': float(msg.max_range),
                    'fov': float(msg.field_of_view)
                }
                self.get_logger().debug(f"Received Range on {topic}: {val['range']}m")
            elif isinstance(msg, Twist):
                val = {'linear_x': msg.linear.x, 'angular_z': msg.angular.z}
            elif isinstance(msg, String):
                # Skip long behavior/condition list messages
                if any(x in topic for x in ['/behaviors/list', '/conditions/list']):
                    val = '<behavior/condition list>'
                else:
                    val = msg.data
            else:
                val = repr(msg)[:200]
        except Exception as e:
            val = f"<parse_error: {e}>"

        # Preserve existing subscription when updating topic info
        existing = self.last.get(topic, {})
        self.last[topic] = {
            'present': True,
            'type': type(msg).__name__,
            'value': val,
            'ts': time.time(),
            'sub': existing.get('sub')  # Keep the subscription reference
        }

    def _print_status(self):
        current = dict(self.get_topic_names_and_types())
        for key in list(self.last.keys()):
            name = key
            if name not in current and ('/' + key) in current:
                name = '/' + key
            present = name in current
            self.last[key]['present'] = present
            if present and self.last[key].get('type') is None:
                types = current.get(name, [])
                self.last[key]['type'] = types[0] if types else None

        out_lines = []
        out_lines.append('--- epuck_topic_inspector status ---')
        ts = time.strftime('%H:%M:%S', time.localtime())
        out_lines.append(f'time: {ts}')
        rs_key = 'robotState'
        if rs_key in self.last and self.last[rs_key]['present'] and self.last[rs_key]['value'] is not None:
            out_lines.append('robotState:')
            for k, v in self.last[rs_key]['value'].items():
                out_lines.append(f'  {k}: {v}')
        else:
            out_lines.append('robotState: (no data)')

        out_lines.append('tracked topics:')
        for t in sorted(self.last.keys()):
            info = self.last[t]
            present = info.get('present', False)
            typ = info.get('type')
            val = info.get('value')
            age = '--' if info.get('ts') is None else f"{(time.time()-info['ts']):.2f}s"
            out_lines.append(f' - {t:25s} present={str(present):5s} type={str(typ):25s} age={age} val={val}')
        self.get_logger().info('\n'.join(out_lines))


def main(args=None):
    rclpy.init(args=args)
    node = EpuckTopicInspector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()