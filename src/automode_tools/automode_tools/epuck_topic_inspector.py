#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from typing import Dict, Any
import time

from std_msgs.msg import Float32, Float32MultiArray, String, Int32
from sensor_msgs.msg import Range, LaserScan, Illuminance
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from automode_interfaces.msg import RobotState
from webots_ros2_msgs.msg import StringStamped, FloatStamped

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
    
    # LED status
    '/led0', '/led1', '/led2', '/led3', '/led4',
    '/led5', '/led6', '/led7', '/led8', '/led9',
    '/pi_puck_led_0', '/pi_puck_led_1', '/pi_puck_led_2',
]

class EpuckTopicInspector(Node):
    def __init__(self):
        super().__init__('epuck_topic_inspector')
        self._subscriptions = []
        self._last = {}
        self.get_logger().info('epuck_topic_inspector starting')
        # Allow time for topic discovery
        time.sleep(1.0)
        self.create_timer(0.5, self._print_status)
        self._create_subscriptions_for_ref_topics()

    def _create_subscriptions_for_ref_topics(self):
        topics_and_types = dict(self.get_topic_names_and_types())
        self.get_logger().debug(f"Found topics: {list(topics_and_types.keys())}")

        for topic in REF_TOPICS:
            candidates = [topic]
            if not topic.startswith('/'):
                candidates.insert(0, '/' + topic)

            found = False
            for tname in candidates:
                if tname in topics_and_types:
                    types = topics_and_types[tname]
                    for typ in types:
                        if typ in TYPE_MAP:
                            msg_cls = TYPE_MAP[typ]
                            self._subscribe_safe(tname, msg_cls)
                            found = True
            if not found:
                self._last[topic] = {'present': False, 'type': None, 'value': None, 'ts': None}

    def _subscribe_safe(self, topic_name: str, msg_cls):
        # Use reliable QoS for all messages
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        try:
            def callback(msg):
                try:
                    self._generic_cb(topic_name, msg)
                except Exception as e:
                    self.get_logger().error(f"Callback error for {topic_name}: {e}")

            sub = self.create_subscription(msg_cls, topic_name, callback, qos)
            self._subscriptions.append(sub)
            self.get_logger().debug(f"Created subscription for {topic_name} ({msg_cls.__name__})")
            
            self._last[topic_name] = {
                'present': True,
                'type': msg_cls.__name__,
                'value': None,
                'ts': None,
                'sub': sub
            }
            return sub
        except Exception as e:
            self.get_logger().error(f"Failed to subscribe to {topic_name}: {e}")
            return None

    def _generic_cb(self, topic: str, msg):
        try:
            if isinstance(msg, RobotState):
                val = {
                    'robot_id': msg.robot_id,
                    'proximity_magnitude': msg.proximity_magnitude,
                    'proximity_angle': msg.proximity_angle,
                    'light_magnitude': msg.light_magnitude,
                    'light_angle': msg.light_angle,
                    'neighbour_count': msg.neighbour_count,
                    'attraction_angle': msg.attraction_angle,
                    'floor_color': msg.floor_color,
                    'stamp': msg.stamp
                }
            elif isinstance(msg, Range):
                val = float(msg.range)  # Only store the range value
                self.get_logger().debug(f"Range on {topic}: {val}m")
            elif isinstance(msg, Odometry):
                val = {
                    'pos': (msg.pose.pose.position.x, msg.pose.pose.position.y),
                    'orient': msg.pose.pose.orientation.z,
                }
            elif isinstance(msg, LaserScan):
                val = min(r for r in msg.ranges if r > 0)  # Only store closest valid range
            elif isinstance(msg, Int32):
                val = int(msg.data)
            elif isinstance(msg, Illuminance):
                val = float(msg.illuminance)
            elif isinstance(msg, Vector3Stamped):
                val = (msg.vector.x, msg.vector.y, msg.vector.z)
            elif isinstance(msg, StringStamped):
                val = msg.data
            elif isinstance(msg, FloatStamped):
                val = float(msg.data)
            elif isinstance(msg, TFMessage):
                val = len(msg.transforms)
            elif isinstance(msg, Float32):
                val = float(msg.data)
            elif isinstance(msg, Float32MultiArray):
                val = list(msg.data) if msg.data is not None else []
            elif isinstance(msg, Twist):
                val = (msg.linear.x, msg.angular.z)
            elif isinstance(msg, String):
                val = msg.data
            else:
                val = str(msg)[:100]  # Truncate long messages

            # Update topic info while preserving subscription
            existing = self._last.get(topic, {})
            self._last[topic] = {
                'present': True,
                'type': type(msg).__name__,
                'value': val,
                'ts': time.time(),
                'sub': existing.get('sub')
            }
            self.get_logger().debug(f"Updated {topic} with value: {val}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing {topic}: {e}")
            
    def _print_status(self):
        current = dict(self.get_topic_names_and_types())
        for key in list(self._last.keys()):
            name = key
            if name not in current and ('/' + key) in current:
                name = '/' + key
            present = name in current
            self._last[key]['present'] = present
            if present and self._last[key].get('type') is None:
                types = current.get(name, [])
                self._last[key]['type'] = types[0] if types else None

        out_lines = []
        out_lines.append('--- epuck_topic_inspector status ---')
        ts = time.strftime('%H:%M:%S', time.localtime())
        out_lines.append(f'time: {ts}')
        
        # Print robot state if available
        rs_key = 'robotState'
        if rs_key in self._last and self._last[rs_key]['present'] and self._last[rs_key]['value'] is not None:
            out_lines.append('robotState:')
            for k, v in self._last[rs_key]['value'].items():
                out_lines.append(f'  {k}: {v}')
        else:
            out_lines.append('robotState: (no data)')

        # Print all tracked topics
        out_lines.append('tracked topics:')
        for t in sorted(self._last.keys()):
            info = self._last[t]
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