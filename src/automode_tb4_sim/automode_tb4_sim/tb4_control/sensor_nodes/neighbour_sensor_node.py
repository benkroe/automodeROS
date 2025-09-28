import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from math import atan2
import math


# This node publishes neighbour count and attraction angle as a string message.
BASE_FRAME = 'mission'
#BASE_FRAME = 'white'


class NeighboursSensor(Node):

    DETECTION_RADIUS = 2.5  # meters

    def __init__(self):
        super().__init__('neighbours_sensor_node')

        self.sensor_frame = 'neighbours_info'
        self.base_frame = BASE_FRAME

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.2, self.on_timer)  # Publish at 5 Hz

        self.neighbours_publisher = self.create_publisher(String, self.sensor_frame, qos_profile_sensor_data)

        self.namespace = self.get_namespace().strip('/')

        # List of robot namespaces to check (could be parameterized)
        self.robot_namespaces = ['tb1', 'tb2', 'tb3', 'tb4']

    def on_timer(self):
        own_frame = f'{self.namespace}/turtlebot4/base_link'
        try:
            own_transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                own_frame,
                rclpy.time.Time()
            )
        except TransformException:
            self.get_logger().info(f'Could not transform {own_frame} to {self.base_frame}')
            return

        own_x = own_transform.transform.translation.x
        own_y = own_transform.transform.translation.y
        q = own_transform.transform.rotation
       
        yaw = quaternion_to_yaw(q)

        count = 0
        angles = []

        for ns in self.robot_namespaces:
            if ns == self.namespace:
                continue
            target_frame = f'{ns}/turtlebot4/base_link'
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    target_frame,
                    rclpy.time.Time()
                )
                x = t.transform.translation.x
                y = t.transform.translation.y
                dx = x - own_x
                dy = y - own_y
                dist = (dx ** 2 + dy ** 2) ** 0.5
                # self.get_logger().info(f"Neighbour {ns}: dx={dx:.2f}, dy={dy:.2f}, dist={dist:.2f}")

                if dist <= self.DETECTION_RADIUS:
                    count += 1
                    world_angle = atan2(dy, dx)
                    rel_angle = self.normalize_angle(world_angle - yaw)
                    angles.append(rel_angle)
            except TransformException:
                continue

        if count > 0:
            x_sum = sum(math.cos(a) for a in angles)
            y_sum = sum(math.sin(a) for a in angles)
            attraction_angle = math.atan2(y_sum, x_sum)
        else:
            attraction_angle = 0.0

        msg = String()
        msg.data = f"{attraction_angle},{count}"
        # self.get_logger().info(f"Publishing neighbours_info: {msg.data}") Logger to see neigboru sensor information
        self.neighbours_publisher.publish(msg)

        self.neighbours_publisher.publish(msg)

    def normalize_angle(self, angle):
        from math import atan2, sin, cos
        return atan2(sin(angle), cos(angle))
        
def quaternion_to_yaw(q):
    import math
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = NeighboursSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()