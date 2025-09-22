import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from math import atan2

# This node publishes neighbour count and attraction angle as a string message.

class NeighboursSensor(Node):

    DETECTION_RADIUS = 1.5  # meters

    def __init__(self):
        super().__init__('neighbours_sensor_node')

        self.sensor_frame = 'neighbours_info'
        self.base_frame = 'white'  # Arena/world frame

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

        count = 0
        sum_dx = 0.0
        sum_dy = 0.0

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
                if dist <= self.DETECTION_RADIUS:
                    count += 1
                    sum_dx += dx
                    sum_dy += dy
            except TransformException:
                continue

        if count > 0:
            attraction_angle = atan2(sum_dy, sum_dx)
        else:
            attraction_angle = 0.0

        msg = String()
        msg.data = f"{attraction_angle},{count}"
        self.neighbours_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NeighboursSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()