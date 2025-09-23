import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

class GroundSensor(Node):

    # Black patch position and size (must match SDF)
    BLACK_X = -9
    BLACK_Y = -9
    BLACK_SIZE = 2  # meters (patch is 2x2)
    WHITE_X = 0
    WHITE_Y = 0
    WHITE_SIZE = 20  # meters (arena is 20x20, adjust as needed)

    def __init__(self):
        super().__init__('ground_sensor_node')

        self.sensor_frame = 'ground_sensor_center'
        self.base_frame = 'white'  # Reference frame for the arena

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.01, self.on_timer)  # Publish at 100 Hz

        self.ground_publisher = self.create_publisher(String, self.sensor_frame, qos_profile_sensor_data)

        self.namespace = self.get_namespace().strip('/')

        self.last_color = "white"  # Start with white

    def on_timer(self):
        target_frame = f'{self.namespace}/turtlebot4/base_link'

        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                target_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {target_frame} to {self.base_frame}: {ex}')
            return

        x = t.transform.translation.x
        y = t.transform.translation.y

        # Check if sensor is over the black patch
        if (GroundSensor.BLACK_X <= x <= GroundSensor.BLACK_X + GroundSensor.BLACK_SIZE and
            GroundSensor.BLACK_Y <= y <= GroundSensor.BLACK_Y + GroundSensor.BLACK_SIZE):
            color = "black"
        # Check if sensor is over the white floor
        elif (GroundSensor.WHITE_X <= x <= GroundSensor.WHITE_X + GroundSensor.WHITE_SIZE and
              GroundSensor.WHITE_Y <= y <= GroundSensor.WHITE_Y + GroundSensor.WHITE_SIZE):
            color = "white"
        else:
            # On gray: keep previous color
            color = self.last_color

        self.last_color = color

        msg = String()
        msg.data = color
        self.ground_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ground_sensor_node = GroundSensor()
    rclpy.spin(ground_sensor_node)
    ground_sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()