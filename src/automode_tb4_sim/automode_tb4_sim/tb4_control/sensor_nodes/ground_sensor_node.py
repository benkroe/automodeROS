import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

# This node emulates a single ground color sensor at the center of the robot.

class GroundSensor(Node):

    # Black patch position and size (must match SDF)
    BLACK_X = -8
    BLACK_Y = -8
    BLACK_SIZE = 2  # meters (patch is 2x2)

    def __init__(self):
        super().__init__('ground_sensor_node')

        self.sensor_frame = 'ground_sensor_center'
        self.base_frame = 'white'  # Reference frame for the arena

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.01, self.on_timer)  # Publish at 100 Hz

        # Publisher for the ground sensor
        self.ground_publisher = self.create_publisher(String, self.sensor_frame, qos_profile_sensor_data)

        self.namespace = self.get_namespace().strip('/')

    def on_timer(self):
        target_frame = f'{self.namespace}/{self.sensor_frame}'

        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame,  # Reference frame
                target_frame,     # Child frame
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
        else:
            color = "white"

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