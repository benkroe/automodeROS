import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data

from math import sqrt

# This node emulates 3 light sensors.
# It gets the distance of a light sensor (a tf frame) from the light source.
# The position of the light is hardcoded (values from arena.sdf).
# As the TurtleBot moves away from the light, the sensor readings decrease.

class LightSensor(Node):

    # Light source position from arena.sdf
    LIGHT_1_X = 0
    LIGHT_1_Y = 0

    def __init__(self):
        super().__init__('light_sensors_node')

        self.light_sensors_list = ['light_sensor_front_left', 'light_sensor_front_right', 'light_sensor_back']
        self.base_frame = 'white'  # Reference frame for the arena

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.01, self.on_timer)  # Publish at 100 Hz

        # Create publishers for the light sensors
        self.light_publishers = {}
        for light in self.light_sensors_list:
            self.light_publishers[light] = self.create_publisher(Float32, light, qos_profile_sensor_data)

        self.namespace = self.get_namespace().strip('/')

    def on_timer(self):
        for light in self.light_sensors_list:
            target_frame = f'{self.namespace}/{light}'

            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame,  # Reference frame
                    target_frame,     # Child frame
                    rclpy.time.Time()
                )
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {target_frame} to {self.base_frame}: {ex}')
                continue

            # Compute the Euclidean distance from the single light source
            x_diff = LightSensor.LIGHT_1_X - t.transform.translation.x
            y_diff = LightSensor.LIGHT_1_Y - t.transform.translation.y

            distance = sqrt(x_diff**2 + y_diff**2)

            # Sensor value decreases with distance (inverse square law)
            light_sensor_msg = Float32()
            if distance > 0:
                light_sensor_msg.data = 1 / (distance ** 2)
            else:
                light_sensor_msg.data = 0.0

            self.light_publishers[light].publish(light_sensor_msg)

def main(args=None):
    rclpy.init(args=args)
    light_sensor_node = LightSensor()
    rclpy.spin(light_sensor_node)
    light_sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()