import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy.time
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import numpy as np

# this node emulate the set of cliff sensors available on the TurtleBot4, it creates the tf static (one for each cliff sensor) based on the position of each sensor outputs a value

class Cliff_Sensor(Node):

    # limits of the 4 different areas (from arena.sdf file)
    NEST_LIMIT = 0.5
    CACHE_LIMIT = 1.5
    SLOPE_LIMIT = 2.6

    def __init__(self):


        self.cliff_sensors_list = ['cliff_sensor_side_left', 'cliff_sensor_side_right', 'cliff_sensor_front_left', 'cliff_sensor_front_right']
        self.base_frame = 'arena' # to get the position related to the arena

        super().__init__('cliff_sensors_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.01, self.on_timer) # publish at 100 Hz

        # create publishers
        self.cliff_publishers = {}
        for cliff in self.cliff_sensors_list:
            self.cliff_publishers[cliff] = self.create_publisher(Float32, cliff, qos_profile_sensor_data)

        self.namespace = self.get_namespace().strip('/')


    
    def on_timer(self):

        for cliff in self.cliff_sensors_list:

            target_frame = f'{self.namespace}/{cliff}'

            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, # reference frame
                    target_frame, # child frame
                    rclpy.time.Time()
                )

            except TransformException as ex:
                self.get_logger().info(f'Could not transform {target_frame} to {self.base_frame}: {ex}')
                return

            y_pose = t.transform.translation.y

            intensity_msg = Float32()

            elements = [0.2, 0.4, 0.6, 0.8] # values for the cliff sensors, they are used to simulate the cliff sensors readings
                                            # nest -> 0.2, cache -> 0.4, slope -> 0.6, source -> 0.8

            if y_pose <= Cliff_Sensor.NEST_LIMIT: # robot on the nest
                probabilities = [0.85, 0.08, 0.05, 0.02]
            elif y_pose <= Cliff_Sensor.CACHE_LIMIT: # robot on the cache
                probabilities = [0.06, 0.85, 0.06, 0.03]
            elif y_pose <= Cliff_Sensor.SLOPE_LIMIT: # robot on the slope
                probabilities = [0.03, 0.06, 0.85, 0.06]
            else:                                   # robot on the source
                probabilities = [0.02, 0.05, 0.08, 0.85]

            intensity_msg.data = np.random.choice(elements, p=probabilities)


            self.cliff_publishers[cliff].publish(intensity_msg)


def main(args=None):
    rclpy.init(args=args)
    cliff_sensor_node = Cliff_Sensor()
    rclpy.spin(cliff_sensor_node)
    cliff_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()