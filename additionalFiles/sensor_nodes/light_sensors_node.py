import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data


from math import sqrt

# this node emulates 3 light sensors.
# it simply get the distance of a light sensor (basically a tf) from the light. The position of the light is hardcoded in this code (values from file arena.sdf).
# Basically, as the TurtleBot moves away from the lights (and, consequently, from the sensors attached to it), the (emulated) readings from each sensor decrease linearly.


class LightSensor(Node):

    # lights position from arena.sdf
    LIGHT_1_X = 0
    LIGHT_1_Y = -0.8

    LIGHT_2_X = 1.5
    LIGHT_2_Y = -0.8

    LIGHT_3_X = -1.5
    LIGHT_3_Y = -0.8
    

    def __init__(self):

        super().__init__('light_sensors_node')
        
        self.light_sensors_list = ['light_sensor_front_left', 'light_sensor_front_right', 'light_sensor_back']
        self.base_frame = 'arena' # to get the position related to the arena
        

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.01, self.on_timer) # publish at 100 Hz

        # create publishers for the light sensors
        self.light_publishers = {}
        for light in self.light_sensors_list:
            self.light_publishers[light] = self.create_publisher(Float32, light, qos_profile_sensor_data)

        self.namespace = self.get_namespace().strip('/')

    def on_timer(self):


        for light in self.light_sensors_list:

            target_frame = f'{self.namespace}/{light}'

            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, # reference frame
                    target_frame, # child frame
                    rclpy.time.Time()
                    )
                
                
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {target_frame} to {self.base_frame}: {ex}')
                return
            
            # create and publish light message
            light_sensor_msg = Float32()
            
            # compute the eculidean distance from each light
            # the value of the light detected (output of the sensor) will be the sum of the euclildean distances
            # as far as the overall sum of the distances increase the value willl decrease
            x_diff1 = LightSensor.LIGHT_1_X - t.transform.translation.x
            y_diff1 = LightSensor.LIGHT_1_Y - t.transform.translation.y

            x_diff2 = LightSensor.LIGHT_2_X - t.transform.translation.x
            y_diff2 = LightSensor.LIGHT_2_Y - t.transform.translation.y

            x_diff3 = LightSensor.LIGHT_3_X - t.transform.translation.x
            y_diff3 = LightSensor.LIGHT_3_Y - t.transform.translation.y


            distance1 = sqrt(x_diff1**2 + y_diff1**2)
            distance2 = sqrt(x_diff2**2 + y_diff2**2)
            distance3 = sqrt(x_diff3**2 + y_diff3**2)

            #self.get_logger().info(f'Light sensor {light} distance1: {distance1}, distance2: {distance2}, distance3: {distance3}')

            #light_sensor_msg.data = (-(distance1 + distance2 + distance3)/3) * 10 # use a linear function here
            light_sensor_msg.data = 1/(distance1**2) + 1/(distance2**2) + 1/(distance3**2)
            #self.get_logger().info(f'Light sensor {light} value: {light_sensor_msg.data}')

            #light_sensor_msg.data = (light_sensor_msg.data) ** 100 # scale the value to be between 0 and 1000

            self.light_publishers[light].publish(light_sensor_msg)



def main(args=None):
    rclpy.init(args=args)
    light_sensor_node = LightSensor()
    rclpy.spin(light_sensor_node)
    light_sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
