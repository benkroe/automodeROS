import rclpy
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.node import Node
from std_msgs.msg import Float32, String

# this class is used only by the FSM controller

class Cliff_Processing(Node):

    # values of the intensities readings of the 4 different areas (published by the cliff sensors, from cliff_sensors_node.py)
    # since an average of the readings used to compute the area I added a 0.09 offset to all the values
    NEST_intensity = 0.2 + 0.09
    CACHE_intensity = 0.3+ 0.09
    SLOPE_intensity = 0.4+ 0.09
    SOURCE_intensity = 0.5+ 0.09
    
    def __init__(self):
        

        super().__init__('cliff_processing_node')

        # initialize the cliff sensors values
        self.cliff_side_left_value = None
        self.cliff_side_right_value = None
        self.cliff_front_left_value = None
        self.cliff_front_right_value = None

        # subscibers to the cliff sensors
        self.cliff_side_left_subsriber = self.create_subscription(Float32, 'cliff_sensor_side_left', self.cliff_side_left_callback, qos_profile_sensor_data)
        self.cliff_side_right_subsriber = self.create_subscription(Float32, 'cliff_sensor_side_right', self.cliff_side_right_callback, qos_profile_sensor_data)
        self.cliff_front_left_subsriber = self.create_subscription(Float32, 'cliff_sensor_front_left', self.cliff_front_left_callback, qos_profile_sensor_data)
        self.cliff_front_right_subsriber = self.create_subscription(Float32, 'cliff_sensor_front_right', self.cliff_front_right_callback, qos_profile_sensor_data)


        # publisher
        self.cliff_area_publisher = self.create_publisher(String, 'cliff_area', qos_profile_sensor_data)

        # timer to forward the cliff area information
        self.timer = self.create_timer(0.016, self.on_timer) # publish at 60 Hz

    # cliff sensors callbacks
    def cliff_side_left_callback(self, message):
        self.cliff_side_left_value = message.data
        

    def cliff_front_left_callback(self, message):
        self.cliff_front_left_value = message.data
        
    
    def cliff_front_right_callback(self, message):
        self.cliff_front_right_value = message.data
        

    def cliff_side_right_callback(self, message):
        self.cliff_side_right_value = message.data
        

    def on_timer(self):

        # check if there is at least one None variable. If yes, don't compute light direction
        if any(cliff_value is None for cliff_value in [self.cliff_side_left_value, self.cliff_front_left_value, self.cliff_front_right_value, self.cliff_side_right_value]):
            self.get_logger().warning(f'Cannot compute cliff area')
            return

        # compute the cliff area based on the values of the cliff sensors

        # get the average of the cliff sensors values
        cliff_average = (self.cliff_side_left_value + self.cliff_front_left_value + self.cliff_front_right_value + self.cliff_side_right_value) / 4

        # check the area based on the average of the cliff sensors values
        if cliff_average <= Cliff_Processing.NEST_intensity:
            actual_area = 'nest'
        elif cliff_average <= Cliff_Processing.CACHE_intensity:
            actual_area = 'cache'
        elif cliff_average <= Cliff_Processing.SLOPE_intensity:
            actual_area = 'slope'
        else:
            actual_area = 'source'


        # publish the actual area
        area_msg = String()
        area_msg.data = actual_area
        self.cliff_area_publisher.publish(area_msg)
        #self.get_logger().info(f'Cliff area: {actual_area}, cliff average: {cliff_average}')

def main(args=None):
    rclpy.init(args=args)
    cliff_processing_node = Cliff_Processing()
    rclpy.spin(cliff_processing_node)
    cliff_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()