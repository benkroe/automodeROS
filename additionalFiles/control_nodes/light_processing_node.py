import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32, String
from rclpy.qos import qos_profile_sensor_data
from math import isclose
import numpy as np
import math


# node to compute the light direciton based on the value from the light sensors
# callback functions just update the value of the last reading
# compute_light_directon function is called periodically and takes the actual values of the light intensity stored for each sensor


class Light_Processing(Node):

    def __init__(self):

        super().__init__("light_direction_node")

        # subscribers
        self.light_frontL_subscriber = self.create_subscription(
            Float32,
            "light_sensor_front_left",
            self.light_frontL_callback,
            qos_profile_sensor_data,
        )  # light front left
        self.light_frontR_subscriber = self.create_subscription(
            Float32,
            "light_sensor_front_right",
            self.light_frontR_callback,
            qos_profile_sensor_data,
        )  # light front right
        self.light_back_subscriber = self.create_subscription(
            Float32,
            "light_sensor_back",
            self.light_back_callback,
            qos_profile_sensor_data,
        )  # light back

        # publisher
        self.light_direction_publisher = self.create_publisher(
            Float32, "light_direction", qos_profile_sensor_data
        )
        self.light_direction_publisher_string = self.create_publisher(
            String, "light_direction_string", qos_profile_sensor_data
        )

        # light sensors reading
        self.light_frontL_value = None
        self.light_frontR_value = None
        self.light_back_value = None

        # timer to call the function to compute and publish the light direction periodically
        self.timer = self.create_timer(0.016, self.compute_light_direction) # publish at 60 Hz

        # position of the light sensors on the top plate of the robot (0 deg is the front of the robot)
        self.angles = {
            "FL": math.radians(60),  # Front Left
            "FR": math.radians(-60),  # Front Right
            "B": math.radians(180),  # Back
        }

    def light_frontL_callback(self, light_intensity_message):
        self.light_frontL_value = light_intensity_message.data

    def light_frontR_callback(self, light_intensity_message):
        self.light_frontR_value = light_intensity_message.data

    def light_back_callback(self, light_intensity_message):
        self.light_back_value = light_intensity_message.data

    # I simplified the light direction in 4 possible orientation values + a fallback value
    # if you want to use vector-based computation define another callback function
    def compute_light_direction(self):

        # check if there is at least one None variable. If yes, don't compute light direction
        if any(
            light_value is None
            for light_value in [
                self.light_frontL_value,
                self.light_frontR_value,
                self.light_back_value,
            ]
        ):

            self.get_logger().warning(f"Cannot compute light direction")
            return
        
        x = (
            self.light_frontL_value * math.cos(self.angles["FL"])
            + self.light_frontR_value * math.cos(self.angles["FR"])
            + self.light_back_value * math.cos(self.angles["B"])
        )
        y = (
            self.light_frontL_value * math.sin(self.angles["FL"])
            + self.light_frontR_value * math.sin(self.angles["FR"])
            + self.light_back_value * math.sin(self.angles["B"])
        )

        heading = math.degrees(math.atan2(y, x)) % 360
        if heading > 180:
            heading -= 360

        # my_angle = Float32()
        # my_angle.data = heading
        # self.angle_publisher.publish(my_angle)

        if -30 <= heading <= 30:  # Front
            light_direction = 0.2
            light_direction_string = 'front'
        elif 30 < heading <= 150:  # Left
            light_direction = 0.8
            light_direction_string = 'left'
        elif -150 < heading <= -30:  # Right
            light_direction = 0.4
            light_direction_string = 'right'
        else:  # Back
            light_direction = 0.6
            light_direction_string = 'back'

        # create and publish light direction message
        light_direction_msg = Float32()
        light_direction_msg.data = (
            light_direction  # normalize the value between 0 and 1
        )

        # light_direction_msg.data = angle_rad # uncommet this line for the fsm
        self.light_direction_publisher.publish(light_direction_msg)

        # create and publish light direction string message
        light_direction_string_msg = String()
        light_direction_string_msg.data = light_direction_string
        self.light_direction_publisher_string.publish(light_direction_string_msg)


def main(args=None):

    rclpy.init(args=args)
    light_direction_node = Light_Processing()
    rclpy.spin(light_direction_node)
    light_direction_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
