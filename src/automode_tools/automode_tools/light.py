from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_veml7700
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data

class LightSensorNode(Node):
    def __init__(self):

        super().__init__('light_sensor_node')

        self.robot_name = "turtlebot4_11"

        self.light_sensors_list = [f'/{self.robot_name}/light_sensor_front_left', f'/{self.robot_name}/light_sensor_front_right', f'/{self.robot_name}/light_sensor_back']

        i2c_1 = I2C(1)
        self.fr_sensor = adafruit_veml7700.VEML7700(i2c_1)

        i2c_4 = I2C(4) 
        self.back_sensor = adafruit_veml7700.VEML7700(i2c_4)

        i2c_5 = I2C(5)
        self.fl_sensor = adafruit_veml7700.VEML7700(i2c_5)

        self.light_publishers = {}
        for light in self.light_sensors_list:
            self.light_publishers[light] = self.create_publisher(Float32, light, qos_profile_sensor_data)

        self.namespace = self.robot_name
        self.get_logger().info(f"Namespace: {self.namespace}")
        timer_period = 1/30.0

        self.timer = self.create_timer(timer_period, self.on_timer)

    def on_timer(self):
        c1 = self.fr_sensor.light  
        c2 = self.back_sensor.light
        c3 = self.fl_sensor.light

        self.get_logger().info(f"Front Right: {c1}, Back: {c2}, Front Left: {c3}")

        msg1 = Float32()
        msg1.data = float(c1)
        self.light_publishers[f'/{self.robot_name}/light_sensor_front_right'].publish(msg1)

        msg2 = Float32()
        msg2.data = float(c2)
        self.light_publishers[f'/{self.robot_name}/light_sensor_back'].publish(msg2)

        msg3 = Float32()
        msg3.data = float(c3)
        self.light_publishers[f'/{self.robot_name}/light_sensor_front_left'].publish(msg3)

def main(args=None):
    rclpy.init(args=args)
    light_sensor_node = LightSensorNode()
    rclpy.spin(light_sensor_node)
    light_sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

