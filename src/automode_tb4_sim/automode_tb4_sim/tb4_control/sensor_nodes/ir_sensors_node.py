import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from irobot_create_msgs.msg import IrIntensityVector, IrIntensity
import math
import functools
from rclpy.qos import qos_profile_sensor_data
import numpy as np


class IRSensor(Node):

    def __init__(self):
        super().__init__('ir_sensors_node')

        self.namespace = self.get_namespace().strip('/')

        # List of IR sensor topic names
        self.ir_sensors_list = [
            f'/{self.namespace}/_internal/ir_intensity_front_center_left/scan',
            f'/{self.namespace}/_internal/ir_intensity_front_center_right/scan',
            f'/{self.namespace}/_internal/ir_intensity_front_left/scan',
            f'/{self.namespace}/_internal/ir_intensity_front_right/scan',
            f'/{self.namespace}/_internal/ir_intensity_left/scan',
            f'/{self.namespace}/_internal/ir_intensity_right/scan',
            f'/{self.namespace}/_internal/ir_intensity_side_left/scan'
        ]

        # Dictionary to store IR sensor values (initialized to 0)
        self.ir_values = {sensor: 0.0 for sensor in self.ir_sensors_list}

        # Subscriptions to LaserScan topics
        self.scan_subscriptions = []
        for topic in self.ir_sensors_list:
            sub = self.create_subscription(
                LaserScan, topic,
                functools.partial(self.ir_scan_callback, topic=topic),
                qos_profile_sensor_data  # Reliable QoS for sensor data
            )
            self.scan_subscriptions.append(sub)

        # Publisher for the IR intensity vector (changed from Float32MultiArray)
        self.ir_intensities_publisher = self.create_publisher(IrIntensityVector, 'ir_intensities', qos_profile_sensor_data)


    def ir_scan_callback(self, msg, topic):
        """
        Callback function triggered when a LaserScan message is received.
        Computes the IR intensity and updates the corresponding sensor value.
        """
        # Filter out `inf` values (invalid readings)
        valid_ranges = [r for r in msg.ranges if math.isfinite(r)]

        # If all values are `inf`, set intensity to 0
        if not valid_ranges:
            self.ir_values[topic] = 0.0
        else:
            detection = min(valid_ranges)  # Use only valid values

            # Compute the IR intensity using the C++ formula
            # from create3_sim/irobot_create_ignition/irobot_create_ignition_toolbox/src/sensors/ir_intensity.cpp
            scaled_detection = 3500 * math.exp(detection * (-2 * math.e / msg.range_max))

            
            # Ensure the max value is 1725 
            if scaled_detection > 1725:
                scaled_detection = 1725

            # Update sensor value
            self.ir_values[topic] = float(scaled_detection)

        # Publish the updated IR intensity vector
        self.publish_ir_data()



    def publish_ir_data(self):
        """Publishes the updated IR intensity vector to the topic 'ir_intensities'."""
        ir_intensities_msg = IrIntensityVector()

        # Produce the same numeric values you used before (raw [0..1725] with noise)
        values = [self.ir_values[sensor] for sensor in self.ir_sensors_list]
        values = [x + np.random.normal(0.0, 10) for x in values]
        values = np.clip(values, 0.0, 1725.0).tolist()  # ensure plain Python list

        # Create IrIntensity reading objects and populate .value
        ir_intensities_msg.readings = []
        for v in values:
            reading = IrIntensity()
            reading.value = int(round(v))
            ir_intensities_msg.readings.append(reading)

        self.ir_intensities_publisher.publish(ir_intensities_msg)
        # self.get_logger().info(f'Published IR intensities (IrIntensityVector): {[r.value for r in ir_intensities_msg.readings]}')


def main(args=None):
    rclpy.init(args=args)
    ir_sensor_node = IRSensor()
    rclpy.spin(ir_sensor_node)
    ir_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()