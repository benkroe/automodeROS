import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node


class ir_Processing(Node):

    def __init__(self):

        super().__init__("ir_processing_node")

        # subscriber
        self.ir_subscriber = self.create_subscription(
            Float32MultiArray,
            "ir_intensities",
            self.ir_callback,
            qos_profile_sensor_data,
        )

        # publisher
        self.ir_areas_publisher = self.create_publisher(
            Float32MultiArray, "ir_intensities_normalized", qos_profile_sensor_data
        )

        self.ir_intensities_data = []  # data to publish

        self.timer = self.create_timer(0.016, self.on_timer)  # publish at 60 Hz

        # The following tresholds are computed from the IR intensity using the C++ formula
        # from create3_sim/irobot_create_ignition/irobot_create_ignition_toolbox/src/sensors/ir_intensity.cpp
        self.MAX_IR_INTENSITY = 1725.0
        self.MIN_IR_INTENSITY = 0.0

    def ir_callback(self, msg):

        # normalize the ir intensities between 0 and 1
        ir_intensities_normalized_msg = Float32MultiArray()
        ir_intensities_normalized_msg.data = [
            (ir_intensity - self.MIN_IR_INTENSITY)
            / (self.MAX_IR_INTENSITY - self.MIN_IR_INTENSITY)
            for ir_intensity in msg.data
        ]

        # update the ir intensities data to publish
        self.ir_intensities_data = ir_intensities_normalized_msg

    def on_timer(self):

        if self.ir_intensities_data:

            # publish the normalized IR intensities

            self.ir_areas_publisher.publish(self.ir_intensities_data)

        else:

            self.get_logger().warn("No IR intensities data to process")


def main(args=None):
    rclpy.init(args=args)
    ir_processing_node = ir_Processing()
    rclpy.spin(ir_processing_node)
    ir_processing_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
