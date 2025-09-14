import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.qos import qos_profile_sensor_data

class Bumper_Processing(Node):

    def __init__(self):

        super().__init__('bumper_processing_node')

        # subscriber
        self.hazard_subscription = self.create_subscription(HazardDetectionVector, 'hazard_detection', self.hazard_callback, qos_profile_sensor_data) 

        # publisher
        self.bumper_areas_publisher = self.create_publisher(Int8MultiArray, 'bumper_areas', qos_profile_sensor_data)

        # list of bumper areas
        self.bumper_areas =  {
            "bump_right" : False,
            "bump_front_right": False,
            "bump_front_center": False,
            "bump_front_left": False,
            "bump_left": False
        }

        self.bumper_areas_triggered = set() # variable to save bumper areas triggered at every iteraction


    def hazard_callback(self, hazard_msg):
        
        # in the hazard detection we have different tipes of hazards, we filter only the bumper triggered
        # see https://github.com/iRobotEducation/irobot_create_msgs/tree/humble/msg

        self.bumper_areas_triggered.clear()

        if not hazard_msg.detections: # if there aren't hazard detected
            for bumper_index in self.bumper_areas:
                self.bumper_areas[bumper_index] = False

        else:
            for hazard in hazard_msg.detections:
                if hazard.hazard_detected == 1: # if the hazard is a bumper
                    self.bumper_areas_triggered.add(hazard.hazard_index)

            for bumper_index in self.bumper_areas:
                self.bumper_areas[bumper_index] = bumper_index in self.bumper_areas_triggered

        # publish the bumper areas status (triggered or not)
        bumper_areas_msg = Int8MultiArray()
        bumper_areas_msg.data = [int(self.bumper_areas[bumper_index]) for bumper_index in self.bumper_areas]
        self.bumper_areas_publisher.publish(bumper_areas_msg)




def main(args=None):
    rclpy.init(args=args)
    bumper_processing_node = Bumper_Processing()
    rclpy.spin(bumper_processing_node)
    bumper_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()