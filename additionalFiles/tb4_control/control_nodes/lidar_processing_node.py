import rclpy
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from math import pi, cos, sin
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class Lidar_Processing(Node):

    def __init__(self):

        super().__init__('lidar_processing_node')

        # subscriber
        self.lidar_subscriber = self.create_subscription(LaserScan, 'scan', self.lidar_callback, qos_profile_sensor_data)

        # publisher
        self.lidar_areas_publisher = self.create_publisher(Float32MultiArray, 'lidar_areas_normalized', qos_profile_sensor_data)
        
        # split laser scan lectures in 8 areas as follows:
        # [-pi/8, pi/8], [pi/8, 3/8 pi], [3/8 pi, 5/8 pi], [5/8 pi, 7/8 pi]
        # [-pi/8, -3/8 pi], [-3/8 pi, -5/8 pi], [-5/8 pi, -7/8 pi], [-7/8 pi, 7/8 pi]
        # lidar section limits name, corresponding angle interval and value
        self.SECTION_LIMITS = {"lidar_side_left":(-pi/8, pi/8, None),
                               "lidar_back_left":(pi/8, 3/8*pi, None),
                               "lidar_back_center":(3/8*pi, 5/8*pi, None),
                               "lidar_back_right":(5/8*pi, 7/8*pi, None),
                               "lidar_front_left":(-pi/8, -3/8*pi, None),
                               "lidar_front_center":(-3/8*pi, -5/8*pi, None),
                               "lidar_front_right":(-5/8*pi, -7/8*pi, None),
                               "lidar_side_right":(-7/8*pi, 7/8*pi, None)}
        
        # Publisher for each specific area, used to visualize the lidar readings in Rviz
        # FIXME this publishers should be enabled/disabled with a parameter
        self.lidar_side_left_publisher = self.create_publisher(LaserScan, 'lidar_area/lidar_side_left', qos_profile_system_default)
        self.lidar_back_left_publisher = self.create_publisher(LaserScan, 'lidar_area/lidar_back_left', qos_profile_system_default)
        self.lidar_back_center_publisher = self.create_publisher(LaserScan, 'lidar_area/lidar_back_center', qos_profile_system_default)
        self.lidar_back_right_publisher = self.create_publisher(LaserScan, 'lidar_area/lidar_back_right', qos_profile_system_default)
        self.lidar_front_left_publisher = self.create_publisher(LaserScan, 'lidar_area/lidar_front_left', qos_profile_system_default)
        self.lidar_front_center_publisher = self.create_publisher(LaserScan, 'lidar_area/lidar_front_center', qos_profile_system_default)
        self.lidar_front_right_publisher = self.create_publisher(LaserScan, 'lidar_area/lidar_front_right', qos_profile_system_default)
        self.lidar_side_right_publisher = self.create_publisher(LaserScan, 'lidar_area/lidar_side_right', qos_profile_system_default)


    def lidar_callback(self, lidar_message):

        switch_lectures = False

        for section_name, (min_angle, max_angle, _) in self.SECTION_LIMITS.items():
            
            # get the lower and upper indexes of the lidar readings that correspond to the current area (specified by the min_angle and max angle limits)
            lower_index, upper_index = self.get_indexes_from_angles(lidar_message, min_angle, max_angle)

            # this because the index in the interval [-7/8*pi, 7/8*pi] are from 39 to 559 but we need 559 to 29, i.e. from 7/8*pi to -7/8*pi
            if (min_angle, max_angle) == list(self.SECTION_LIMITS.values())[-1][:2]:
                switch_lectures = True

            # distances are normalized between 0 and 1
            min_distance_normalized = self.get_min_distance(lidar_message, lower_index, upper_index, switch_lectures)

            # update the value of the minimum distance for the current area
            self.SECTION_LIMITS[section_name] = (min_angle, max_angle, min_distance_normalized)

            self.publish_area_scan(lidar_message, lower_index, upper_index, section_name, switch_lectures)

            #print(f"Section: {section_name}, Indices: ({lower_index}, {upper_index}), {switch_lectures}")

        # publish the minimum distance for each area
        lidar_areas_message = Float32MultiArray()
        lidar_areas_message.data = [value for _, _, value in self.SECTION_LIMITS.values()]
        self.lidar_areas_publisher.publish(lidar_areas_message)
  

    def get_indexes_from_angles(self, lidar_message, min_angle, max_angle):

        # get the indexes of the lidar readings that correspond to the current area interval
        
        min_angle = min_angle + pi
        max_angle = max_angle + pi

        msg_angle_min = lidar_message.angle_min + pi

        # compute correspondig index
        lower_index = int((min_angle - msg_angle_min) / lidar_message.angle_increment)
        upper_index = int((max_angle - msg_angle_min) / lidar_message.angle_increment)

        if lower_index > upper_index:
            lower_index, upper_index = upper_index, lower_index

        return lower_index, upper_index
    
    def get_min_distance(self, lidar_message, lower_index, upper_index, switch_lectures):
        
        # here I have to take the indexes from (7/8*pi) to the end of the array and from 0 to (-7/8*pi)
        if switch_lectures:
            section_interval =  lidar_message.ranges[upper_index:] + lidar_message.ranges[0:lower_index]
        else:
            section_interval = lidar_message.ranges[lower_index:upper_index]



        # define expitly the lidar range max value
        lidar_saturation = 1.0
        lidar_range_max = 1.0


        # remove inf values
        # since the values are between 0.164 and 1, is enough to discard all the values -inf or inf by substitiuting them with the lidar saturation value
        # anyway I take the minimum reading for each sector that will be or lidar_saturation or a value between 0.164 and 1
        #normalized_lectures = [lidar_message.range_max if x==float('inf') or x == float('-inf') else x for x in section_interval]
        normalized_lectures = [lidar_saturation if x==float('inf') or x == float('-inf') else x for x in section_interval]
        
        # since the lidar max readin is 1, there is no need to normalize
        
        # normalise lectures between 0 and 1 (to normalize input values of the ANN)
        # normalized_lectures = [(x - lidar_message.range_min) / (lidar_message.range_max - lidar_message.range_min) for x in normalized_lectures]
        # normalized_lectures = [(x - lidar_message.range_min) / (lidar_range_max - lidar_message.range_min) for x in normalized_lectures]
        
        #return min(normalized_lectures)

        # apply gaussian noise to the normalized lecturess (proportional to the distance reading)
        noisy_lectures = [x + np.random.normal(0.0, x*0.01) for x in normalized_lectures]

        # clip the values to be between 0 and 1
        noisy_lectures = np.clip(noisy_lectures, 0.0, 1.0)



        return min(noisy_lectures)


    def publish_area_scan(self, lidar_message, lower_index, upper_index, section_name, switch_lectures):

        if switch_lectures:
            
            ranges = lidar_message.ranges[upper_index:] + lidar_message.ranges[0:lower_index]

            angle_min = lidar_message.angle_min + upper_index * lidar_message.angle_increment
            angle_max = lidar_message.angle_min + lower_index * lidar_message.angle_increment


        else:
            ranges = lidar_message.ranges[lower_index:upper_index]

            angle_min = lidar_message.angle_min + lower_index * lidar_message.angle_increment
            angle_max = lidar_message.angle_min + upper_index * lidar_message.angle_increment

        scan_message = LaserScan()
        scan_message.header = lidar_message.header 
        scan_message.angle_min = angle_min
        scan_message.angle_max = angle_max
        scan_message.angle_increment = lidar_message.angle_increment
        scan_message.time_increment = lidar_message.time_increment
        scan_message.scan_time = lidar_message.scan_time
        scan_message.range_min = lidar_message.range_min
        scan_message.range_max = lidar_message.range_max
        scan_message.ranges = ranges
        scan_message.intensities = lidar_message.intensities[lower_index:upper_index]

        # select the publisher for the current area
        publisher = getattr(self, f'{section_name}_publisher')
        publisher.publish(scan_message)
        


def main(args=None):
    rclpy.init(args=args)
    lidar_processing_node = Lidar_Processing()
    rclpy.spin(lidar_processing_node)
    lidar_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()