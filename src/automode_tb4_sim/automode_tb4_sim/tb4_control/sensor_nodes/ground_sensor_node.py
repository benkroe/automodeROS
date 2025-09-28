import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

class GroundSensor(Node):

    # ======= Uncomment ONE block below to select world patches and base frame =======

    # --- Black and white patches for white.sdf ---
    # BASE_FRAME = 'white'
    # BLACK_X = -9
    # BLACK_Y = -9
    # BLACK_SIZE = 2  # meters (patch is 2x2)
    # WHITE_X = -2
    # WHITE_Y = -2
    # WHITE_SIZE = 4  # meters (patch is 4x4)
    # USE_MISSION = False

    # --- Black and white patches for mission.sdf ---
    BASE_FRAME = 'mission'
    WHITE_X_MIN = -3
    WHITE_X_MAX = 3
    WHITE_Y_MIN = 1
    WHITE_Y_MAX = 3
    BLACK1_X = -1.5
    BLACK1_Y = -1.0
    BLACK1_RADIUS = 0.4
    BLACK2_X = 1.5
    BLACK2_Y = -1.0
    BLACK2_RADIUS = 0.4
    USE_MISSION = True

    def __init__(self):
        super().__init__('ground_sensor_node')
        self.sensor_frame = 'ground_sensor_center'
        self.base_frame = self.BASE_FRAME  # Use the variable!
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.2, self.on_timer)
        self.ground_publisher = self.create_publisher(String, self.sensor_frame, qos_profile_sensor_data)
        self.namespace = self.get_namespace().strip('/')
        self.last_color = "white"

    def on_timer(self):
        target_frame = f'{self.namespace}/turtlebot4/base_link'
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                target_frame,
                rclpy.time.Time(),
            )
        except TransformException as ex:
            return

        x = t.transform.translation.x
        y = t.transform.translation.y

        if getattr(self, "USE_MISSION", False):
            # Mission world logic
            if (self.WHITE_X_MIN <= x <= self.WHITE_X_MAX and
                self.WHITE_Y_MIN <= y <= self.WHITE_Y_MAX):
                color = "white"
            elif ((x - self.BLACK1_X)**2 + (y - self.BLACK1_Y)**2 <= self.BLACK1_RADIUS**2):
                color = "black"
            elif ((x - self.BLACK2_X)**2 + (y - self.BLACK2_Y)**2 <= self.BLACK2_RADIUS**2):
                color = "black"
            else:
                color = "gray"
        else:
            # White world logic
            if (self.WHITE_X <= x <= self.WHITE_X + self.WHITE_SIZE and
                self.WHITE_Y <= y <= self.WHITE_Y + self.WHITE_SIZE):
                color = "white"
            elif (self.BLACK_X <= x <= self.BLACK_X + self.BLACK_SIZE and
                  self.BLACK_Y <= y <= self.BLACK_Y + self.BLACK_SIZE):
                color = "black"
            else:
                color = "gray"

        self.last_color = color
        msg = String()
        msg.data = color
        self.ground_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ground_sensor_node = GroundSensor()
    rclpy.spin(ground_sensor_node)
    ground_sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()