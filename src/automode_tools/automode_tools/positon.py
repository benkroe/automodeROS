import rclpy
from rclpy.node import Node
from vicon_receiver.msg import Position
from vicon_receiver.msg import PositionList

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        vicon_topic = "vicon/default/data"
        
        self.vicon_subscription =  self.create_subscription(
            PositionList,
            vicon_topic,
            self.vicon_callback,
            10
        )
        self.my_id = "id0"
        self.my_position = None
    def vicon_callback(self, msg):
        for i in range(msg.n):
            if msg.positions[i].subject_name == self.my_id:
                print('Id = %s' % self.my_id)
                self.my_position = msg.positions[i]
                break
        print(self.my_position)
        print('done')