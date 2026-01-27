import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from automode_interfaces.msg import RobotState


class GroupEvaluator(Node):
    def __init__(self):
        super().__init__('group_evaluator')

        # Subscribe to tb1 robotState
        self._last_state = None
        self._in_group = False
        self._count = 0

        self._sub = self.create_subscription(
            RobotState,
            '/tb1/robotState',
            self._robot_state_cb,
            10
        )

        # Timer to check state every 0.2s
        self.create_timer(0.2, self._check_group)

        # Publisher that publishes count every 1s
        self._pub = self.create_publisher(Int32, 'group_count', 10)
        self.create_timer(1.0, self._publish_count)

        # (simulation time publishing removed)

        self.get_logger().info('GroupEvaluator started, subscribing to /tb1/robotState')

    def _robot_state_cb(self, msg: RobotState):
        self._last_state = msg

    def _check_group(self):
        if self._last_state is None:
            return

        neigh = int(self._last_state.neighbour_count)
        if neigh == 2:
            if not self._in_group:
                self._count += 1
                self._in_group = True
                self.get_logger().info(f'Group formed (count={self._count})')
        else:
            if self._in_group:
                self.get_logger().info('Group left')
            self._in_group = False

    def _publish_count(self):
        msg = Int32()
        msg.data = int(self._count)
        self._pub.publish(msg)

        # no sim-time published


def main(args=None):
    rclpy.init(args=args)
    node = GroupEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
