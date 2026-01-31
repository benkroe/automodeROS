import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os

ROBOT_NAMES = ["tb1", "tb2", "tb3", "tb4"]

class PositionCollectorNode(Node):
    def __init__(self):
        super().__init__('position_collector')

        self.robot_positions = {robot: None for robot in ROBOT_NAMES}

        for robot in ROBOT_NAMES:
            self.create_subscription(
                Odometry,
                f'/{robot}/ground_truth_odom',
                lambda msg, r=robot: self._odom_cb(msg, r),
                10
            )

        # Timer to collect and write every 0.1s
        self.create_timer(0.1, self._collect_positions)

        # Open CSV file
        data_dir = '/home/ben/ros2_ws/automodeROS/data'
        os.makedirs(data_dir, exist_ok=True)
        self.csv_file_path = os.path.join(data_dir, 'positions_log.csv')
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header
        header = ['timestamp']
        for robot in ROBOT_NAMES:
            header.extend([f'{robot}_x', f'{robot}_y'])
        self.csv_writer.writerow(header)

    def _odom_cb(self, msg, robot):
        self.robot_positions[robot] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _collect_positions(self):
        now = self.get_clock().now().nanoseconds / 1e9  # seconds
        row = [now]
        for robot in ROBOT_NAMES:
            pos = self.robot_positions[robot]
            if pos:
                row.extend([pos[0], pos[1]])
            else:
                row.extend(['NaN', 'NaN'])
        self.csv_writer.writerow(row)
        # Flush to ensure written
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PositionCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()