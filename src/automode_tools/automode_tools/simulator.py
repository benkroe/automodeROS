#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from automode_interfaces.msg import RobotState
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker, MarkerArray
import math
import time

# Arena parameters
ARENA_SIZE = 4.0  # 4x4 meters
WALL_DISTANCE = ARENA_SIZE / 2.0  # Walls at +/- 2m from center

# Light source on left side
LIGHT_X = -1.5
LIGHT_Y = 0.0

# Floor colors
WHITE_X_MIN = -1.5
WHITE_X_MAX = 1.5
WHITE_Y_MIN = -0.5
WHITE_Y_MAX = 0.5
BLACK1_X = -1.0
BLACK1_Y = 1.0
BLACK1_RADIUS = 0.4
BLACK2_X = 1.0
BLACK2_Y = -1.0
BLACK2_RADIUS = 0.4

# Robot parameters
WHEEL_BASE = 0.3  # meters
MAX_SPEED = 0.5  # m/s
PROXIMITY_MAX_RANGE = 0.25  # 25cm
NEIGHBOR_DETECTION_RANGE = 1.5  # meters
SPEED_FACTOR = 1.0  # Simulation speed multiplier (real-time)

# Robot names
ROBOT_NAMES = ['tb1', 'tb2', 'tb3', 'tb4']

class Robot:
    def __init__(self, name, robot_id, x, y, theta):
        self.name = name
        self.id = robot_id
        self.x = x
        self.y = y
        self.theta = theta
        self.left_speed = 0.0
        self.right_speed = 0.0

class SimulatorNode(Node):
    def __init__(self):
        super().__init__('simulator_node')

        # Initialize robots
        self.robots = []
        initial_positions = [
            (-1.5, -1.5, 0.0),  # tb1
            (1.5, -1.5, math.pi/2),  # tb2
            (1.5, 1.5, math.pi),  # tb3
            (-1.5, 1.5, -math.pi/2)  # tb4
        ]
        for i, (name, pos) in enumerate(zip(ROBOT_NAMES, initial_positions)):
            robot = Robot(name, i+1, pos[0], pos[1], pos[2])
            self.robots.append(robot)

        # Publishers
        self.robot_state_pubs = {}
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)

        for robot in self.robots:
            topic = f'/{robot.name}/robotState'
            self.robot_state_pubs[robot.name] = self.create_publisher(RobotState, topic, 10)

        # Subscribers
        self.wheel_subs = {}
        for robot in self.robots:
            topic = f'/{robot.name}/wheels_speed'
            self.wheel_subs[robot.name] = self.create_subscription(
                Float32MultiArray, topic, 
                lambda msg, r=robot: self.wheels_callback(msg, r), 10)

        # Simulation timer (100Hz)
        self.timer = self.create_timer(0.01, self.simulation_step)

        # Simulation time
        self.sim_time = self.get_clock().now()

        self.get_logger().info('Simulator node initialized')

    def wheels_callback(self, msg, robot):
        if len(msg.data) >= 2:
            robot.left_speed = msg.data[0]
            robot.right_speed = msg.data[1]

    def simulation_step(self):
        # Update simulation time
        self.sim_time = self.sim_time + rclpy.duration.Duration(seconds=0.05 * SPEED_FACTOR)
        
        # Publish clock
        clock_msg = Clock()
        clock_msg.clock = self.sim_time.to_msg()
        self.clock_pub.publish(clock_msg)

        # Update robot positions
        for robot in self.robots:
            self.update_robot_position(robot)

        # Publish robot states and visualization
        self.publish_robot_states()
        self.publish_visualization()

    def update_robot_position(self, robot):
        # Clamp speeds
        left_speed = max(-MAX_SPEED, min(MAX_SPEED, robot.left_speed))
        right_speed = max(-MAX_SPEED, min(MAX_SPEED, robot.right_speed))

        # Differential drive kinematics
        linear_velocity = (left_speed + right_speed) / 2.0
        angular_velocity = (right_speed - left_speed) / WHEEL_BASE

        # Update position
        dt = 0.01
        if abs(angular_velocity) < 1e-6:
            # Straight line
            dx = linear_velocity * dt * math.cos(robot.theta)
            dy = linear_velocity * dt * math.sin(robot.theta)
        else:
            # Arc
            radius = linear_velocity / angular_velocity
            dtheta = angular_velocity * dt
            dx = radius * (math.sin(robot.theta + dtheta) - math.sin(robot.theta))
            dy = -radius * (math.cos(robot.theta + dtheta) - math.cos(robot.theta))
            robot.theta += dtheta

        # Normalize theta
        robot.theta = math.atan2(math.sin(robot.theta), math.cos(robot.theta))

        # Check wall collisions
        new_x = robot.x + dx
        new_y = robot.y + dy

        # Wall boundaries
        if abs(new_x) > WALL_DISTANCE:
            new_x = WALL_DISTANCE if new_x > 0 else -WALL_DISTANCE
        if abs(new_y) > WALL_DISTANCE:
            new_y = WALL_DISTANCE if new_y > 0 else -WALL_DISTANCE

        robot.x = new_x
        robot.y = new_y

    def get_floor_color(self, x, y):
        # White area
        if WHITE_X_MIN <= x <= WHITE_X_MAX and WHITE_Y_MIN <= y <= WHITE_Y_MAX:
            return "white"
        # Black circles
        if ((x - BLACK1_X)**2 + (y - BLACK1_Y)**2) <= BLACK1_RADIUS**2:
            return "black"
        if ((x - BLACK2_X)**2 + (y - BLACK2_Y)**2) <= BLACK2_RADIUS**2:
            return "black"
        return "gray"

    def calculate_proximity(self, robot):
        # Distance to walls
        dist_left = WALL_DISTANCE + robot.x
        dist_right = WALL_DISTANCE - robot.x
        dist_front = WALL_DISTANCE - robot.y
        dist_back = WALL_DISTANCE + robot.y

        # Find closest wall
        walls = [
            (dist_front, math.pi/2),  # front (positive y)
            (dist_back, -math.pi/2),  # back (negative y)
            (dist_left, math.pi),  # left (negative x)
            (dist_right, 0.0)  # right (positive x)
        ]

        min_dist = min(w[0] for w in walls)
        closest_wall = min(walls, key=lambda w: w[0])

        wall_angle = closest_wall[1]
        robot_angle = wall_angle - robot.theta
        robot_angle = math.atan2(math.sin(robot_angle), math.cos(robot_angle))

        # Only detect walls in front (within 180 degrees of forward direction, i.e., -90° to +90°)
        if abs(robot_angle) > math.pi / 2:
            return 0.0, 0.0  # Wall is behind, no detection

        if min_dist > PROXIMITY_MAX_RANGE:
            return 0.0, 0.0

        # Magnitude (higher when closer)
        magnitude = 1.0 - (min_dist / PROXIMITY_MAX_RANGE)

        return magnitude, robot_angle

    def calculate_light(self, robot):
        dx = LIGHT_X - robot.x
        dy = LIGHT_Y - robot.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance == 0:
            return 1.0, 0.0

        # Angle in world frame
        world_angle = math.atan2(dy, dx)
        # Angle in robot frame
        robot_angle = world_angle - robot.theta
        robot_angle = math.atan2(math.sin(robot_angle), math.cos(robot_angle))

        # Magnitude higher when closer (inverse square law approximation)
        magnitude = 1.0 / (1.0 + distance**2)
        magnitude = min(1.0, magnitude * 10.0)  # Scale up

        return magnitude, robot_angle

    def calculate_neighbors(self, robot):
        neighbors = []
        for other in self.robots:
            if other == robot:
                continue
            dx = other.x - robot.x
            dy = other.y - robot.y
            distance = math.sqrt(dx**2 + dy**2)
            if distance <= NEIGHBOR_DETECTION_RANGE:
                neighbors.append((dx, dy, distance))

        if not neighbors:
            return 0, 0.0

        # Average direction to neighbors
        total_x, total_y = 0.0, 0.0
        for dx, dy, dist in neighbors:
            weight = 1.0 / (1.0 + dist)  # Higher weight for closer neighbors
            total_x += dx * weight
            total_y += dy * weight

        world_angle = math.atan2(total_y, total_x)
        robot_angle = world_angle - robot.theta
        robot_angle = math.atan2(math.sin(robot_angle), math.cos(robot_angle))

        # Magnitude based on number of neighbors and average distance
        avg_dist = sum(dist for _, _, dist in neighbors) / len(neighbors)
        magnitude = len(neighbors) / 4.0 * (1.0 - avg_dist / NEIGHBOR_DETECTION_RANGE)
        magnitude = min(1.0, magnitude)

        return len(neighbors), robot_angle

    def publish_robot_states(self):
        stamp = self.sim_time.to_msg()
        for robot in self.robots:
            msg = RobotState()
            msg.robot_id = robot.id
            msg.stamp = stamp
            msg.floor_color = self.get_floor_color(robot.x, robot.y)
            msg.proximity_magnitude, msg.proximity_angle = self.calculate_proximity(robot)
            msg.light_magnitude, msg.light_angle = self.calculate_light(robot)
            msg.target_magnitude = 0.0  # Not implemented
            msg.target_position = 0.0
            msg.neighbour_count, msg.attraction_angle = self.calculate_neighbors(robot)

            self.robot_state_pubs[robot.name].publish(msg)

    def publish_visualization(self):
        marker_array = MarkerArray()

        # Arena boundary
        arena_marker = Marker()
        arena_marker.header.frame_id = "world"
        arena_marker.header.stamp = self.sim_time.to_msg()
        arena_marker.ns = "arena"
        arena_marker.id = 0
        arena_marker.type = Marker.CUBE
        arena_marker.action = Marker.ADD
        arena_marker.pose.position.x = 0.0
        arena_marker.pose.position.y = 0.0
        arena_marker.pose.position.z = 0.0
        arena_marker.pose.orientation.w = 1.0
        arena_marker.scale.x = ARENA_SIZE
        arena_marker.scale.y = ARENA_SIZE
        arena_marker.scale.z = 0.01
        arena_marker.color.r = 0.8
        arena_marker.color.g = 0.8
        arena_marker.color.b = 0.8
        arena_marker.color.a = 0.3
        marker_array.markers.append(arena_marker)

        # Floor colors
        # White strip
        white_marker = Marker()
        white_marker.header.frame_id = "world"
        white_marker.header.stamp = self.sim_time.to_msg()
        white_marker.ns = "floor"
        white_marker.id = 1
        white_marker.type = Marker.CUBE
        white_marker.action = Marker.ADD
        white_marker.pose.position.x = 0.0
        white_marker.pose.position.y = 0.0
        white_marker.pose.position.z = 0.0
        white_marker.pose.orientation.w = 1.0
        white_marker.scale.x = WHITE_X_MAX - WHITE_X_MIN
        white_marker.scale.y = WHITE_Y_MAX - WHITE_Y_MIN
        white_marker.scale.z = 0.005
        white_marker.color.r = 1.0
        white_marker.color.g = 1.0
        white_marker.color.b = 1.0
        white_marker.color.a = 0.5
        marker_array.markers.append(white_marker)

        # Black circles
        for i, (bx, by, br) in enumerate([(BLACK1_X, BLACK1_Y, BLACK1_RADIUS), (BLACK2_X, BLACK2_Y, BLACK2_RADIUS)]):
            black_marker = Marker()
            black_marker.header.frame_id = "world"
            black_marker.header.stamp = self.sim_time.to_msg()
            black_marker.ns = "floor"
            black_marker.id = 2 + i
            black_marker.type = Marker.CYLINDER
            black_marker.action = Marker.ADD
            black_marker.pose.position.x = bx
            black_marker.pose.position.y = by
            black_marker.pose.position.z = 0.0
            black_marker.pose.orientation.w = 1.0
            black_marker.scale.x = br * 2
            black_marker.scale.y = br * 2
            black_marker.scale.z = 0.005
            black_marker.color.r = 0.0
            black_marker.color.g = 0.0
            black_marker.color.b = 0.0
            black_marker.color.a = 0.5
            marker_array.markers.append(black_marker)

        # Light source
        light_marker = Marker()
        light_marker.header.frame_id = "world"
        light_marker.header.stamp = self.sim_time.to_msg()
        light_marker.ns = "light"
        light_marker.id = 10
        light_marker.type = Marker.SPHERE
        light_marker.action = Marker.ADD
        light_marker.pose.position.x = LIGHT_X
        light_marker.pose.position.y = LIGHT_Y
        light_marker.pose.position.z = 0.5
        light_marker.pose.orientation.w = 1.0
        light_marker.scale.x = 0.2
        light_marker.scale.y = 0.2
        light_marker.scale.z = 0.2
        light_marker.color.r = 1.0
        light_marker.color.g = 1.0
        light_marker.color.b = 0.0
        light_marker.color.a = 1.0
        marker_array.markers.append(light_marker)

        # Robots
        for robot in self.robots:
            # Robot body
            robot_marker = Marker()
            robot_marker.header.frame_id = "world"
            robot_marker.header.stamp = self.sim_time.to_msg()
            robot_marker.ns = "robots"
            robot_marker.id = robot.id * 10
            robot_marker.type = Marker.CYLINDER
            robot_marker.action = Marker.ADD
            robot_marker.pose.position.x = robot.x
            robot_marker.pose.position.y = robot.y
            robot_marker.pose.position.z = 0.1
            robot_marker.pose.orientation.w = 1.0
            robot_marker.scale.x = 0.2
            robot_marker.scale.y = 0.2
            robot_marker.scale.z = 0.1
            robot_marker.color.r = 0.0
            robot_marker.color.g = 1.0
            robot_marker.color.b = 0.0
            robot_marker.color.a = 1.0
            marker_array.markers.append(robot_marker)

            # Robot heading arrow
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "world"
            arrow_marker.header.stamp = self.sim_time.to_msg()
            arrow_marker.ns = "robots"
            arrow_marker.id = robot.id * 10 + 1
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = robot.x
            arrow_marker.pose.position.y = robot.y
            arrow_marker.pose.position.z = 0.2
            arrow_marker.pose.orientation.z = math.sin(robot.theta / 2)
            arrow_marker.pose.orientation.w = math.cos(robot.theta / 2)
            arrow_marker.scale.x = 0.3
            arrow_marker.scale.y = 0.05
            arrow_marker.scale.z = 0.05
            arrow_marker.color.r = 1.0
            arrow_marker.color.g = 0.0
            arrow_marker.color.b = 0.0
            arrow_marker.color.a = 1.0
            marker_array.markers.append(arrow_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()