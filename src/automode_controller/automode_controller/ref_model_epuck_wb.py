#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, String
from sensor_msgs.msg import Range, Illuminance
from automode_interfaces.msg import RobotState

import math
import time
from typing import List, Optional

WHEEL_BASE = 0.053  # meters, approximate e-puck wheel base


class EPuckReferenceNode(Node):
    def __init__(self):
        super().__init__('epuck_reference_node')

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 20)
        self._robot_state_pub = self.create_publisher(RobotState, 'robotState', 20)

        # Subscribe to actuator commands (from automode behaviours)
        self.create_subscription(Float32MultiArray, 'wheels_speed', self._wheels_speed_cb, 20)

        # Sensor storage (8 proximity sensors + 8 light sensors typical)
        self._ps_values: List[Optional[float]] = [None] * 8
        self._ls_values: List[Optional[float]] = [None] * 8

        # Dynamically create callbacks for ps0..ps7 and ls0..ls7
        for i in range(8):
            # some sims publish sensor_msgs/Range for proximity, others Float32
            self.create_subscription(Range, f'ps{i}', self._make_ps_range_cb(i), qos_profile_sensor_data)
            self.create_subscription(Illuminance, f'ls{i}', self._make_ls_illuminance_cb(i), qos_profile_sensor_data)

        # Neighbour/attraction info (receiver) - prefer namespaced topic in your sim
        # subscribe both to '/e_puck/receiver/data' and fallback un-namespaced 'receiver/data'
        try:
            self.create_subscription(String, '/e_puck/receiver/data', self._receiver_cb, qos_profile_sensor_data)
        except Exception:
            pass
        self.create_subscription(String, 'receiver/data', self._receiver_cb, qos_profile_sensor_data)

        # internal state
        self.robot_id = 1
        self.latest_wheels_speed = [0.0, 0.0]
        self.latest_cmd_vel = (0.0, 0.0)
        self.latest_floor_color = "gray"

        # receiver / attraction
        self._latest_neighbour_count = 0
        self._latest_attraction_angle = 0.0  # degrees from message; will publish radians

        # publish RobotState at 20 Hz
        self.create_timer(0.05, self._publish_robot_state)

        self.get_logger().info('EPuck reference node started')

    def _make_ls_illuminance_cb(self, idx: int):
        def cb(msg: Illuminance):
            try:
                # Convert illuminance to 0-100 scale like proximity sensors
                if msg.illuminance <= 0.0:
                    light_value = 0.0
                else:
                    # Scale to 0-100 range, assuming max 1000 lux
                    light_value = min(100.0, (msg.illuminance / 1000.0) * 100.0)
                
                self._ls_values[idx] = light_value
                self.get_logger().debug(f'LS{idx}: illuminance={msg.illuminance:.3f}, converted={light_value:.1f}/100')
            except Exception as e:
                self.get_logger().warning(f"Failed to read ls{idx} (Illuminance): {str(e)}")
        return cb
    def _make_ps_range_cb(self, idx: int):
        def cb(msg: Range):
            try:
                # Convert range to proximity value (0-100 scale)
                if msg.range <= msg.min_range:
                    proximity_value = 100.0  # closest = highest value
                elif msg.range >= msg.max_range:
                    proximity_value = 0.0    # too far = no detection
                else:
                    normalized_range = (msg.range - msg.min_range) / (msg.max_range - msg.min_range)
                    proximity_value = 100.0 * (1.0 - normalized_range)

                self._ps_values[idx] = proximity_value
                
            except Exception as e:
                self.get_logger().warning(f"Failed to read ps{idx} (Range): {str(e)}")
        return cb
    def _wheels_speed_cb(self, msg: Float32MultiArray):
        """Handle wheel speed commands from behaviors."""
        try:
            if len(msg.data) >= 2:
                # Store original wheel speeds (-1.0 to 1.0)
                self.latest_wheels_speed = [float(msg.data[0]), float(msg.data[1])]
                
                # Convert to Twist message
                twist = Twist()
                # Average for linear velocity, difference for angular
                twist.linear.x = float((msg.data[0] + msg.data[1]) / 2.0)
                twist.angular.z = float((msg.data[1] - msg.data[0]) / WHEEL_BASE)
                
                self.latest_cmd_vel = (twist.linear.x, twist.angular.z)
                self._cmd_vel_pub.publish(twist)
        except Exception as e:
            self.get_logger().warning(f"Failed to process wheel speeds: {str(e)}")

    def compute_proximity(self):
        """
        Compute proximity magnitude and average angle from ps sensors.
        PS7(-15°) and PS0(+15°) are in front
        PS1(+45°), PS2(+90°) on right
        PS3(+165°), PS4(-165°) in back
        PS5(-90°), PS6(-45°) on left
        Returns (magnitude, angle_degrees in [-180..180)).
        """
        if any(v is None for v in self._ps_values):
            return 0.0, 0.0

        # Sensor angles in degrees, matching physical layout
        angles = [15,    # PS0 (front-right)
                45,     # PS1 (right-front)
                90,     # PS2 (right)
                165,    # PS3 (back-right)
                -165,   # PS4 (back-left)
                -90,    # PS5 (left)
                -45,    # PS6 (left-front)
                -15]    # PS7 (front-left)
        
        weights = [max(0.0, v) for v in self._ps_values]
        
        # Find maximum sensor value
        max_weight = max(weights)
        if max_weight <= 0.0:
            return 0.0, 0.0

        # Calculate weighted direction using only significant readings
        significant_pairs = [(w, a) for w, a in zip(weights, angles) 
                            if w > 0.5 * max_weight]
        
        if not significant_pairs:
            return 0.0, 0.0
        
        # Calculate weighted average angle from significant readings
        total_weight = sum(w for w, _ in significant_pairs)
        x = sum(w * math.cos(math.radians(a)) for w, a in significant_pairs)
        y = sum(w * math.sin(math.radians(a)) for w, a in significant_pairs)
        
        # Calculate angle (-180 to +180 degrees)
        avg_ang = math.degrees(math.atan2(y, x))
        
        return max_weight, avg_ang

    def compute_light(self):
        """
        Compute light magnitude and angle from ls sensors.
        LS7(-15°) and LS0(+15°) are in front
        LS1(+45°), LS2(+90°) on right
        LS3(+165°), LS4(-165°) in back
        LS5(-90°), LS6(-45°) on left
        Returns (magnitude, angle_degrees in [-180..180)).
        """
        if any(v is None for v in self._ls_values):
            return 0.0, 0.0

        # Light sensor angles match proximity sensor layout
        angles = [15,    # LS0 (front-right)
                45,     # LS1 (right-front)
                90,     # LS2 (right)
                165,    # LS3 (back-right)
                -165,   # LS4 (back-left)
                -90,    # LS5 (left)
                -45,    # LS6 (left-front)
                -15]    # LS7 (front-left)
        
        values = [max(0.0, v) for v in self._ls_values]
        
        # Find maximum sensor value
        max_value = max(values)
        if max_value <= 0.0:
            return 0.0, 0.0

        # Use significant readings only (above 50% of max)
        significant_pairs = [(v, a) for v, a in zip(values, angles) 
                           if v > 0.5 * max_value]
        
        if not significant_pairs:
            return 0.0, 0.0
        
        # Calculate weighted direction
        total_weight = sum(v for v, _ in significant_pairs)
        x = sum(v * math.cos(math.radians(a)) for v, a in significant_pairs)
        y = sum(v * math.sin(math.radians(a)) for v, a in significant_pairs)
        
        # Calculate magnitude and angle (-180 to +180 degrees)
        mag = max_value  # Use maximum sensor reading as magnitude
        avg_ang = math.degrees(math.atan2(y, x))

        return mag, avg_ang

    def _publish_robot_state(self):
        msg = RobotState()
        # populate common fields
        try:
            msg.robot_id = self.robot_id
        except Exception:
            pass
        try:
            msg.stamp = self.get_clock().now().to_msg()
        except Exception:
            pass

        msg.neighbour_count = self._latest_neighbour_count
        # Keep attraction angle in degrees (don't convert to radians)
        try:
            msg.attraction_angle = self._latest_attraction_angle
        except Exception:
            msg.attraction_angle = 0.0

        msg.floor_color = self.latest_floor_color

        prox_mag, prox_ang_deg = self.compute_proximity()
        light_mag, light_ang_deg = self.compute_light()

        # Keep all angles in degrees
        msg.proximity_magnitude = prox_mag
        msg.proximity_angle = prox_ang_deg if prox_ang_deg is not None else 0.0
        msg.light_magnitude = light_mag
        msg.light_angle = light_ang_deg if light_ang_deg is not None else 0.0

        self._robot_state_pub.publish(msg)

        # Debug output now shows actual degrees being published
        self.get_logger().debug(
            f"\n=== Robot State ===\n"
            f"Robot ID: {msg.robot_id}\n"
            f"Time: {msg.stamp.sec}.{msg.stamp.nanosec:09d}\n"
            f"Proximity: mag={msg.proximity_magnitude:.3f}, angle={msg.proximity_angle:.1f}°\n"
            f"Light: mag={msg.light_magnitude:.3f}, angle={msg.light_angle:.1f}°\n"
            f"Neighbors: count={msg.neighbour_count}, attraction={msg.attraction_angle:.1f}°\n"
            f"Floor: {msg.floor_color}\n"
            f"Wheels: L={self.latest_wheels_speed[0]:.3f}, R={self.latest_wheels_speed[1]:.3f}\n"
            f"Cmd_vel: lin={self.latest_cmd_vel[0]:.3f}, ang={self.latest_cmd_vel[1]:.3f}\n"
            f"==================="
        )

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EPuckReferenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Shutting down due to interrupt or external shutdown')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()