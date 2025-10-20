#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, String
from sensor_msgs.msg import Range
from automode_interfaces.msg import RobotState

import math
import time
from typing import List, Optional

WHEEL_BASE = 0.053  # meters, approximate e-puck wheel base
PROXIMITY_SMOOTH_WINDOW = 3
PROXIMITY_HYSTERESIS = 0.02


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
            self.create_subscription(Float32, f'ls{i}', self._make_ls_cb(i), qos_profile_sensor_data)

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

        # proximity smoothing/hysteresis
        self._prox_mag_hist: List[float] = []
        self._prox_ang_hist: List[float] = []
        self._prox_smooth_window = PROXIMITY_SMOOTH_WINDOW
        self._prox_mag_last = 0.0
        self._prox_ang_last = 0.0
        self._prox_hysteresis = PROXIMITY_HYSTERESIS

        # receiver / attraction
        self._latest_neighbour_count = 0
        self._latest_attraction_angle = 0.0  # degrees from message; will publish radians

        # publish RobotState at 20 Hz
        self.create_timer(0.05, self._publish_robot_state)

        self.get_logger().info('EPuck reference node started')

    def _make_ps_cb(self, idx: int):
        def cb(msg: Float32):
            try:
                self._ps_values[idx] = float(msg.data)
                # Debug output
                self.get_logger().debug(f'PS{idx}: value={msg.data:.3f} (Float32)')
            except Exception as e:
                self.get_logger().warning(f"Failed to read ps{idx} (Float32): {str(e)}")
        return cb
    def _make_ps_range_cb(self, idx: int):
        def cb(msg: Range):
            try:
                # Convert range to proximity value (0-100 scale)
                # The closer the object, the higher the value
                if msg.range <= msg.min_range:
                    proximity_value = 100.0  # closest = highest value
                elif msg.range >= msg.max_range:
                    proximity_value = 0.0    # too far = no detection
                else:
                    # Linear scaling inverted (closer = higher value)
                    # Scale from min_range..max_range to 100..0
                    normalized_range = (msg.range - msg.min_range) / (msg.max_range - msg.min_range)
                    proximity_value = 100.0 * (1.0 - normalized_range)

                self._ps_values[idx] = proximity_value
                
                # Debug output
                self.get_logger().debug(
                    f'PS{idx}: range={msg.range:.3f}m, max={msg.max_range:.3f}m, ' +
                    f'converted={proximity_value:.1f}/100'
                )
            except Exception as e:
                self.get_logger().warning(f"Failed to read ps{idx} (Range): {str(e)}")
        return cb

    def _make_ls_cb(self, idx: int):
        def cb(msg: Float32):
            try:
                self._ls_values[idx] = float(msg.data)
            except Exception:
                self.get_logger().warning(f"Failed to read ls{idx}")
        return cb

    def _receiver_cb(self, msg: String):
        # expected formats: "angle,count" or "count"
        try:
            parts = msg.data.split(',')
            if len(parts) >= 2:
                ang = float(parts[0])
                cnt = int(parts[1])
                self._latest_attraction_angle = ang
                self._latest_neighbour_count = cnt
            else:
                self._latest_neighbour_count = int(parts[0])
        except Exception:
            self.get_logger().debug(f"Could not parse receiver data: {msg.data}")

    def compute_proximity(self):
        """
        Compute proximity magnitude and average angle from ps sensors.
        Angles assume index 0 is front and increase clockwise.
        Returns (magnitude, angle_degrees [0..360)).
        """
        if any(v is None for v in self._ps_values):
            return 0.0, 0.0

        # sensor angles in degrees for indexes 0..7 (front = 0)
        angles = [0, -45, -90, -135, 180, 135, 90, 45]
        weights = [max(0.0, v) for v in self._ps_values]
        total = sum(weights)

        if total <= 0.0:
            self._prox_mag_hist.append(0.0)
            self._prox_ang_hist.append(0.0)
            return 0.0, 0.0

        # Find maximum sensor value for magnitude
        max_weight = max(weights)
        # Calculate weighted direction
        x = sum(w * math.cos(math.radians(a)) for w, a in zip(weights, angles))
        y = sum(w * math.sin(math.radians(a)) for w, a in zip(weights, angles))
        avg_ang = (math.degrees(math.atan2(y, x))) % 360
        
        self._prox_mag_hist.append(max_weight)
        self._prox_ang_hist.append(avg_ang)

        # keep smoothing window
        if len(self._prox_mag_hist) > self._prox_smooth_window:
            self._prox_mag_hist.pop(0)
            self._prox_ang_hist.pop(0)

        mag_smoothed = sum(self._prox_mag_hist) / max(1, len(self._prox_mag_hist))
        sin_sum = sum(math.sin(math.radians(a)) for a in self._prox_ang_hist)
        cos_sum = sum(math.cos(math.radians(a)) for a in self._prox_ang_hist)
        if sin_sum == 0 and cos_sum == 0:
            ang_smoothed = 0.0
        else:
            ang_smoothed = (math.degrees(math.atan2(sin_sum, cos_sum))) % 360

        # apply simple hysteresis to avoid jitter
        if abs(mag_smoothed - self._prox_mag_last) > self._prox_hysteresis:
            self._prox_mag_last = mag_smoothed
        if abs(ang_smoothed - self._prox_ang_last) > self._prox_hysteresis:
            self._prox_ang_last = ang_smoothed

        return self._prox_mag_last, self._prox_ang_last

    def compute_light(self):
        """
        Compute light magnitude and angle from ls sensors.
        Returns (magnitude, angle_degrees [0..360)).
        """
        if any(v is None for v in self._ls_values):
            return 0.0, 0.0

        angles = [0, -45, -90, -135, 180, 135, 90, 45]
        values = [max(0.0, v) for v in self._ls_values]
        total = sum(values)
        if total <= 0.0:
            return 0.0, 0.0

        x = sum(v * math.cos(math.radians(a)) for v, a in zip(values, angles))
        y = sum(v * math.sin(math.radians(a)) for v, a in zip(values, angles))
        mag = math.sqrt(x * x + y * y)
        ang = (math.degrees(math.atan2(y, x))) % 360
        return mag, ang

    def _wheels_speed_cb(self, msg: Float32MultiArray):
        if not msg.data or len(msg.data) < 2:
            self.get_logger().warning("wheels_speed must contain two values [left, right]")
            return
        
        try:
            # Ensure we're getting proper float values
            left = float(msg.data[0])
            right = float(msg.data[1])
            
            # Store original wheel speeds
            self.latest_wheels_speed = [left, right]

            # Scale wheel speeds from behavior range (-1 to 1) to Twist range
            scale = 0.1  # Max linear velocity in m/s
            
            # convert differential wheel speeds to Twist (linear.x, angular.z)
            twist = Twist()
            twist.linear.x = scale * float((left + right) / 2.0)
            twist.angular.z = scale * float((right - left) / WHEEL_BASE)

            self.latest_cmd_vel = (twist.linear.x, twist.angular.z)
            self._cmd_vel_pub.publish(twist)
            
        except (TypeError, ValueError) as e:
            self.get_logger().error(f"Invalid wheel speed values: {e}")

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
        # convert attraction angle from degrees -> radians for behaviours
        try:
            msg.attraction_angle = math.radians(self._latest_attraction_angle)
        except Exception:
            msg.attraction_angle = 0.0

        msg.floor_color = self.latest_floor_color

        prox_mag, prox_ang_deg = self.compute_proximity()
        light_mag, light_ang_deg = self.compute_light()

        msg.proximity_magnitude = prox_mag
        msg.proximity_angle = math.radians(prox_ang_deg) if prox_ang_deg is not None else 0.0
        msg.light_magnitude = light_mag
        msg.light_angle = math.radians(light_ang_deg) if light_ang_deg is not None else 0.0

        self._robot_state_pub.publish(msg)

                # Add debug output for robot state
        self.get_logger().debug(
            f"\n=== Robot State ===\n"
            f"Robot ID: {msg.robot_id}\n"
            f"Time: {msg.stamp.sec}.{msg.stamp.nanosec:09d}\n"
            f"Proximity: mag={msg.proximity_magnitude:.3f}, angle={math.degrees(msg.proximity_angle):.1f}°\n"
            f"Light: mag={msg.light_magnitude:.3f}, angle={math.degrees(msg.light_angle):.1f}°\n"
            f"Neighbors: count={msg.neighbour_count}, attraction={math.degrees(msg.attraction_angle):.1f}°\n"
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