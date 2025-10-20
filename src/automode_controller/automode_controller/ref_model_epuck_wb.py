#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, String
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
            self.create_subscription(Float32, f'ps{i}', self._make_ps_cb(i), qos_profile_sensor_data)
            self.create_subscription(Float32, f'ls{i}', self._make_ls_cb(i), qos_profile_sensor_data)

        # Optional neighbour/attraction info (receiver)
        self._latest_neighbour_count = 0
        self._latest_attraction_angle = 0.0
        # try both local and namespaced receiver topics
        self.create_subscription(String, 'receiver/data', self._receiver_cb, qos_profile_sensor_data)
        try:
            self.create_subscription(String, '/e_puck/receiver/data', self._receiver_cb, qos_profile_sensor_data)
        except Exception:
            pass

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

        # publish RobotState at 20 Hz
        self.create_timer(0.05, self._publish_robot_state)

        self.get_logger().info('EPuck reference node started')

    def _make_ps_cb(self, idx: int):
        def cb(msg: Float32):
            try:
                self._ps_values[idx] = float(msg.data)
            except Exception:
                self.get_logger().warning(f"Failed to read ps{idx}")
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
            # record zero in smoothing history
            self._prox_mag_hist.append(0.0)
            self._prox_ang_hist.append(0.0)
        else:
            x = sum(w * math.cos(math.radians(a)) for w, a in zip(weights, angles))
            y = sum(w * math.sin(math.radians(a)) for w, a in zip(weights, angles))
            avg_ang = (math.degrees(math.atan2(y, x))) % 360
            mag = total / len(self._ps_values)
            self._prox_mag_hist.append(mag)
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
        # Expect [left, right]
        if not msg.data or len(msg.data) < 2:
            self.get_logger().warning("wheels_speed must contain two values [left, right]")
            return
        left = float(msg.data[0])
        right = float(msg.data[1])
        self.latest_wheels_speed = [left, right]

        # convert differential wheel speeds to Twist (linear.x, angular.z)
        twist = Twist()
        # linear velocity approx (left + right) / 2
        twist.linear.x = (left + right) / 2.0
        # angular velocity approx (right - left) / WHEEL_BASE
        if abs(WHEEL_BASE) < 1e-6:
            twist.angular.z = 0.0
        else:
            twist.angular.z = (right - left) / WHEEL_BASE

        self.latest_cmd_vel = (twist.linear.x, twist.angular.z)
        self._cmd_vel_pub.publish(twist)

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
        msg.attraction_angle = self._latest_attraction_angle
        msg.floor_color = self.latest_floor_color

        msg.proximity_magnitude, msg.proximity_angle = self.compute_proximity()
        msg.light_magnitude, msg.light_angle = self.compute_light()

        self._robot_state_pub.publish(msg)

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