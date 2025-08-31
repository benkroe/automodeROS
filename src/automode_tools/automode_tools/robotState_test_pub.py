#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from automode_interfaces.msg import RobotState
from builtin_interfaces.msg import Time
import threading
import time
from rclpy.executors import ExternalShutdownException

class RobotStateTestPublisher(Node):
    def __init__(self):
        super().__init__('robotstate_test_pub')
        
        # Publisher
        self._pub = self.create_publisher(RobotState, 'robotState', 10)
        
        # Timer to publish every second
        self.create_timer(1.0, self.publish_state)
        
        # Default robot state values (easy to change)
        self.robot_id = 1
        self.ground_black_floor = False
        self.proximity_magnitude = 0.0
        self.proximity_angle = 0.0
        self.light_magnitude = 0.0
        self.light_angle = 0.0
        self.attraction_angle = 0.0
        self.neighbour_count = 0
        
        self.get_logger().info("RobotState Test Publisher started")
        self.get_logger().info("Publishing to /robotState every 1 second")
        self.print_help()
        
        # Start interactive input in separate thread
        self.input_thread = threading.Thread(target=self.interactive_input)
        self.input_thread.daemon = True
        self.input_thread.start()
    
    def publish_state(self):
        """Publish the current robot state."""
        msg = RobotState()
        
        # Set timestamp
        now = self.get_clock().now()
        msg.stamp = now.to_msg()
        
        # Set values
        msg.robot_id = self.robot_id
        msg.ground_black_floor = self.ground_black_floor
        msg.proximity_magnitude = self.proximity_magnitude
        msg.proximity_angle = self.proximity_angle
        msg.light_magnitude = self.light_magnitude
        msg.light_angle = self.light_angle
        msg.attraction_angle = self.attraction_angle
        msg.neighbour_count = self.neighbour_count
        
        self._pub.publish(msg)
        
        # Log current values (every 5 seconds to avoid spam)
        if int(time.time()) % 5 == 0:
            self.get_logger().info(f'Publishing: proximity={self.proximity_magnitude:.1f}@{self.proximity_angle:.0f}°, neighbors={self.neighbour_count}')
    
    def print_help(self):
        """Print available commands."""
        print("\n=== RobotState Test Publisher ===")
        print("Commands:")
        print("  pm <value>  - Set proximity_magnitude (e.g., 'pm 0.8')")
        print("  pa <value>  - Set proximity_angle in degrees (e.g., 'pa -45')")
        print("  nc <value>  - Set neighbour_count (e.g., 'nc 3')")
        print("  bf <0|1>    - Set ground_black_floor (e.g., 'bf 1')")
        print("  lm <value>  - Set light_magnitude (e.g., 'lm 0.9')")
        print("  la <value>  - Set light_angle in degrees (e.g., 'la 90')")
        print("  show        - Show current values")
        print("  help        - Show this help")
        print("  quit        - Exit")
        print("\nType commands and press Enter:")
    
    def interactive_input(self):
        """Handle interactive input in separate thread."""
        while rclpy.ok():
            try:
                cmd = input().strip().lower()
                if not cmd:
                    continue
                    
                if cmd == 'quit' or cmd == 'q':
                    rclpy.shutdown()
                    break
                elif cmd == 'help' or cmd == 'h':
                    self.print_help()
                elif cmd == 'show' or cmd == 's':
                    self.show_current_values()
                else:
                    self.process_command(cmd)
                    
            except (KeyboardInterrupt, EOFError):
                rclpy.shutdown()
                break
            except Exception as e:
                print(f"Error: {e}")
    
    def show_current_values(self):
        """Display current robot state values."""
        print(f"\n=== Current Values ===")
        print(f"robot_id: {self.robot_id}")
        print(f"ground_black_floor: {self.ground_black_floor}")
        print(f"proximity_magnitude: {self.proximity_magnitude}")
        print(f"proximity_angle: {self.proximity_angle}°")
        print(f"light_magnitude: {self.light_magnitude}")
        print(f"light_angle: {self.light_angle}°")
        print(f"attraction_angle: {self.attraction_angle}°")
        print(f"neighbour_count: {self.neighbour_count}")
        print()
    
    def process_command(self, cmd):
        """Process user commands."""
        parts = cmd.split()
        if len(parts) != 2:
            print("Invalid command format. Use 'help' for usage.")
            return
            
        command, value_str = parts
        
        try:
            if command == 'pm':
                self.proximity_magnitude = float(value_str)
                print(f"Set proximity_magnitude = {self.proximity_magnitude}")
            elif command == 'pa':
                self.proximity_angle = float(value_str)
                print(f"Set proximity_angle = {self.proximity_angle}°")
            elif command == 'nc':
                self.neighbour_count = int(value_str)
                print(f"Set neighbour_count = {self.neighbour_count}")
            elif command == 'bf':
                self.ground_black_floor = bool(int(value_str))
                print(f"Set ground_black_floor = {self.ground_black_floor}")
            elif command == 'lm':
                self.light_magnitude = float(value_str)
                print(f"Set light_magnitude = {self.light_magnitude}")
            elif command == 'la':
                self.light_angle = float(value_str)
                print(f"Set light_angle = {self.light_angle}°")
            else:
                print(f"Unknown command: {command}")
        except ValueError:
            print(f"Invalid value: {value_str}")

def main(args=None):
    rclpy.init(args=args)
    
    node = RobotStateTestPublisher()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\nShutting down due to external request or keyboard interrupt...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()