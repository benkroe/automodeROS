#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from automode_interfaces.action import Behavior
import signal
import sys
import threading
import time
import os

class BehaviorTestNode(Node):
    def __init__(self):
        super().__init__('behavior_test_node')
        self._action_client = ActionClient(self, Behavior, 'behavior_action')
        self._current_goal_handle = None
        self._goal_lock = threading.Lock()
        self._running = True
        self._executor = None
        
        self.get_logger().info('Behavior Test Node started')
        self.get_logger().info('Available commands:')
        self.get_logger().info('  start <behavior_name> [param1] [param2] ... - Start a behavior')
        self.get_logger().info('  stop - Cancel current behavior')
        self.get_logger().info('  status - Show current status')
        self.get_logger().info('  quit - Exit the test node')

    def start_interactive_mode(self, executor):
        """Start the interactive command mode."""
        self._executor = executor
        
        # Start command input thread
        self._input_thread = threading.Thread(target=self._command_loop, daemon=True)
        self._input_thread.start()

    def _command_loop(self):
        """Main command input loop."""
        while self._running:
            try:
                command = input("\naction_test> ").strip().split()
                if not command:
                    continue
                    
                cmd = command[0].lower()
                
                if cmd == 'start':
                    if len(command) < 2:
                        print("Usage: start <behavior_name> [param1] [param2] ...")
                        continue
                    behavior_name = command[1]
                    params = command[2:] if len(command) > 2 else []
                    self._start_behavior(behavior_name, params)
                    
                elif cmd == 'stop':
                    self._cancel_current_goal()
                    
                elif cmd == 'status':
                    self._show_status()
                    
                elif cmd in ['quit', 'exit', 'q']:
                    print("Shutting down...")
                    self._running = False
                    if self._current_goal_handle:
                        print("Cancelling running behavior...")
                        self._cancel_current_goal()
                        time.sleep(0.5)
                    # Stop the executor
                    if self._executor:
                        self._executor.shutdown()
                    os._exit(0)
                    
                else:
                    print(f"Unknown command: {cmd}")
                    print("Available commands: start, stop, status, quit")
                    
            except EOFError:
                # Handle Ctrl+D
                print("\nShutting down...")
                self._running = False
                if self._executor:
                    self._executor.shutdown()
                os._exit(0)
            except KeyboardInterrupt:
                # Handle Ctrl+C
                print("\nShutting down...")
                self._running = False
                if self._executor:
                    self._executor.shutdown()
                os._exit(0)
            except Exception as e:
                print(f"Error in command loop: {e}")

    def _start_behavior(self, behavior_name, params):
        """Start a behavior with given parameters."""
        with self._goal_lock:
            if self._current_goal_handle is not None:
                print("A behavior is already running. Stop it first or wait for completion.")
                return
        
        print(f"Starting behavior '{behavior_name}' with params: {params}")
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            print("ERROR: Action server not available!")
            return

        goal_msg = Behavior.Goal()
        goal_msg.behavior_name = behavior_name
        goal_msg.params = params

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self._feedback_callback
        )
        
        # Wait for goal acceptance in a non-blocking way
        def goal_accepted_callback(future):
            try:
                goal_handle = future.result()
                with self._goal_lock:
                    if goal_handle.accepted:
                        self._current_goal_handle = goal_handle
                        print(f"✓ Goal accepted for behavior '{behavior_name}'")
                        
                        # Get result asynchronously
                        result_future = goal_handle.get_result_async()
                        result_future.add_done_callback(self._result_callback)
                    else:
                        print(f"✗ Goal rejected for behavior '{behavior_name}'")
                        self._current_goal_handle = None
            except Exception as e:
                print(f"Error in goal acceptance: {e}")
                with self._goal_lock:
                    self._current_goal_handle = None
        
        send_goal_future.add_done_callback(goal_accepted_callback)

    def _cancel_current_goal(self):
        """Cancel the currently running goal."""
        with self._goal_lock:
            if not self._current_goal_handle:
                print("No behavior currently running.")
                return
            
            print("Cancelling current behavior...")
            
            try:
                # Follow ROS 2 documentation pattern exactly
                future = self._current_goal_handle.cancel_goal_async()
                print(f"🔍 Cancel future created: {future}")
                future.add_done_callback(self.cancel_done)
                print("🔍 Cancel callback registered")
                
            except Exception as e:
                print(f"Exception in cancel: {e}")
                import traceback
                traceback.print_exc()

    def cancel_done(self, future):
        """Handle cancellation response - this is a class method like in ROS 2 examples."""
        print("🚨 CANCEL CALLBACK EXECUTED! 🚨")
        try:
            cancel_response = future.result()
            print(f"Cancel response received")
            print(f"   Return code: {cancel_response.return_code}")
            print(f"   Goals canceling: {len(cancel_response.goals_canceling)}")
            
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info('Cancel goal accepted')
                print("✓ Behavior cancellation accepted")
            else:
                print("✗ No goals were cancelled")
                
        except Exception as e:
            print(f"Error in cancel callback: {e}")
            import traceback
            traceback.print_exc()
        
        # Clear the goal handle
        with self._goal_lock:
            self._current_goal_handle = None
            print("Goal handle cleared")

    def _feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        print(f"📡 Feedback received")

    def _result_callback(self, future):
        """Handle the final result from the action."""
        try:
            result = future.result().result
            status = future.result().status
            
            # Map status codes to readable names
            status_names = {
                1: "EXECUTING",
                2: "CANCELING", 
                3: "SUCCEEDED",
                4: "CANCELED",
                5: "ABORTED"
            }
            
            status_name = status_names.get(status, f"UNKNOWN({status})")
            
            with self._goal_lock:
                if result.success:
                    print(f"✓ Behavior completed successfully!")
                    print(f"  Status: {status_name}")
                    print(f"  Message: {result.message}")
                else:
                    print(f"✗ Behavior failed or was cancelled.")
                    print(f"  Status: {status_name}")
                    print(f"  Message: {result.message}")
                
                self._current_goal_handle = None
                
        except Exception as e:
            print(f"Error processing result: {e}")
            with self._goal_lock:
                self._current_goal_handle = None

    def _show_status(self):
        """Show the current status of the action client."""
        with self._goal_lock:
            if self._current_goal_handle:
                print("📊 Status: Behavior is currently running")
                try:
                    goal_id = self._current_goal_handle.goal_id
                    print(f"   Goal ID: {goal_id}")
                except:
                    pass
            else:
                print("📊 Status: No behavior currently running")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ActionTestNode()
        
        # Use MultiThreadedExecutor to handle callbacks properly
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        # Start interactive mode
        node.start_interactive_mode(executor)
        
        # Spin in the main thread
        try:
            executor.spin()
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            executor.shutdown()
            node.destroy_node()
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()