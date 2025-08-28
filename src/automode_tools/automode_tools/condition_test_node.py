#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from automode_interfaces.action import Condition
import signal
import sys
import threading
import time
import os
from typing import Dict, Any

class ConditionTestNode(Node):
    def __init__(self):
        super().__init__('condition_test_node')
        self._action_client = ActionClient(self, Condition, 'condition_action')
        self._active_goals: Dict[str, Any] = {}  # goal_id -> goal_info
        self._goal_lock = threading.Lock()
        self._running = True
        self._executor = None
        self._next_goal_id = 1
        
        self.get_logger().info('Condition Test Node started')
        self.get_logger().info('Available commands:')
        self.get_logger().info('  start <condition_name> [param1] [param2] ... - Start a condition')
        self.get_logger().info('  stop <goal_id> - Cancel specific condition')
        self.get_logger().info('  stop all - Cancel all conditions')
        self.get_logger().info('  list - Show all running conditions')
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
                command = input("\ncondition_test> ").strip().split()
                if not command:
                    continue
                    
                cmd = command[0].lower()
                
                if cmd == 'start':
                    if len(command) < 2:
                        print("Usage: start <condition_name> [param1] [param2] ...")
                        continue
                    condition_name = command[1]
                    params = command[2:] if len(command) > 2 else []
                    self._start_condition(condition_name, params)
                    
                elif cmd == 'stop':
                    if len(command) < 2:
                        print("Usage: stop <goal_id> or stop all")
                        continue
                    if command[1].lower() == 'all':
                        self._cancel_all_goals()
                    else:
                        try:
                            goal_id = int(command[1])
                            self._cancel_goal(goal_id)
                        except ValueError:
                            print("Goal ID must be a number or 'all'")
                    
                elif cmd == 'list':
                    self._list_conditions()
                    
                elif cmd == 'status':
                    self._show_status()
                    
                elif cmd in ['quit', 'exit', 'q']:
                    print("Shutting down...")
                    self._running = False
                    self._cancel_all_goals()
                    time.sleep(0.5)
                    if self._executor:
                        self._executor.shutdown()
                    os._exit(0)
                    
                else:
                    print(f"Unknown command: {cmd}")
                    print("Available commands: start, stop, list, status, quit")
                    
            except EOFError:
                print("\nShutting down...")
                self._running = False
                if self._executor:
                    self._executor.shutdown()
                os._exit(0)
            except KeyboardInterrupt:
                print("\nShutting down...")
                self._running = False
                if self._executor:
                    self._executor.shutdown()
                os._exit(0)
            except Exception as e:
                print(f"Error in command loop: {e}")

    def _start_condition(self, condition_name, params):
        """Start a condition with given parameters."""
        print(f"Starting condition '{condition_name}' with params: {params}")
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            print("ERROR: Condition action server not available!")
            return

        goal_msg = Condition.Goal()
        goal_msg.condition_name = condition_name
        goal_msg.params = params

        # Assign local goal ID for tracking
        with self._goal_lock:
            local_goal_id = self._next_goal_id
            self._next_goal_id += 1

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=lambda feedback: self._feedback_callback(feedback, local_goal_id)
        )
        
        # Wait for goal acceptance in a non-blocking way
        def goal_accepted_callback(future):
            try:
                goal_handle = future.result()
                with self._goal_lock:
                    if goal_handle.accepted:
                        self._active_goals[local_goal_id] = {
                            'goal_handle': goal_handle,
                            'condition_name': condition_name,
                            'params': params,
                            'start_time': time.time()
                        }
                        print(f"✓ Goal {local_goal_id} accepted for condition '{condition_name}'")
                        
                        # Get result asynchronously
                        result_future = goal_handle.get_result_async()
                        result_future.add_done_callback(lambda future: self._result_callback(future, local_goal_id))
                    else:
                        print(f"✗ Goal rejected for condition '{condition_name}'")
            except Exception as e:
                print(f"Error in goal acceptance: {e}")
        
        send_goal_future.add_done_callback(goal_accepted_callback)

    def _cancel_goal(self, local_goal_id):
        """Cancel a specific goal."""
        with self._goal_lock:
            if local_goal_id not in self._active_goals:
                print(f"No condition with ID {local_goal_id} found.")
                return
            
            goal_info = self._active_goals[local_goal_id]
            condition_name = goal_info['condition_name']
            goal_handle = goal_info['goal_handle']
            
        print(f"Cancelling condition {local_goal_id} ({condition_name})...")
        
        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(lambda future: self._cancel_done(future, local_goal_id))
        except Exception as e:
            print(f"Exception in cancel: {e}")

    def _cancel_all_goals(self):
        """Cancel all running goals."""
        with self._goal_lock:
            if not self._active_goals:
                print("No conditions currently running.")
                return
            
            active_ids = list(self._active_goals.keys())
        
        print(f"Cancelling all {len(active_ids)} running conditions...")
        for goal_id in active_ids:
            self._cancel_goal(goal_id)

    def _cancel_done(self, future, local_goal_id):
        """Handle cancellation response for a specific goal."""
        try:
            cancel_response = future.result()
            
            with self._goal_lock:
                condition_name = self._active_goals.get(local_goal_id, {}).get('condition_name', 'unknown')
            
            if len(cancel_response.goals_canceling) > 0:
                print(f"✓ Condition {local_goal_id} ({condition_name}) cancellation accepted")
            else:
                print(f"✗ Condition {local_goal_id} ({condition_name}) could not be cancelled")
                
        except Exception as e:
            print(f"Error in cancel callback for goal {local_goal_id}: {e}")

    def _feedback_callback(self, feedback_msg, local_goal_id):
        """Handle feedback from a specific condition."""
        with self._goal_lock:
            if local_goal_id in self._active_goals:
                condition_name = self._active_goals[local_goal_id]['condition_name']
                print(f"📡 [{local_goal_id}] {condition_name}: {getattr(feedback_msg.feedback, 'current_status', 'No status')}")

    def _result_callback(self, future, local_goal_id):
        """Handle the final result from a specific condition."""
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
                condition_info = self._active_goals.get(local_goal_id, {})
                condition_name = condition_info.get('condition_name', 'unknown')
                start_time = condition_info.get('start_time', time.time())
                duration = time.time() - start_time
                
                if result.success:
                    print(f"✓ Condition {local_goal_id} ({condition_name}) completed successfully!")
                    print(f"  Status: {status_name}")
                    print(f"  Message: {result.message}")
                    print(f"  Duration: {duration:.1f}s")
                else:
                    print(f"✗ Condition {local_goal_id} ({condition_name}) failed or was cancelled.")
                    print(f"  Status: {status_name}")
                    print(f"  Message: {result.message}")
                    print(f"  Duration: {duration:.1f}s")
                
                # Remove from active goals
                if local_goal_id in self._active_goals:
                    del self._active_goals[local_goal_id]
                
        except Exception as e:
            print(f"Error processing result for goal {local_goal_id}: {e}")
            with self._goal_lock:
                if local_goal_id in self._active_goals:
                    del self._active_goals[local_goal_id]

    def _list_conditions(self):
        """List all currently running conditions."""
        with self._goal_lock:
            if not self._active_goals:
                print("📊 No conditions currently running")
                return
            
            print(f"📊 Running conditions ({len(self._active_goals)}):")
            current_time = time.time()
            for goal_id, info in self._active_goals.items():
                condition_name = info['condition_name']
                params = info['params']
                start_time = info['start_time']
                duration = current_time - start_time
                
                print(f"  [{goal_id}] {condition_name}")
                print(f"      Params: {params}")
                print(f"      Running for: {duration:.1f}s")

    def _show_status(self):
        """Show the current status of all conditions."""
        with self._goal_lock:
            count = len(self._active_goals)
            if count == 0:
                print("📊 Status: No conditions currently running")
            else:
                print(f"📊 Status: {count} condition(s) currently running")
                for goal_id, info in self._active_goals.items():
                    condition_name = info['condition_name']
                    duration = time.time() - info['start_time']
                    print(f"   [{goal_id}] {condition_name} (running {duration:.1f}s)")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ConditionTestNode()
        
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