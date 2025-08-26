#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_srvs.srv import Trigger
import threading
import json

# Import the action interface
try:
    from automode_interfaces.action import Behavior
except ImportError:
    print("Warning: automode_interfaces.action.Behavior not available")
    Behavior = None

class ActionTestNode(Node):
    def __init__(self):
        super().__init__('action_test_node')
        
        # Action client
        if Behavior is not None:
            self._action_client = ActionClient(self, Behavior, 'behavior_action')
        else:
            self._action_client = None
            self.get_logger().error("Behavior action not available")
            
        # Service to start behavior
        self._start_service = self.create_service(
            Trigger, 
            'test/start_behavior', 
            self._start_behavior_callback
        )
        
        # Service to stop behavior
        self._stop_service = self.create_service(
            Trigger, 
            'test/stop_behavior', 
            self._stop_behavior_callback
        )
        
        # Publisher to show current status
        self._status_pub = self.create_publisher(String, 'test/status', 10)
        
        # Current action state
        self._current_goal_handle = None
        self._current_behavior = None
        self._is_running = False
        
        # Default test behavior and parameters
        self._test_behavior = "stop"
        self._test_params = []
        
        # Status timer
        self.create_timer(1.0, self._publish_status)
        
        self.get_logger().info("Action Test Node started")
        self.get_logger().info("Services available:")
        self.get_logger().info("  - ros2 service call /test/start_behavior std_srvs/srv/Trigger")
        self.get_logger().info("  - ros2 service call /test/stop_behavior std_srvs/srv/Trigger")

    def _start_behavior_callback(self, request, response):
        """Start a test behavior."""
        if self._action_client is None:
            response.success = False
            response.message = "Action client not available"
            return response
            
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = "Behavior action server not available"
            return response
            
        if self._is_running:
            response.success = False
            response.message = f"Behavior '{self._current_behavior}' is already running. Stop it first."
            return response
            
        # Create goal
        goal = Behavior.Goal()
        goal.behavior_name = self._test_behavior
        goal.params = self._test_params
        
        self.get_logger().info(f"Starting behavior: {self._test_behavior} with params: {self._test_params}")
        
        # Send goal
        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        
        # Handle goal response in separate thread to avoid blocking service
        threading.Thread(target=self._handle_goal_response, args=(future,)).start()
        
        response.success = True
        response.message = f"Started behavior: {self._test_behavior}"
        return response
        
    def _stop_behavior_callback(self, request, response):
        """Stop the currently running behavior."""
        if not self._is_running or self._current_goal_handle is None:
            response.success = False
            response.message = "No behavior is currently running"
            return response
            
        self.get_logger().info(f"Stopping behavior: {self._current_behavior}")
        
        # Cancel the goal
        cancel_future = self._current_goal_handle.cancel_goal_async()
        
        response.success = True
        response.message = f"Stopped behavior: {self._current_behavior}"
        return response
        
    def _handle_goal_response(self, future):
        """Handle the goal response in a separate thread."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal was rejected")
                return
                
            self._current_goal_handle = goal_handle
            self._current_behavior = self._test_behavior
            self._is_running = True
            
            self.get_logger().info("Goal accepted")
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            result = result_future.result()
            
            # Handle result
            if result.result.success:
                self.get_logger().info(f"Behavior completed successfully: {result.result.message}")
            else:
                self.get_logger().warning(f"Behavior failed: {result.result.message}")
                
        except Exception as e:
            self.get_logger().error(f"Error handling goal response: {str(e)}")
        finally:
            # Reset state
            self._current_goal_handle = None
            self._current_behavior = None
            self._is_running = False
            
    def _feedback_callback(self, feedback_msg):
        """Handle feedback from the behavior action."""
        feedback = feedback_msg.feedback
        if hasattr(feedback, 'current_step') and hasattr(feedback, 'status'):
            self.get_logger().info(f"Feedback - Step: {feedback.current_step}, Status: {feedback.status}")
        else:
            self.get_logger().info(f"Feedback: {feedback}")
            
    def _publish_status(self):
        """Publish current status."""
        status = {
            "is_running": self._is_running,
            "current_behavior": self._current_behavior,
            "test_behavior": self._test_behavior,
            "test_params": self._test_params
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)
        
    def set_test_behavior(self, behavior_name: str, params: list = None):
        """Set the behavior to test."""
        self._test_behavior = behavior_name
        self._test_params = params or []
        self.get_logger().info(f"Test behavior set to: {behavior_name} with params: {self._test_params}")

def main(args=None):
    rclpy.init(args=args)
    
    node = ActionTestNode()
    
    # Example: Set different test behaviors
    # node.set_test_behavior("forward", ["speed=2.0", "duration=5.0"])
    # node.set_test_behavior("turn", ["angle=90", "direction=left"])
    node.set_test_behavior("stop", [])
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._is_running and node._current_goal_handle:
            node.get_logger().info("Cancelling running behavior before shutdown")
            node._current_goal_handle.cancel_goal_async()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()