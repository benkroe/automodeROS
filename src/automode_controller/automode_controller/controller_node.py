#!/usr/bin/env python3

# TODO:
#   1. All methods for action client with multiple action requests (possible -should be)
#   2. Datastructure for the FSM --> other file finished
#   3. Creator of the FSM --> also other field not yet done but simple example
#   4. Start and stop of actions
#   5. Switch of states


from action_msgs.msg import GoalStatus
from automode_interfaces.action import Condition
from automode_interfaces.action import Behavior
from .fsm import create_simple_fsm
from .fsm import FSM


import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from typing import Dict, Any




class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('fsm_config', 'simple_fsm')

        _fsm_config = self.get_parameter('fsm_config').get_parameter_value().string_value
        self._fsm: FSM = self._create_fsm_from_config(_fsm_config)


        self._action_client_behavior = ActionClient(self, Behavior, 'behavior_action')
        self._action_client_condition = ActionClient(self, Condition, 'condition_action')

        self._active_behavior_goal = None
        self._active_condition_goals: Dict[str, Any] = {}


    def start_state(self):
        # starts the action of the current state and all necessary conditions
        try:   
            self.start_behavior_action(self._fsm.get_current_state())

            for edge in self._fsm.get_outgoing_edges(self._fsm.get_current_state().name):
                self.start_condition_action(edge)
        except Exception as e:
            self.get_logger().error(f"Error starting state: {e}")

    def stop_state(self, state):
        # stops the action of the current state and all necessary conditions
        try:
            self.stop_behavior_action(state)

            for edge in self._fsm.get_outgoing_edges(state.name):
                self.stop_condition_action(edge)
        except Exception as e:
            self.get_logger().error(f"Error stopping state: {e}")

    def stop_behavior_action(self, state):
        if not self._action_client_behavior.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Behavior action server not available!")
            return
        if self._active_behavior_goal:
            self._active_behavior_goal.cancel_goal_async()
            self._active_behavior_goal = None
            self.get_logger().info(f"Stopped behavior: {state.behavior_name}")

    def stop_condition_action(self, edge):
        if not self._action_client_condition.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Condition action server not available!")
            return
        if edge.condition_name in self._active_condition_goals:
            self._active_condition_goals[edge.condition_name].cancel_goal_async()
            del self._active_condition_goals[edge.condition_name]
            self.get_logger().info(f"Stopped condition: {edge.condition_name}")

    def start_behavior_action(self, state):
        if not self._action_client_behavior.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Behavior action server not available!")
            return
        
        goal_msg = Behavior.Goal()
        goal_msg.behavior_name = state.behavior_name
        goal_msg.params = state.behavior_params

        send_goal_future = self._action_client_behavior.send_goal_async(goal_msg, feedback_callback=self.feedback_callback_behavior)
        send_goal_future.add_done_callback(self.goal_response_behavior)

        self.get_logger().info(f"Started behavior: {state.behavior_name} with params: {state.behavior_params}")


    def start_condition_action(self, edge):
        if not self._action_client_condition.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Condition action server not available!")
            return
        
        goal_msg = Condition.Goal()
        goal_msg.condition_name = edge.condition_name
        goal_msg.params = edge.condition_params

        send_goal_future = self._action_client_condition.send_goal_async(
            goal_msg, 
            feedback_callback=lambda feedback, edge_ref=edge: 
                self.feedback_callback_condition(feedback, edge_ref)
        )

        send_goal_future.add_done_callback(lambda future, edge_ref=edge: self.goal_response_condition(future, edge_ref))
        self.get_logger().info(f"Started condition: {edge.condition_name} with params: {edge.condition_params}")
    

    def goal_response_behavior(self, future):
        # We need that primarily to set the _active_behavior_goals
        try: 
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Behavior goal accepted :)')
                self._active_behavior_goal = goal_handle

        except Exception as e:
            self.get_logger().error(f"Error in goal response: {e}")

    def goal_response_condition(self, future, edge):
        # We need that primarily to set the _active_condition_goals
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info(f'Condition goal {edge.condition_name} accepted :)')
                self._active_condition_goals[edge.condition_name] = goal_handle


        except Exception as e:
            self.get_logger().error(f"Error in goal response: {e}")

    def feedback_callback_behavior(self, feedback):
        # Actually we don't need to do anything with the feedback maybe error handling
        pass
    
    def feedback_callback_condition(self, feedback, edge):
        try:
            # Ignore feedback from conditions that have been stopped
            if edge.condition_name not in self._active_condition_goals:
                self.get_logger().debug(f"Ignoring feedback from cancelled condition: {edge.condition_name}")
                return

            condition_result = feedback.feedback
            if condition_result.condition_met:
                self.get_logger().info(f"Condition {edge.condition_name} met: {condition_result.condition_met}")
                self._transition_to_state(edge.target_state)

        except Exception as e:
            self.get_logger().error(f"Error occurred during condition feedback processing: {e}")

    def _transition_to_state(self, new_state: str):
        # switch to the new state
        try:
            old_state = self._fsm.get_current_state()

            if self._fsm.transition_to(new_state):
                self.get_logger().info(f"Transitioning from {old_state.behavior_name} to {new_state}")
                
                # Stop old state
                self.stop_state(old_state)
                
                # Start new state
                self.start_state()
            else:
                self.get_logger().error(f"Failed to transition to state: {new_state}")
        except Exception as e:
            self.get_logger().error(f"Error occurred during state transition: {e}")

    def _create_fsm_from_config(self, _fsm_config):
        # Create FSM from configuration by getting descriptions from behavior and condition nodes
        try:
            # Import required services
            from std_srvs.srv import Trigger
            import json
            
            # Create service clients
            behavior_list_client = self.create_client(Trigger, 'behaviors/list_srv')
            condition_list_client = self.create_client(Trigger, 'conditions/list_srv')
            
            # Get behavior descriptions
            behavior_descriptions = None
            if behavior_list_client.wait_for_service(timeout_sec=5.0):
                request = Trigger.Request()
                future = behavior_list_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() and future.result().success:
                    behavior_descriptions = json.loads(future.result().message)
                    self.get_logger().info(f"Retrieved {len(behavior_descriptions)} behavior descriptions")
                else:
                    self.get_logger().error("Failed to get behavior descriptions")
            else:
                self.get_logger().error("Behavior list service not available!")
            
            # Get condition descriptions
            condition_descriptions = None
            if condition_list_client.wait_for_service(timeout_sec=5.0):
                request = Trigger.Request()
                future = condition_list_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() and future.result().success:
                    condition_descriptions = json.loads(future.result().message)
                    self.get_logger().info(f"Retrieved {len(condition_descriptions)} condition descriptions")
                else:
                    self.get_logger().error("Failed to get condition descriptions")
            else:
                self.get_logger().error("Condition list service not available!")
            
            # Check if we got both descriptions
            if behavior_descriptions is None or condition_descriptions is None:
                self.get_logger().warn("Failed to get descriptions, falling back to simple FSM")
                return create_simple_fsm()
            
            # Call the FSM factory with descriptions (method from fsm file)
            from .fsm import create_fsm_from_config
            return create_fsm_from_config(_fsm_config, behavior_descriptions, condition_descriptions) # method from fsm file

        except Exception as e:
            self.get_logger().error(f"Error creating FSM from config: {e}")
            return create_simple_fsm()
            


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller_node = ControllerNode()
        controller_node.start_state()  # Start the FSM
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()


    