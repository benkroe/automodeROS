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
        self._fsm: FSM = self.create_fsm(_fsm_config)


        self._action_client_behavior = ActionClient(self, Behavior, 'behavior_action')
        self._action_client_condition = ActionClient(self, Condition, 'condition_action')

        self._active_behavior_goal = None
        self._active_condition_goals: Dict[str, Any] = {}


    def start_state(self):
        # starts the action of the current state and all necessary conditions
        try:   
            self.start_behavior_action(self._fsm.get_current_state())

            for edge in self._fsm.get_outgoing_edges(self._fsm.get_current_state().behavior_name):
                self.start_condition_action(edge)
        except Exception as e:
            self.get_logger().error(f"Error starting state: {e}")

    def stop_state(self):
        # stops the action of the current state and all necessary conditions
        try:
            self.stop_behavior_action(self._fsm.get_current_state())

            for edge in self._fsm.get_outgoing_edges(self._fsm.get_current_state().condition_name):
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

        send_goal_future = self._action_client_condition.send_goal_async(goal_msg, feedback_callback=self.feedback_callback_condition)
        send_goal_future.add_done_callback(lambda future, edge_ref=edge: self.goal_response_condition(future, edge_ref))
        self.get_logger().info(f"Started condition: {edge.condition_name} with params: {edge.condition_params}")


    def create_fsm(self, _fsm_config):
        # TODO: create the fsm from the parameters
        if _fsm_config == 'simple_fsm':
            return create_simple_fsm()
        else:
            # TODO: create FSM from config
            pass
    

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
    
    def feedback_callback_condition(self, feedback):
        # TODO: implement logic. If for one of the conditions is true we need to change state
        pass


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


    