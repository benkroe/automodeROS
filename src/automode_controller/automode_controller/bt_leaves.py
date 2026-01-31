import py_trees
import rclpy
from rclpy.action import ActionClient
from automode_interfaces.action import Behavior, Condition


class BehaviorLeaf(py_trees.behaviour.Behaviour):

    def __init__(self, name: str, controller: rclpy.node.Node, behavior_name: str, behavior_params=None):
        super().__init__(name)
        self.controller = controller
        self.behavior_name = behavior_name
        self.behavior_params = behavior_params or []

    def setup(self):
        """Called once to set up the action client."""
        pass

    def initialise(self):
        """Called at the start of execution. Request to set current behavior."""
        self.controller.set_current_behavior(self.behavior_name, self.behavior_params)

    def update(self):
        """Tick function: always running for continuous behavior."""
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # Not needed as the behavior_node only allows on behavior active at a time and the new one will override the old one
        pass


class ConditionLeaf(py_trees.behaviour.Behaviour):

    def __init__(self, name: str, node: rclpy.node.Node, condition_name: str, condition_params=None):
        super().__init__(name)
        self.node = node
        self.condition_name = condition_name
        self.condition_params = condition_params or []

        self._client = None
        self._goal_handle = None
        self._condition_met = False
        self._result = None
        self._sent = False
        self._previous_met = False

    def setup(self):

        self._client = ActionClient(self.node, Condition, 'condition_action')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().info(f'Condition action server not available yet for {self.condition_name} (waiting)')
            return
        if not self._sent:
            goal_msg = Condition.Goal()
            goal_msg.condition_name = self.condition_name
            goal_msg.params = self.condition_params
            self.node.get_logger().info(f'Sending condition goal: {self.condition_name} params={self.condition_params}')
            send_future = self._client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
            send_future.add_done_callback(self._on_goal_response)
            self._sent = True

    def initialise(self):
        pass

    def update(self):
        if self._condition_met != self._previous_met:
            self.node.get_logger().info(f'Condition {self.condition_name} status changed: met={self._condition_met}')
            self._previous_met = self._condition_met
        if self._condition_met:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        try:
            if self._goal_handle is not None:
                self._client.cancel_goal(self._goal_handle)
        except Exception:
            pass

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.node.get_logger().warning(f'Condition goal {self.condition_name} rejected')
                self._result = type('R', (), {'success': False})()
                return
            self._goal_handle = goal_handle
            self.node.get_logger().info(f'Condition goal accepted: {self.condition_name}')
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self._on_result)
        except Exception as e:
            self.node.get_logger().error(f'Error sending condition goal: {e}')
            self._result = type('R', (), {'success': False})()

    def _on_feedback(self, feedback_msg):
        try:
            fb = feedback_msg.feedback
            met = getattr(fb, 'condition_met', False)
            if self._condition_met != met:
                self.node.get_logger().info(f'Condition {self.condition_name} status changed: met={met}')
                self._condition_met = met
        except Exception:
            pass

    def _on_result(self, future):
        try:
            res = future.result().result
            self._result = res
        except Exception as e:
            self.node.get_logger().error(f'Error getting condition result: {e}')
            self._result = type('R', (), {'success': False})()
