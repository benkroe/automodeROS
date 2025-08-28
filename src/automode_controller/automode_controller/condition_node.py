#!/usr/bin/env python3
import importlib
import json
import time
import pkgutil
import traceback
import threading
from typing import Dict, Any, List, Optional, Set
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse
from std_msgs.msg import String
from std_srvs.srv import Trigger

from automode_interfaces.action import Condition

class ConditionNode(Node):
    """
    Condition action server supporting parallel execution.
    
    - Discovers condition modules
    - Runs multiple conditions simultaneously 
    - Each condition monitored in separate thread
    """

    def __init__(self, name: str = 'condition_node'):
        super().__init__(name)
        self.get_logger().info('Starting ConditionNode')
        
        self._conditions: Dict[str, Dict[str, Any]] = {}
        
        # Active condition tracking
        self._active_conditions: Dict[str, Dict[str, Any]] = {}
        self._condition_lock = threading.Lock()
        
        # Thread pool for parallel condition execution
        self._executor_pool = ThreadPoolExecutor(max_workers=10)
        
        # Discovery and services
        self._list_pub = self.create_publisher(String, 'conditions/list', 10)
        self._list_srv = self.create_service(Trigger, 'conditions/list_srv', self._handle_list_srv)
        self.create_timer(1.0, self._publish_list)
        
        # Discover conditions
        self._discover_conditions()
        
        # Action server
        self._action_server = ActionServer(
            node=self,
            action_type=Condition,
            action_name='condition_action',
            execute_callback=self._execute_action,
            goal_callback=self._accept_multiple_goals,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info(f'Discovered conditions: {", ".join(sorted(self._conditions.keys()))}')

    def _accept_multiple_goals(self, goal_request):
        return rclpy.action.GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        goal_id = str(goal_handle.goal_id)
        
        with self._condition_lock:
            if goal_id in self._active_conditions:
                self._active_conditions[goal_id]['cancelled'] = True
                self.get_logger().info(f'Marked condition {goal_id} for cancellation')
        
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT


    def _discover_conditions(self) -> None:
        try:
            import automode_controller.modules.conditions as cond_pkg
        except Exception:
            self.get_logger().warning('No conditions package found')
            return

        for finder, mod_name, ispkg in pkgutil.iter_modules(cond_pkg.__path__):
            try:
                mod = importlib.import_module(f'automode_controller.modules.conditions.{mod_name}')
            except Exception:
                self.get_logger().error(f'Failed to import condition module {mod_name}:\n{traceback.format_exc()}')
                continue
            
            # Only classes support Condition
            if not hasattr(mod, 'Condition'):
                self.get_logger().warning(f'Condition module {mod_name} missing Condition class')
                continue

            try:
                cls = getattr(mod, 'Condition')
                condition_instance = cls()
            except Exception:
                self.get_logger().error(f'Failed to instantiate Condition in {mod_name}:\n{traceback.format_exc()}')
                continue

            # descriptor  
            try:
                descriptor = condition_instance.get_description() or {}
            except Exception:
                self.get_logger().error(f'Failed to get description from {mod_name}:\n{traceback.format_exc()}')
                descriptor = {}
            
            # Name
            condition_name = descriptor.get('name') if isinstance(descriptor, dict) and descriptor.get('name') else mod_name

            # Listeners
            if hasattr(condition_instance, 'setup_listeners') and callable(condition_instance.setup_listeners):
                try:
                    condition_instance.setup_listeners(self)
                except Exception:
                    self.get_logger().error(f'Failed to setup listeners for "{condition_name}"({mod_name}):\n{traceback.format_exc()}')
            
            self._conditions[condition_name] = {
                'instance': condition_instance,
                'descriptor': descriptor,
                'module_name': mod_name
            }
    def _publish_list(self) -> None:
        # Publish available conditions (maybe later only service)
        overview = {k: v.get('descriptor', {}) for k, v in self._conditions.items()}
        msg = String()
        msg.data = json.dumps(overview)
        self._list_pub.publish(msg)


    def _handle_list_srv(self, request, response):
        # Service for returning JSON overview
        overview = {k: v.get('descriptor', {}) for k, v in self._conditions.items()}
        response.success = True
        response.message = json.dumps(overview)
        return response
    
    def _parse_params(self, params: List[str], descriptor: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Convert a list of strings into a params dict.

        - accepts "key=value" pairs
        - otherwise maps positional params to descriptor['params'] if available (supports list of names)
        - attempts to convert numeric/bool strings
        """
        parsed: Dict[str, Any] = {}
        if not params:
            return parsed

        # try key=value pairs first
        kv_pairs = [p for p in params if '=' in p]
        if kv_pairs:
            for p in kv_pairs:
                try:
                    k, v = p.split('=', 1)
                    parsed[k] = self._coerce_value(v)
                except Exception:
                    parsed[p] = p
            return parsed

        # otherwise positional mapping using descriptor
        if descriptor and isinstance(descriptor.get('params'), list):
            names = []
            for entry in descriptor['params']:
                if isinstance(entry, dict) and 'name' in entry:
                    names.append(entry['name'])
                elif isinstance(entry, str):
                    names.append(entry)
            for i, val in enumerate(params):
                if i < len(names):
                    parsed[names[i]] = self._coerce_value(val)
                else:
                    # extra positional params kept as p0, p1...
                    parsed[f'p{i}'] = self._coerce_value(val)
            return parsed

        # fallback: store as p0..pn
        for i, val in enumerate(params):
            parsed[f'p{i}'] = self._coerce_value(val)
        return parsed
    
    @staticmethod
    def _coerce_value(v: str):
        # try int, float, bool, else string
        if v.lower() in ('true', 'false'):
            return v.lower() == 'true'
        try:
            iv = int(v)
            return iv
        except Exception:
            pass
        try:
            fv = float(v)
            return fv
        except Exception:
            pass
        return v

    async def _execute_action(self, goal_handle):
        goal = goal_handle.request 
        req_name = getattr(goal, 'condition_name', None)
        params_list = list(getattr(goal, 'params', [])) if hasattr(goal, 'params') else []
        result = Condition.Result() if Condition is not None else None

        goal_id = str(goal_handle.goal_id)
        with self._condition_lock:
            self._active_conditions[goal_id] = {
                'name': req_name,
                'goal_handle': goal_handle,
                'cancelled': False
            }
        try:
            self.get_logger().info(f'Condition action request for "{req_name}" with params: {params_list}')

            condition_instance = self._setup_condition(req_name, params_list, goal_handle, result)
            if condition_instance is None:
                return result
            
            return self._run_condition_loop(condition_instance, req_name, goal_handle, result)
        finally:
            with self._condition_lock:
                if goal_id in self._active_conditions:
                    del self._active_conditions[goal_id]

    def _setup_condition(self, req_name, params_list, goal_handle, result):
        # Setup condition
        if not req_name or req_name not in self._conditions:
            return self._abort_with_message(f'Unknown condition: {req_name}', goal_handle, result)
        
        entry = self._conditions[req_name]
        inst = entry['instance']
        descriptor = entry.get('descriptor', {})

        params_dict = self._parse_params(params_list, descriptor)

        try:
            inst.set_params(params_dict)
            inst.reset()
            inst.setup_communication(self)
        except Exception as e:
            self.get_logger().error(f'Failed to setup condition "{req_name}": {e}')
            return self._abort_with_message(f'Setup failed: {str(e)}', goal_handle, result)
                

        goal_id = str(goal_handle.goal_id)
        with self._condition_lock:
            if goal_id in self._active_conditions:
                self._active_conditions[goal_id]['instance'] = inst
        return inst
    
    def _run_condition_loop(self, inst, req_name, goal_handle, result):
        # Run the continuous condition loop (is for one instance of action)
        step_count = 0
        execution_rate = 10.0 # Hz
        sleep_duration = 1.0 / execution_rate

        try:
            while rclpy.ok():
                # Debug output
                if step_count % 10 == 0:
                    self.get_logger().info(f'Condition "{req_name}" running step {step_count}')
                
                # CHECK: Per-goal cancellation instead of global
                goal_id = str(goal_handle.goal_id)
                with self._condition_lock:
                    if goal_id in self._active_conditions and self._active_conditions[goal_id].get('cancelled', False):
                        return self._handle_cancellation(req_name, step_count, "Cancelled by user", goal_handle, result)
                
                # Execute one step here
                step_result = self._execute_condition_step(inst, req_name, step_count)

                if step_result is None:
                    return self._abort_with_message(f'Condition "{req_name}" step execution failed', goal_handle, result)
                
                condition_met, message = step_result
                step_count += 1

                self._publish_feedback(step_count, message, goal_handle, condition_met)

                time.sleep(sleep_duration)
            
        except Exception as e:
            self.get_logger().error(f'Unexpected error in condition "{req_name}": {e}')
            return self._abort_with_message(f'Unexpected error: {str(e)}', goal_handle, result)

    def _execute_condition_step(self, inst, req_name, step_count):
        # exec one evaluation step
        try: 
            ret = inst.execute_step()
            if isinstance(ret, tuple) and len(ret) >= 2:
                condition_met, message = ret[0], ret[1]
            else:
                condition_met, message = bool(ret), str(ret)
            return condition_met, message
        except Exception as e:
            self.get_logger().error(f'Condition "{req_name}" step execution error:\n{traceback.format_exc()}')
            return False, f'Exception: {str(e)}', True


    def _publish_feedback(self, step_count, message, goal_handle, condition_met):
        # feedback important here
        try:
            feedback_msg = Condition.Feedback()
            
            # Get condition info for better feedback
            goal_id = str(goal_handle.goal_id)
            condition_name = "unknown"
            
            with self._condition_lock:
                if goal_id in self._active_conditions:
                    condition_name = self._active_conditions[goal_id].get('name', 'unknown')
            
            # Format status message with condition name and step info
            feedback_msg.condition_met = condition_met
            feedback_msg.current_status = f"[{condition_name}] Step {step_count}: {message}"
            
            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)
            
            # Log feedback periodically to avoid spam (every 20 steps)
            if step_count % 20 == 0:
                self.get_logger().debug(f'Condition "{condition_name}" step {step_count}: {message}')
                
        except Exception as e:
            self.get_logger().warning(f'Failed to publish feedback for step {step_count}: {e}')

    def _abort_with_message(self, message, goal_handle, result):
        # used above to abort in exeptions. Makes the abortion above shorter (oneline)
        self.get_logger().warning(message)
        if result is not None:
            goal_handle.abort()
            result.success = False
            result.message = message
        return result


    def _handle_failure(self, req_name, step_count, message, goal_handle, result):
        """Handle condition failure."""
        self.get_logger().warning(f'Condition "{req_name}" failed at step {step_count}: {message}')
        goal_handle.abort()
        result.success = False
        result.message = f'Failed at step {step_count}: {message}'
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ConditionNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    finally:
        node._executor_pool.shutdown(wait=True)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()