#!/usr/bin/env python3

import importlib
import json
import time
import pkgutil
import traceback
from typing import Dict, Any, List, Optional


import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.executors import ExternalShutdownException

from automode_interfaces.action import Behavior




class BehaviorNode(Node):
    """
    Behavior action server + discovery node.

    - discovers modules under automode_controller.modules.behaviors
    - publishes discovery on 'behaviors/list' (std_msgs/String) as JSON
    - provides a Trigger service 'behaviors/list_srv' returning the same JSON in message
    - provides an action server 'behavior_action' if automode action is available
    """

    def __init__(self, name: str = 'behavior_node'):
        super().__init__(name)
        
        # parameter for modules pkg
        self.declare_parameter('module_package', 'basic_modules')

        self.get_logger().info('Starting BehaviorNode')
        # mapping: behavior_name -> {'instance': obj, 'descriptor': dict, 'module_name': str}
        self._behaviors: Dict[str, Dict[str, Any]] = {}
        self._list_pub = self.create_publisher(String, 'behaviors/list', 10)
        self._list_srv = self.create_service(Trigger, 'behaviors/list_srv', self._handle_list_srv)
        # timer to republish discovery periodically
        self.create_timer(1.0, self._publish_list)

        # discover behavior modules
        self._discover_behaviors()

        # create action server if generated action exists
        if Behavior is not None:
            self._action_server = ActionServer(
                node=self, 
                action_type=Behavior, 
                action_name='behavior_action', 
                execute_callback=self._execute_action, 
                cancel_callback=self.cancel_callback)

        self.get_logger().info('Discovered behaviors: %s' % (', '.join(sorted(self._behaviors.keys()))))


    # cancel calback example i thought i woudl not need that but lets see
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT


    def _discover_behaviors(self) -> None:
        base_package = self.get_parameter('module_package').value
        package_name = f"{base_package}.behaviors"
        try:
            bh_pkg = importlib.import_module(package_name)
        except Exception:
            self.get_logger().warning(f'No behaviors package found: {package_name}')
            return

        for finder, mod_name, ispkg in pkgutil.iter_modules(bh_pkg.__path__):
            try:
                mod = importlib.import_module(f'{package_name}.{mod_name}')
            except Exception:
                self.get_logger().error(f'Failed to import behavior module "{mod_name}":\n{traceback.format_exc()}')
                continue

            # Only support class-based API: a class named 'Behavior'
            if not hasattr(mod, 'Behavior'):
                self.get_logger().warning(f'Behavior module "{mod_name}" missing required Behavior class')
                continue

            try:
                cls = getattr(mod, 'Behavior')
                behavior_instance = cls()
            except Exception:
                self.get_logger().error(f'Failed to instantiate Behavior class in "{mod_name}":\n{traceback.format_exc()}')
                continue

            # Get descriptor
            try:
                descriptor = behavior_instance.get_description() or {}
            except Exception:
                self.get_logger().error(f'get_description() failed for "{mod_name}":\n{traceback.format_exc()}')
                descriptor = {}

            # Determine canonical behavior name
            behavior_name = descriptor.get('name') if isinstance(descriptor, dict) and descriptor.get('name') else mod_name

            # Setup listeners
            if hasattr(behavior_instance, 'setup_listeners') and callable(behavior_instance.setup_listeners):
                try:
                    behavior_instance.setup_listeners(self)
                except Exception:
                    self.get_logger().error(f'setup_listeners failed for "{behavior_name}" ({mod_name}):\n{traceback.format_exc()}')

            self._behaviors[behavior_name] = {
                'instance': behavior_instance,
                'descriptor': descriptor,
                'module_name': mod_name,
        }

    def _publish_list(self) -> None:
        # Publish available behaviors as JSON on 'behaviors/list'.
        overview = {k: v.get('descriptor', {}) for k, v in self._behaviors.items()}
        msg = String()
        msg.data = json.dumps(overview)
        self._list_pub.publish(msg)

    def _handle_list_srv(self, request, response):
        # Trigger service handler returning JSON overview in response.message
        overview = {k: v.get('descriptor', {}) for k, v in self._behaviors.items()}
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
        req_name = getattr(goal, 'behavior_name', None)
        params_list = list(getattr(goal, 'params', [])) if hasattr(goal, 'params') else []
        result = Behavior.Result() if Behavior is not None else None

        self.get_logger().info(f'Received goal for behavior "{req_name}" with params: {params_list}')
        # Validate and setup behavior
        behavior_instance = self._setup_behavior(req_name, params_list, goal_handle, result)
        if behavior_instance is None:
            return result 

        # Run continuous execution loop
        return self._run_behavior_loop(behavior_instance, req_name, goal_handle, result)

    def _setup_behavior(self, req_name, params_list, goal_handle, result):
        """Setup and validate behavior for execution."""
        # Lookup behavior
        if not req_name or req_name not in self._behaviors:
            return self._abort_with_message(f'Unknown behavior: {req_name}', goal_handle, result)

        entry = self._behaviors[req_name]
        inst = entry['instance']
        descriptor = entry.get('descriptor', {})

        # Parse params and setup
        params_dict = self._parse_params(params_list, descriptor)
        try:
            inst.set_params(params_dict)
            inst.reset()
            inst.setup_communication(self)
        except Exception as e:
            self.get_logger().error(f'Failed to setup behavior "{req_name}": {e}')
            return self._abort_with_message(f'Setup failed: {str(e)}', goal_handle, result)
        return inst

    def _run_behavior_loop(self, inst, req_name, goal_handle, result):
        # Run the continuous behavior loop
        step_count = 0
        execution_rate = 30.0  # Hz
        sleep_duration = 1.0 / execution_rate

        try:
            while rclpy.ok():
                # Add debug output
                if step_count % 10 == 0:  # Every 1 second
                    # self.get_logger().info(f'Behavior "{req_name}" step {step_count}, cancel_requested: {goal_handle.is_cancel_requested}')
                    pass # Logger for debugging
                if goal_handle.is_cancel_requested:
                    return self._handle_cancellation(req_name, step_count, "User requested cancellation", goal_handle, result)

                # Execute one step
                step_result = self._execute_behavior_step(inst, req_name, step_count)
                if step_result is None:
                    return self._abort_with_message('module has no execute_step method', goal_handle, result)

                success, message, completed = step_result
                step_count += 1

                # Publish feedback
                self._publish_feedback(step_count, message, goal_handle)

                # Check completion or failure
                if completed:
                    return self._handle_completion(req_name, step_count, message, goal_handle, result)
                if not success:
                    return self._handle_failure(req_name, step_count, message, goal_handle, result)

                time.sleep(sleep_duration)

        except Exception as e:
            self.get_logger().error(f'Behavior "{req_name}" execution failed:\n{traceback.format_exc()}')
            return self._abort_with_message(f'Exception at step {step_count}: {str(e)}', goal_handle, result)

        return self._abort_with_message('Behavior execution loop ended unexpectedly', goal_handle, result)

    def _execute_behavior_step(self, inst, req_name, step_count):
        #exec one behavior step

        try:
            ret = inst.execute_step()
            if isinstance(ret, tuple) and len(ret) >= 2:
                success, message = ret[0], ret[1]
                completed = ret[2] if len(ret) > 2 else success  
            else:
                success, message = bool(ret), str(ret)
                completed = success
            return success, message, completed
        except Exception as e:
            self.get_logger().error(f'Behavior "{req_name}" step {step_count} failed:\n{traceback.format_exc()}')
            return False, f'Exception: {str(e)}', True  


    def _publish_feedback(self, step_count, message, goal_handle):
        # In this actions feedback is not realy needed
        pass

    def _abort_with_message(self, message, goal_handle, result):
        # used above to abort in exeptions. Makes the abortion above shorter (oneline)
        self.get_logger().warning(message)
        if result is not None:
            goal_handle.abort()
            result.success = False
            result.message = message
        return result
    
    def _handle_completion(self, req_name, step_count, message, goal_handle, result):
        """Handle successful behavior completion."""
        self.get_logger().info(f'Behavior "{req_name}" completed after {step_count} steps')
        goal_handle.succeed()
        result.success = True
        result.message = f'Completed: {message}'
        return result

    def _handle_cancellation(self, req_name, step_count, message, goal_handle, result):
        """Handle behavior cancellation."""
        self.get_logger().info(f'Behavior "{req_name}" cancelled after {step_count} steps')
        goal_handle.canceled()
        result.success = False
        result.message = f'Cancelled after {step_count} steps'
        return result

    def _handle_failure(self, req_name, step_count, message, goal_handle, result):
        """Handle behavior failure."""
        self.get_logger().warning(f'Behavior "{req_name}" failed at step {step_count}: {message}')
        goal_handle.abort()
        result.success = False
        result.message = f'Failed at step {step_count}: {message}'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Shutting down due to interrupt or external shutdown')
    finally:
        # clean up action server if created
        if getattr(node, '_action_server', None) is not None:
            node._action_server.destroy()
        node.destroy_node()

if __name__ == '__main__':
    main()