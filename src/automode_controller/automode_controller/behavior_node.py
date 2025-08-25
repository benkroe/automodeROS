#!/usr/bin/env python3

import importlib
import json
import pkgutil
import traceback
from typing import Dict, Any, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from std_srvs.srv import Trigger

# try to import generated action; if not available, we still allow discovery/testing

from automode_interfaces.action import Behavior  # type: ignore



class BehaviorNode(Node):
    """
    Behavior action server + discovery node.

    - discovers modules under automode_controller.modules.behaviors
      supports modules that expose:
        * class-based API: class Behavior(...) with methods get_description, setup_listeners, set_params, execute_step
        * legacy module-level API: functions get_description(), execute_step(), optional setup_listeners()/set_params()
    - publishes discovery on 'behaviors/list' (std_msgs/String) as JSON
    - provides a Trigger service 'behaviors/list_srv' returning the same JSON in message
    - provides an action server 'behavior_action' if automode action is available
    """

    def __init__(self, name: str = 'behavior_node'):
        super().__init__(name)
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
            self._action_server = ActionServer(self, Behavior, 'behavior_action', self._execute_action)
            self.get_logger().info('Behavior action server created at "behavior_action"')
        else:
            self._action_server = None
            self.get_logger().warning(
                'Behavior action definition not available (automode_controller.action.Behavior). Action server not created.'
            )

        self.get_logger().info('Discovered behaviors: %s' % (', '.join(sorted(self._behaviors.keys()))))

    def _discover_behaviors(self) -> None:

        try:
            import automode_controller.modules.behaviors as bh_pkg  # package containing behavior modules
        except Exception:
            self.get_logger().warning('No automode_controller.modules.behaviors package found; nothing to discover')
            return

        for finder, mod_name, ispkg in pkgutil.iter_modules(bh_pkg.__path__):
            try:
                mod = importlib.import_module(f'automode_controller.modules.behaviors.{mod_name}')
            except Exception:
                self.get_logger().error(f'Failed to import behavior module "{mod_name}":\n{traceback.format_exc()}')
                continue

            behavior_instance = None
            descriptor: Dict[str, Any] = {}

            # Prefer class-based API: a class named 'Behavior'
            if hasattr(mod, 'Behavior'):
                try:
                    cls = getattr(mod, 'Behavior')
                    behavior_instance = cls()  # instantiate
                except Exception:
                    self.get_logger().error(f'Failed to instantiate Behavior class in "{mod_name}":\n{traceback.format_exc()}')
                    continue
            else:
                # Fallback to legacy module-level API: functions get_description/execute_step
                if hasattr(mod, 'get_description') and hasattr(mod, 'execute_step'):
                    # create a thin wrapper object to present a consistent interface
                    class _Wrapper:
                        pass

                    wrapper = _Wrapper()
                    wrapper.get_description = mod.get_description
                    wrapper.execute_step = mod.execute_step
                    wrapper.setup_listeners = getattr(mod, 'setup_listeners', None)
                    wrapper.set_params = getattr(mod, 'set_params', None)
                    behavior_instance = wrapper
                else:
                    self.get_logger().warning(f'Behavior module "{mod_name}" missing required API (Behavior class or get_description/execute_step)')
                    continue

            # obtain descriptor
            try:
                # get_description may be a staticmethod on the class or a function on the module/wrapper
                descriptor = behavior_instance.get_description() or {}
            except Exception:
                self.get_logger().error(f'get_description() failed for "{mod_name}":\n{traceback.format_exc()}')
                descriptor = {}

            # Determine canonical behavior name (use descriptor.name if provided)
            behavior_name = descriptor.get('name') if isinstance(descriptor, dict) and descriptor.get('name') else mod_name

            # attempt to attach module listeners to our node (if provided)
            setup_fn = getattr(behavior_instance, 'setup_listeners', None)
            if callable(setup_fn):
                try:
                    setup_fn(self)
                except Exception:
                    self.get_logger().error(f'setup_listeners failed for "{behavior_name}" ({mod_name}):\n{traceback.format_exc()}')

            self._behaviors[behavior_name] = {
                'instance': behavior_instance,
                'descriptor': descriptor,
                'module_name': mod_name,
            }

    def _publish_list(self) -> None:
        """
        Publish available behaviors as JSON on 'behaviors/list'.
        """
        overview = {k: v.get('descriptor', {}) for k, v in self._behaviors.items()}
        msg = String()
        try:
            msg.data = json.dumps(overview)
        except Exception:
            msg.data = '{}'
        self._list_pub.publish(msg)

    def _handle_list_srv(self, request, response):
        """
        Trigger service handler returning JSON overview in response.message
        """
        overview = {k: v.get('descriptor', {}) for k, v in self._behaviors.items()}
        try:
            response.success = True
            response.message = json.dumps(overview)
        except Exception:
            response.success = False
            response.message = 'failed to serialize behavior overview'
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
        """
        Action server execute callback.

        Expects Behavior action:
          goal.behavior_name : string
          goal.params : string[]
        """
        # accept goal
        goal_handle.accept_goal()
        goal = goal_handle.request
        req_name = getattr(goal, 'behavior_name', None)
        params_list = list(getattr(goal, 'params', [])) if hasattr(goal, 'params') else []

        result = Behavior.Result() if Behavior is not None else None

        # lookup behavior by name
        if not req_name or req_name not in self._behaviors:
            msg = f'Unknown behavior: {req_name}'
            self.get_logger().warning(msg)
            if result is not None:
                goal_handle.abort()
                result.success = False
                result.message = msg
                return result
            return None

        entry = self._behaviors[req_name]
        inst = entry['instance']
        descriptor = entry.get('descriptor', {})

        # parse params into dict and call set_params if provided
        params_dict = self._parse_params(params_list, descriptor)
        if hasattr(inst, 'set_params') and callable(getattr(inst, 'set_params')):
            try:
                inst.set_params(params_dict)
            except Exception:
                self.get_logger().error(f'set_params failed for "{req_name}":\n{traceback.format_exc()}')

        # execute: if instance has execute_step(), call it and return its tuple
        try:
            if hasattr(inst, 'execute_step') and callable(getattr(inst, 'execute_step')):
                ret = inst.execute_step()
                if isinstance(ret, tuple) and len(ret) == 2:
                    success, message = ret
                else:
                    success, message = bool(ret), str(ret)
            # fallback to run(params) style if provided
            elif hasattr(inst, 'run') and callable(getattr(inst, 'run')):
                try:
                    ret = inst.run(params_list)
                except TypeError:
                    ret = inst.run(params_dict)
                if isinstance(ret, tuple) and len(ret) == 2:
                    success, message = ret
                else:
                    success, message = bool(ret), str(ret)
            else:
                success, message = False, 'module has no execute_step/run method'
        except Exception:
            self.get_logger().error(f'Behavior "{req_name}" execution failed:\n{traceback.format_exc()}')
            success, message = False, f'exception during execution: {traceback.format_exc()}'

        # finalize action result
        if result is not None:
            if success:
                goal_handle.succeed()
                result.success = True
                result.message = str(message)
            else:
                goal_handle.abort()
                result.success = False
                result.message = str(message)
            return result
        return None


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    finally:
        # clean up action server if created
        if getattr(node, '_action_server', None) is not None:
            node._action_server.destroy()
        node.destroy_node()

if __name__ == '__main__':
    main()