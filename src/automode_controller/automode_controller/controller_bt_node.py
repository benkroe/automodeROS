#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import py_trees
from py_trees.trees import BehaviourTree
import json
import os
from pathlib import Path

from .bt_leaves import BehaviorLeaf, ConditionLeaf
from .bt_parser import parse_bt_config, load_categories_from_descriptions
from automode_interfaces.action import Behavior



class ControllerBTNode(Node):
    def __init__(self):
        super().__init__('controller_bt_node')

        self._behavior_client = ActionClient(self, Behavior, 'behavior_action')
        self._current_behavior_name = None
        self._current_goal_handle = None
        self._current_behavior_result = None

        # Accept either a direct bt_config string or a path to a config file
        self.declare_parameter('bt_config', '')
        self.declare_parameter('bt_config_file', '')

        bt_config = self.get_parameter('bt_config').get_parameter_value().string_value
        bt_config_file = self.get_parameter('bt_config_file').get_parameter_value().string_value

        if bt_config_file:
            try:
                if os.path.exists(bt_config_file):
                    with open(bt_config_file, 'r') as fh:
                        bt_config = fh.read().strip()
            except Exception as e:
                self.get_logger().warning(f'Failed to read bt_config_file: {e}')

        if not bt_config:
            self.get_logger().info('No bt_config provided, using default simple behavior')
            bt_config = '--bt-config --nroot 1 --nchildroot 1 --n0 5 --a0 0'

        # parse
        spec = parse_bt_config(bt_config)
        # Log parsed spec for debugging
        try:
            from .bt_parser import pretty_print_spec, validate_spec_against_categories
            self.get_logger().info(f'Parsed BT config spec: root_type={spec.get("root",{}).get("type")}, nodes={len(spec.get("nodes",{}))}')
            try:
                spec_str = pretty_print_spec(spec)
                self.get_logger().info(f'BT Tree Structure:\n{spec_str}')
            except Exception:
                # pretty_print_spec may fail; ignore
                pass
        except Exception:
            validate_spec_against_categories = None

        # Obtain behavior/condition descriptions via services and build
        # numeric type -> name maps (FSM-style). We use descriptions-only.
        node_map = {}
        edge_map = {}
        try:
            from std_srvs.srv import Trigger
            # Create service clients
            behavior_list_client = self.create_client(Trigger, 'behaviors/list_srv')
            condition_list_client = self.create_client(Trigger, 'conditions/list_srv')

            behavior_descriptions = None
            condition_descriptions = None

            if behavior_list_client.wait_for_service(timeout_sec=10.0):
                req = Trigger.Request()
                fut = behavior_list_client.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
                if fut.result() and fut.result().success:
                    behavior_descriptions = json.loads(fut.result().message)
                else:
                    self.get_logger().error('Failed to get behavior descriptions')
            else:
                self.get_logger().error('Behavior list service not available')

            if condition_list_client.wait_for_service(timeout_sec=10.0):
                req = Trigger.Request()
                fut = condition_list_client.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
                if fut.result() and fut.result().success:
                    condition_descriptions = json.loads(fut.result().message)
                else:
                    self.get_logger().error('Failed to get condition descriptions')
            else:
                self.get_logger().error('Condition list service not available')

            if behavior_descriptions is None or condition_descriptions is None:
                self.get_logger().warning('Could not retrieve behavior/condition descriptions; using empty maps')
                behavior_descriptions = {}
                condition_descriptions = {}

            node_map, edge_map = load_categories_from_descriptions(behavior_descriptions, condition_descriptions)
            # log retrieved descriptions and maps
            try:
                self.get_logger().info(f'Retrieved behavior descriptions: {list(behavior_descriptions.keys())}')
                self.get_logger().info(f'Retrieved condition descriptions: {list(condition_descriptions.keys())}')
                self.get_logger().info(f'Built behavior type map: {node_map}')
                self.get_logger().info(f'Built condition type map: {edge_map}')
            except Exception:
                pass

        except Exception as e:
            self.get_logger().error(f'Failed to build category maps from descriptions: {e}')
            raise

        # validate & build tree
        try:
            # validate spec against category maps (similar to FSM checks)
            from .bt_parser import validate_spec_against_categories
            validate_spec_against_categories(spec, node_map, edge_map)
        except Exception as e:
            self.get_logger().error(f'BT spec validation failed: {e}')
            raise

        # build tree
        try:
            root_node = self._build_tree_from_spec(spec, node_map, edge_map)
            self._tree = BehaviourTree(root_node)
            try:
                # Ensure py_trees initialisation is performed so leaves' initialise() runs
                self._tree.setup(timeout=5.0)
                self.get_logger().info('BT tree setup completed successfully')
            except Exception as e:
                self.get_logger().error(f'BT tree setup failed: {e}')
                raise
            self.get_logger().info('controller_bt_node built tree from bt_config')
        except Exception as e:
            import traceback
            self.get_logger().error(f'Failed to build tree from spec: {e}\n{traceback.format_exc()}')
            # fallback to simple behavior
            root = py_trees.composites.Sequence('Root', memory=False)
            beh = BehaviorLeaf('BehaviorLeaf', self, 'exploration')
            root.add_child(beh)
            self._tree = BehaviourTree(root)

        # Timer to tick the tree at 5 Hz
        # track last running nodes so we only log changes
        self._last_running = set()
        self.create_timer(0.2, self._tick)
        self.get_logger().info('controller_bt_node started, running py_trees loop')

    def _log_tree_status(self):
        """Collect running nodes after a tick and log changes."""
        try:
            running = []

            def collect(node):
                try:
                    st = getattr(node, 'status', None)
                    if st == py_trees.common.Status.RUNNING:
                        running.append(node.name)
                except Exception:
                    pass
                for c in getattr(node, 'children', []) or []:
                    collect(c)

            root = getattr(self._tree, 'root', None)
            if root is None:
                return
            collect(root)
            running_set = set(running)
            if running_set != self._last_running:
                self.get_logger().info(f'BT running nodes: {sorted(list(running_set))}')
                self._last_running = running_set
        except Exception as e:
            self.get_logger().debug(f'Failed to collect tree status: {e}')

    def _build_tree_from_spec(self, spec, node_map, edge_map):
        nodes = spec.get('nodes', {})

        def build_node(nid):
            node_spec = nodes.get(nid, {})
            ntype = node_spec.get('type')
            children = node_spec.get('children', [])
            params = node_spec.get('params', {})

            # Map numeric types to py_trees classes
            # 0 Selector, 1 Selector*, 2 Sequence, 3 Sequence*, 4 Decorator, 5 Action, 6 Condition, 7 Parallel.
            if ntype == 0:
                # Use Selector without memory for reactive behavior
                comp = py_trees.composites.Selector(f'Selector_{nid}', memory=False)
                for c in children:
                    comp.add_child(build_node(c))
                self.get_logger().info(f'Built Selector node {nid} with children {children}')
                return comp
            if ntype == 1:
                # memory=True for starred variants (type 1)
                comp = py_trees.composites.Selector(f'Selector_{nid}', memory=True)
                for c in children:
                    comp.add_child(build_node(c))
                self.get_logger().info(f'Built Selector* node {nid} with children {children}')
                return comp
            if ntype in (2, 3):
                memory = True if ntype == 3 else False
                comp = py_trees.composites.Sequence(f'Sequence_{nid}', memory=memory)
                if isinstance(children, list) and len(children) == 2:
                    child0_spec = nodes.get(children[0], {})
                    child1_spec = nodes.get(children[1], {})
                    if child0_spec.get('type') == 6 and child1_spec.get('type') == 5:
                        # Reactive sequence: Sequence(condition, behavior)
                        cond_node = build_node(children[0])
                        beh_node = build_node(children[1])
                        comp.add_child(cond_node)
                        comp.add_child(beh_node)
                        self.get_logger().info(f'Built reactive Sequence{"*" if ntype == 3 else ""} node {nid} with Sequence(condition, behavior)')
                    else:
                        for c in children:
                            comp.add_child(build_node(c))
                        self.get_logger().info(f'Built Sequence{"*" if ntype == 3 else ""} node {nid} with children {children}')
                else:
                    for c in children:
                        comp.add_child(build_node(c))
                    self.get_logger().info(f'Built Sequence{"*" if ntype == 3 else ""} node {nid} with children {children}')
                return comp
            if ntype == 4:
                # Decorator: wrap single child
                child = build_node(children[0]) if children else py_trees.behaviours.Success(f'Dummy_{nid}')
                # Use Inverter as a generic decorator for example
                comp = py_trees.decorators.Inverter(child)
                self.get_logger().info(f'Built Decorator node {nid} wrapping child {children[0] if children else "Dummy"}')
                return comp
            if ntype == 5:
                # Action -> behavior
                # expect param 'a' to contain behavior id
                beh_id = params.get('a')
                beh_name = None
                if isinstance(beh_id, int) and beh_id in node_map:
                    beh_name = node_map[beh_id]
                if beh_name is None:
                    beh_name = str(params.get('name', 'exploration'))
                # prepare params list for behavior action (exclude 'a' and 'c')
                param_list = [f"{k}={v}" for k, v in params.items() if k not in ('a', 'c')]
                leaf = BehaviorLeaf(f'Behavior_{nid}_{beh_name}', self, beh_name, param_list)
                self.get_logger().info(f'Built Action node {nid} for behavior {beh_name} with params {param_list}')
                return leaf
            if ntype == 6:
                # Condition node
                cond_id = params.get('c')
                cond_name = None
                if isinstance(cond_id, int) and cond_id in edge_map:
                    cond_name = edge_map[cond_id]
                if cond_name is None:
                    cond_name = str(params.get('name', 'unknown_condition'))
                param_list = [f"{k}={v}" for k, v in params.items() if k not in ('a', 'c')]
                leaf = ConditionLeaf(f'Condition_{nid}_{cond_name}', self, cond_name, param_list)
                self.get_logger().info(f'Built Condition node {nid} for condition {cond_name} with params {param_list}')
                return leaf

            # Unknown type: return a Success stub
            self.get_logger().warning(f'Unknown node type {ntype} for node {nid}, using Success stub')
            return py_trees.behaviours.Success(f'Unknown_{nid}')

        # Build children of root
        root_spec = spec.get('root', {})
        root_type = root_spec.get('type', 0)
        root_children = root_spec.get('children', [])
        if root_type == 0:
            root_memory = False
            root = py_trees.composites.Selector('Root', memory=root_memory)
        elif root_type == 1:
            root_memory = True
            root = py_trees.composites.Selector('Root', memory=root_memory)
        elif root_type == 2:
            root_memory = False
            root = py_trees.composites.Sequence('Root', memory=root_memory)
        else:
            root_memory = True
            root = py_trees.composites.Sequence('Root', memory=root_memory)
        for c in root_children:
            root.add_child(build_node(c))
        return root

    def set_current_behavior(self, name, params):
        if name != self._current_behavior_name:
            self._current_behavior_name = name
            # Send new goal
            goal_msg = Behavior.Goal()
            goal_msg.behavior_name = name
            goal_msg.params = params
            self.get_logger().info(f'Setting current behavior: {name} with params {params}')
            send_future = self._behavior_client.send_goal_async(goal_msg, feedback_callback=self._on_behavior_feedback)
            send_future.add_done_callback(self._on_behavior_goal_response)

    def _on_behavior_goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning(f'Behavior goal {self._current_behavior_name} rejected')
                self._current_behavior_name = None
                self._current_goal_handle = None
            else:
                self._current_goal_handle = goal_handle
                self.get_logger().info(f'Behavior goal accepted: {self._current_behavior_name}')
                # For continuous behaviors, we don't expect a result, but set up callback just in case
                get_result_future = goal_handle.get_result_async()
                get_result_future.add_done_callback(self._on_behavior_result)
        except Exception as e:
            self.get_logger().error(f'Error sending behavior goal: {e}')
            self._current_behavior_name = None
            self._current_goal_handle = None

    def _on_behavior_feedback(self, feedback_msg):
        try:
            fb = feedback_msg.feedback
            self.get_logger().info(f'Behavior feedback for {self._current_behavior_name}: {fb}')
        except Exception as e:
            self.get_logger().debug(f'Error logging behavior feedback: {e}')

    def _on_behavior_result(self, future):
        try:
            res = future.result().result
            self._current_behavior_result = res
            self.get_logger().info(f'Received result for current behavior {self._current_behavior_name}: success={getattr(res, "success", False)}')
            # Reset if completed, but for continuous, this may not happen
            self._current_behavior_name = None
            self._current_goal_handle = None
        except Exception as e:
            self.get_logger().error(f'Error getting behavior result: {e}')

    def _tick(self):
        try:
            if hasattr(self._tree, 'tick_once'):
                self._tree.tick_once()
            elif hasattr(self._tree, 'tick'):
                # call tick() which advances the tree once
                self._tree.tick()
            else:
                raise RuntimeError('py_trees BehaviourTree has no tick method')
            # log which nodes are currently RUNNING (updates only on change)
            try:
                self._log_tree_status()
            except Exception:
                pass
        except Exception as e:
            self.get_logger().error(f'Error ticking tree: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ControllerBTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down BT controller')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
