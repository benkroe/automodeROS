from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def _build_nodes(context):
    pkg_share = get_package_share_directory('automode_controller')

    config_file = LaunchConfiguration('config_file').perform(context)
    if not config_file:
        config_file = 'config_foraging.yaml'

    cfg_path = config_file
    if not os.path.isabs(cfg_path):
        cfg_path = os.path.join(pkg_share, 'launch', cfg_path)

    try:
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
    except FileNotFoundError:
        cfg = {}

    def _get_arg(name, default):
        value = LaunchConfiguration(name).perform(context)
        return value if value not in (None, '') else default

    namespace = _get_arg('robot_namespace', cfg.get('robot_namespace', ''))
    module_pkg = _get_arg('module_package', cfg.get('module_package', 'basic_modules'))
    fsm_config = _get_arg('fsm_config', cfg.get('fsm_config', '--fsm-config --nstates 1 --s0 0 --rwm0 50'))
    bt_config = _get_arg('bt_config', cfg.get('bt_config', ''))
    controller_type = _get_arg('controller_type', cfg.get('controller_type', 'fsm'))
    ref_model = _get_arg('ref_model', cfg.get('ref_model', 'ref_model_turtlebot4_gz'))

    condition_node = Node(
        package='automode_controller',
        executable='condition_node',
        name='condition_node',
        namespace=namespace,
        parameters=[
            {'use_sim_time': True},
            {'module_package': module_pkg}
        ],
        output='log',
    )

    behavior_node = Node(
        package='automode_controller',
        executable='behavior_node',
        name='behavior_node',
        namespace=namespace,
        parameters=[
            {'use_sim_time': True},
            {'module_package': module_pkg}
        ],
        output='log',
    )

    ref_model_node = Node(
        package='automode_controller',
        executable=ref_model,
        name='turtlebot4_reference_node',
        namespace=namespace,
        parameters=[{'use_sim_time': True}],
        output='log',
    )

    controller_node = Node(
        package='automode_controller',
        executable='controller_node',
        name='controller_node',
        namespace=namespace,
        parameters=[
            {'use_sim_time': True},
            {'fsm_config': fsm_config}
        ],
        output='log',
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'fsm'"])),
    )

    controller_bt_node = Node(
        package='automode_controller',
        executable='controller_bt_node',
        name='controller_bt_node',
        namespace=namespace,
        parameters=[
            {'use_sim_time': True},
            {'bt_config': bt_config}
        ],
        output='log',
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'bt'"]))
    )

    controller_startup_event = RegisterEventHandler(
        OnProcessStart(
            target_action=condition_node,
            on_start=[
                RegisterEventHandler(
                    OnProcessStart(
                        target_action=behavior_node,
                        on_start=[controller_node, controller_bt_node]
                    )
                )
            ]
        )
    )

    return [condition_node, behavior_node, ref_model_node, controller_startup_event]


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='config_foraging.yaml',
        description='YAML config file name in launch/ directory'
    )

    namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Namespace for robot nodes'
    )
    module_package_arg = DeclareLaunchArgument(
        'module_package',
        default_value='',
        description='Module package for condition and behavior nodes'
    )
    fsm_config_arg = DeclareLaunchArgument(
        'fsm_config',
        default_value='',
        description='Path to FSM config file'
    )
    bt_config_arg = DeclareLaunchArgument(
        'bt_config',
        default_value='',
        description='BT config string for controller_bt_node'
    )
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='',
        description='Controller type to start: "fsm" or "bt"'
    )
    ref_model_arg = DeclareLaunchArgument(
        'ref_model',
        default_value='',
        description='Reference model node executable'
    )

    return LaunchDescription([
        config_file_arg,
        namespace_arg,
        module_package_arg,
        fsm_config_arg,
        bt_config_arg,
        controller_type_arg,
        ref_model_arg,
        OpaqueFunction(function=_build_nodes),
    ])