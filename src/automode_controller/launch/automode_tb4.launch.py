from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    pkg_share = get_package_share_directory('automode_controller')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='config_foraging.yaml',
        description='YAML config file name in launch/ directory'
    )
    config_file = LaunchConfiguration('config_file')
    default_cfg_path = os.path.join(pkg_share, 'launch', 'config_foraging.yaml')

    # load configuration from YAML file
    try:
        with open(default_cfg_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
    except FileNotFoundError:
        cfg = {}

    # Declare launch arguments (defaults come from YAML)
    namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value=cfg.get('robot_namespace', ''),
        description='Namespace for robot nodes'
    )
    module_package_arg = DeclareLaunchArgument(
        'module_package',
        default_value=cfg.get('module_package', 'basic_modules'),
        description='Module package for condition and behavior nodes'
    )
    fsm_config_arg = DeclareLaunchArgument(
        'fsm_config',
        default_value=cfg.get('fsm_config', '--fsm-config --nstates 1 --s0 0 --rwm0 50'),
        description='Path to FSM config file'
    )
    bt_config_arg = DeclareLaunchArgument(
        'bt_config',
        default_value=cfg.get('bt_config', ''),
        description='BT config string for controller_bt_node'
    )
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value=cfg.get('controller_type', 'fsm'),
        description='Controller type to start: "fsm" or "bt"'
    )
    ref_model_arg = DeclareLaunchArgument(
        'ref_model',
        default_value=cfg.get('ref_model', 'ref_model_turtlebot4_gz'),
        description='Reference model node executable'
    )

    namespace = LaunchConfiguration('robot_namespace')
    module_pkg = LaunchConfiguration('module_package')
    fsm_config = LaunchConfiguration('fsm_config')
    bt_config = LaunchConfiguration('bt_config')
    controller_type = LaunchConfiguration('controller_type')
    ref_model = LaunchConfiguration('ref_model')
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    # Start controller after both condition and behavior nodes are started
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

    return LaunchDescription([
        config_file_arg,
        fsm_config_arg,
        bt_config_arg,
        controller_type_arg,
        namespace_arg,
        module_package_arg,
        ref_model_arg,
        condition_node,
        behavior_node,
        ref_model_node,
        controller_startup_event,
    ])