from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    pkg_share = get_package_share_directory('automode_controller')
    default_cfg_path = os.path.join(pkg_share, 'launch', 'config_turt_gz_foraging.yaml')

    # load configuration from YAML file
    pkg_share = get_package_share_directory('automode_controller')
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
    ref_model_arg = DeclareLaunchArgument(
        'ref_model',
        default_value=cfg.get('ref_model', 'ref_model_turtlebot4_gz'),
        description='Reference model node executable'
    )

    namespace = LaunchConfiguration('robot_namespace')
    module_pkg = LaunchConfiguration('module_package')
    fsm_config = LaunchConfiguration('fsm_config')
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
    )

    # Start controller after both condition and behavior nodes are started
    controller_startup_event = RegisterEventHandler(
        OnProcessStart(
            target_action=condition_node,
            on_start=[
                RegisterEventHandler(
                    OnProcessStart(
                        target_action=behavior_node,
                        on_start=[controller_node]
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        fsm_config_arg,
        namespace_arg,
        module_package_arg,
        ref_model_arg,
        condition_node,
        behavior_node,
        ref_model_node,
        controller_startup_event,
    ])