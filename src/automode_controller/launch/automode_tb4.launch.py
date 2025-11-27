from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Namespace for robot nodes'
    )
    module_package_arg = DeclareLaunchArgument(
        'module_package',
        default_value='basic_modules',
        description='Module package for condition and behavior nodes'
    )
    fsm_config_arg = DeclareLaunchArgument(
        'fsm_config',
        default_value='--fsm-config --nstates 1 --s0 0 --rwm0 50 ',
        description='Path to FSM config file'
    )

    namespace = LaunchConfiguration('robot_namespace')
    module_pkg = LaunchConfiguration('module_package')
    fsm_config = LaunchConfiguration('fsm_config')

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
        executable='ref_model_turtlebot4',
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
        condition_node,
        behavior_node,
        ref_model_node,
        controller_startup_event,
    ])