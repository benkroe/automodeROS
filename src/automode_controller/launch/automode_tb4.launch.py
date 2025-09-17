from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='tb1',
        description='Namespace for robot nodes'
    )
    module_package_arg = DeclareLaunchArgument(
        'module_package',
        default_value='basic_modules',
        description='Module package for condition and behavior nodes'
    )

    ns = LaunchConfiguration('robot_namespace')
    module_pkg = LaunchConfiguration('module_package')


    condition_node = Node(
        package='automode_controller',
        executable='condition_node',
        name='condition_node',
        namespace=ns,
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
        namespace=ns,
        parameters=[
            {'use_sim_time': True},
            {'module_package': module_pkg}
        ],
        output='log',

    )

    ref_model_node = Node(
        package='automode_controller',
        executable='ref_model_turtlebot4_gz',
        name='turtlebot4_reference_node',
        namespace=ns,
        parameters=[{'use_sim_time': True}],
        output='log',

    )

    controller_node = Node(
        package='automode_controller',
        executable='controller_node',
        name='controller_node',
        namespace=ns,
        parameters=[{'use_sim_time': True}],
        output='log',

    )

    delayed_controller_node = TimerAction(
        period=3.0,
        actions=[controller_node]
    )

    return LaunchDescription([
        namespace_arg,
        module_package_arg,
        condition_node,
        delayed_controller_node,
        behavior_node,
        ref_model_node
    ])