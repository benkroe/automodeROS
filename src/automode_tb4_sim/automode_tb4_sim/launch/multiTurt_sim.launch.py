import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace

def robot_group(robot_ns, idx, delay=0.0):
    # Declare a launch argument for the namespace
    robot_ns_arg = DeclareLaunchArgument(robot_ns, default_value=robot_ns)
    robot_ns_lc = LaunchConfiguration(robot_ns)

    if idx == 0:
        # First robot: full simulation launch
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('turtlebot4_gz_bringup'), 'launch', 'turtlebot4_gz.launch.py')
            ]),
            launch_arguments={
                'namespace': robot_ns_lc,
                'world': 'white',
                'z': '0.2'
            }.items()
        )
    else:
        # Second robot: spawn only
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('turtlebot4_gz_bringup'), 'launch', 'turtlebot4_spawn.launch.py')
            ]),
            launch_arguments={
                'namespace': robot_ns_lc,
                'x': str(idx * 1.5),
                'y': str(idx * 1.5),
                'z': '0.2'
            }.items()
        )

    tf_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(os.path.dirname(__file__), 'tf_broadcaster_launch.py')
        ]),
        launch_arguments={'turtlebot4_id': robot_ns_lc}.items()
    )

    robot_sensors_and_controller = GroupAction([
        PushROSNamespace(robot_ns_lc),
        Node(
            package='automode_tb4_sim',
            executable='light_sensors_node',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='automode_tb4_sim',
            executable='cliff_sensors_node',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='automode_tb4_sim',
            executable='ir_sensors_node',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='automode_tb4_sim',
            executable='ground_sensor_node',
            parameters=[{'use_sim_time': True}]
        )
    ])

    group = [robot_ns_arg, robot_launch, tf_broadcaster, robot_sensors_and_controller]
    if delay > 0.0:
        return [TimerAction(period=delay, actions=group)]
    else:
        return group

def generate_launch_description():
    genome_id_arg = DeclareLaunchArgument('genome_id', default_value='test', description='Genome identifier')

    static_tf_arena = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'white'],
        name='static_tf_arena'
    )

    robot_launches = []
    robot_names = [f'tb{i}' for i in range(1, 3)]  # Only two robots: tb1 and tb2
    for idx, robot_ns in enumerate(robot_names):
        delay = 20.0
        robot_launches.extend(robot_group(robot_ns, idx, delay=delay))

    return LaunchDescription([
        static_tf_arena,
        genome_id_arg,
        *robot_launches,
    ])