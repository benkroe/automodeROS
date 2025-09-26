import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushROSNamespace
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution


# Declare the namespace for the second robot
turtlebot4_id_arg = DeclareLaunchArgument(
    'turtlebot4_id',
    default_value='tb2',
    description='TurtleBot4 identifier for the second robot'
)
turtlebot4_id = LaunchConfiguration('turtlebot4_id')

world_arg = DeclareLaunchArgument(
    'world',
    default_value='mission',
    description='Simulation world name'
)
world = LaunchConfiguration('world')

def generate_launch_description():
        # Static transform for arena/world
    static_tf_arena = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', LaunchConfiguration('world')],
        name='static_tf_arena'
    )

    # Launch the second TurtleBot4 in Gazebo
    turtlebot4_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot4_gz_bringup'), 'launch', 'turtlebot4_spawn.launch.py')
        ]),
        launch_arguments={
            'namespace': turtlebot4_id,
            'x': '1.0',
            'y': '1.0',
            'z': '0.2',
            'world': world
        }.items()
    )

    tf_broadcaster = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(
            os.path.dirname(__file__), 'tf_broadcaster_launch.py'
        )
    ]),
    launch_arguments={'turtlebot4_id': turtlebot4_id}.items()
)

    # Sensor nodes for the second robot
    robot_sensors_node = GroupAction([
        PushROSNamespace(turtlebot4_id),
        Node(
            package='automode_tb4_sim',
            executable='light_sensors_node',
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
        ),
        Node(
            package='automode_tb4_sim',
            executable='neighbour_sensor_node',
            parameters=[{'use_sim_time': True}]
        ),
    ])


    # Set robot_name and dock_name to match your Gazebo model names
    robot_name = [turtlebot4_id, '/turtlebot4']
    dock_name = [turtlebot4_id, '/standard_dock']

    return LaunchDescription([
        world_arg,
        static_tf_arena,
        turtlebot4_id_arg,
        turtlebot4_simulator,
        tf_broadcaster,
        robot_sensors_node,
    ])
