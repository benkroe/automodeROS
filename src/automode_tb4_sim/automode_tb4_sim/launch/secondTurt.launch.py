import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushROSNamespace
from launch.substitutions import LaunchConfiguration

# Declare the namespace for the second robot
turtlebot4_id_arg = DeclareLaunchArgument(
    'turtlebot4_id',
    default_value='tb2',
    description='TurtleBot4 identifier for the second robot'
)
turtlebot4_id = LaunchConfiguration('turtlebot4_id')

def generate_launch_description():
        # Static transform for arena/world
    static_tf_arena = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'white'],
        name='static_tf_arena'
    )

    # Launch the second TurtleBot4 in Gazebo
    turtlebot4_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot4_gz_bringup'), 'launch', 'turtlebot4_spawn.launch.py')
        ]),
        launch_arguments={
            'namespace': turtlebot4_id,
            'x': '2.0',
            'y': '2.0',
            'z': '0.2'
        }.items()
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
            executable='cliff_sensors_node',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='automode_tb4_sim',
            executable='ir_sensors_node',
            parameters=[{'use_sim_time': True}]
        ),
    ])

    return LaunchDescription([
        static_tf_arena,
        turtlebot4_id_arg,
        turtlebot4_simulator,
        robot_sensors_node,
    ])