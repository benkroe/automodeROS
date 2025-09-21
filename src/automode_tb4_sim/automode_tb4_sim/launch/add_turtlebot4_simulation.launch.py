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
            executable='cliff_sensors_node',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='automode_tb4_sim',
            executable='ir_sensors_node',
            parameters=[{'use_sim_time': True}]
        ),
    ])

    #  # Add IR sensor bridges for tb2
    # ir_sensor_bridges = [
    #     Node(
    #         package='ros_gz_bridge',
    #         executable='parameter_bridge',
    #         name=f'ir_intensity_{sensor}_bridge',
    #         arguments=[
    #             f'/world/white/model/tb2/turtlebot4/link/ir_intensity_{sensor}/sensor/ir_intensity_{sensor}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
    #         ],
    #         parameters=[{'use_sim_time': True}]
    #     )
    #     for sensor in [
    #         'front_center_left',
    #         'front_center_right',
    #         'front_left',
    #         'front_right',
    #         'left',
    #         'right',
    #         'side_left'
    #     ]
    # ]

    pkg_irobot_create_gz_bringup = get_package_share_directory('irobot_create_gz_bringup')
    create3_ros_gz_bridge_launch = PathJoinSubstitution(
        [pkg_irobot_create_gz_bringup, 'launch', 'create3_ros_gz_bridge.launch.py']
    )

    # Set robot_name and dock_name to match your Gazebo model names
    robot_name = [turtlebot4_id, '/turtlebot4']
    dock_name = [turtlebot4_id, '/standard_dock']
    world = 'white'  # or 'warehouse', depending on your setup

    create3_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_ros_gz_bridge_launch]),
        launch_arguments=[
            ('robot_name', robot_name),
            ('dock_name', dock_name),
            ('namespace', turtlebot4_id),
            ('world', world)
        ]
    )

    return LaunchDescription([
        static_tf_arena,
        turtlebot4_id_arg,
        turtlebot4_simulator,
        ros_gz_bridge,
        tf_broadcaster,
        robot_sensors_node,
        #*ir_sensor_bridges,
    ])