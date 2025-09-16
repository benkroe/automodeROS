import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushROSNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    genome_id_arg = DeclareLaunchArgument('genome_id', default_value='test', description='Genome identifier')
    turtlebot4_id_arg = DeclareLaunchArgument(
        'turtlebot4_id',
        default_value='tb1',
        description='TurtleBot4 identifier, use tbX where X is the number of the robot'
    )
    turtlebot4_id = LaunchConfiguration('turtlebot4_id')

    # Gazebo simulator with TurtleBot4
    turtlebot4_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot4_gz_bringup'), 'launch', 'turtlebot4_gz.launch.py')
        ]),
        launch_arguments={
            'world': 'arena',
            'z': '0.2',
            'namespace': turtlebot4_id,
        }.items()
    )

    # Object pose bridges
    object_bridges = [
        Node(
            name=f'object{i}_pose_bridge',
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[f'/model/object{i}/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose']
        ) for i in range(1, 8)
    ]

    # TF broadcaster launch (assumes you have a tf_broadcaster_launch.py)
    tf_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                os.path.dirname(__file__), 'tf_broadcaster_launch.py'
            )
        ]),
        launch_arguments={'turtlebot4_id': turtlebot4_id}.items()
    )

    # Sensor nodes and FSM controller, all in robot namespace
    robot_sensors_and_controller = GroupAction([
        PushROSNamespace(turtlebot4_id),
        Node(
            package='turtlebot4_controller',
            executable='light_sensors_node',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='turtlebot4_controller',
            executable='cliff_sensors_node',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='turtlebot4_controller',
            executable='ir_sensors_node',
            parameters=[{'use_sim_time': True}]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('turtlebot4_controller'), 'launch', 'fsm_controller_launch.py')
            ])
        )
    ])

    # Optional: Simulation check node
    check_simulation_node = Node(
        package='turtlebot4_controller',
        executable='check_simulation_node'
    )

    return LaunchDescription([
        genome_id_arg,
        turtlebot4_id_arg,
        turtlebot4_simulator,
        tf_broadcaster,
        robot_sensors_and_controller,
        check_simulation_node,
        *object_bridges,
    ])