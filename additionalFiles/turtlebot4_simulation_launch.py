import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
import numpy as np
from launch.actions import GroupAction
from launch_ros.actions import PushROSNamespace

# this launch file starts gazebo, spawns the first turtlebot4 robot and launches the bridges for the objects position

# genome indetifier for the evolutionary algorithm
genome_id_arg = DeclareLaunchArgument('genome_id', default_value='test', description='Genome identifier')

turtlebot4_id_arg = DeclareLaunchArgument(
    'turtlebot4_id',
    default_value='tb1', # default value for the first robot
    description='TurtleBot4 identifier, use tbX where X is the number of the robot, e.g. tb1 for the first robot'
)

turtlebot4_id = LaunchConfiguration('turtlebot4_id')

def generate_launch_description():
    

    # launch turtlebot4 simulation with the first robot
    turtlebot4_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot4_gz_bringup'), 'launch'),
            '/turtlebot4_gz.launch.py']),
            launch_arguments={'world':'arena',
                               'z': '0.2',
                               'namespace': LaunchConfiguration('turtlebot4_id'),
                              }.items()
        
    )

    # bridges for the objects
    # maybe is better to keep it here since the tf_broadcaster refers to the tf for a specific robot
    ros_ign_object_bridge1 = Node(
        name='object1_pose_bridge',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/object1/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose']
    )

    ros_ign_object_bridge2 = Node(
        name='object2_pose_bridge',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/object2/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose']
    )

    ros_ign_object_bridge3 = Node(
        name='object3_pose_bridge',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/object3/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose']
    )

    ros_ign_object_bridge4 = Node(
        name='object4_pose_bridge',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/object4/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose']
    )

    ros_ign_object_bridge5 = Node(
        name='object5_pose_bridge',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/object5/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose']
    )

    ros_ign_object_bridge6 = Node(
        name='object6_pose_bridge',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/object6/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose']
    )

    ros_ign_object_bridge7 = Node(
        name='object7_pose_bridge',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/object7/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose']
    )


    check_simulation_node = Node(
        package='turtlebot4_controller',
        executable='check_simulation_node'
    )

    robot_sensors_node = GroupAction([
        PushROSNamespace(turtlebot4_id),
        
        # launch light sensor node (this node emulate light sensors, see code for more details)
        Node(
            package='turtlebot4_controller',
            executable='light_sensors_node',
            parameters=[{'use_sim_time': True}]
        ),

        # launch cliff sensor node (this node emulate cliff sensors, see code for more details)
        Node(
            package='turtlebot4_controller',
            executable='cliff_sensors_node',
            parameters=[{'use_sim_time': True}]
        ),

        # # launch ir sensor node (this node emulate ir sensors, see code for more details)
        Node(
            package='turtlebot4_controller',
            executable='ir_sensors_node',
            parameters=[{'use_sim_time': True}]
        ),

        # lanunch the fsm controller node
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot4_controller'), 'launch'),
            '/fsm_controller_launch.py'])
            )
        
        ])

    return LaunchDescription([
        genome_id_arg,
        turtlebot4_id_arg,
        turtlebot4_simulator,
        check_simulation_node,
        ros_ign_object_bridge1, ros_ign_object_bridge2, ros_ign_object_bridge3, ros_ign_object_bridge4,
        ros_ign_object_bridge5, ros_ign_object_bridge6, ros_ign_object_bridge7,
    ])
