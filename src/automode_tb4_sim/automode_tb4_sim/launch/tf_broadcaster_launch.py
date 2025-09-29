from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace

# Declare the launch argument
turtlebot4_id_arg = DeclareLaunchArgument(
    'turtlebot4_id',
    default_value='tb1',
    description='TurtleBot4 identifier, use tbX where X is the number of the robot, e.g. tb1 for the first robot'
)

# Setup function that creates the nodes with dynamic ID substitution
def launch_setup(context, *args, **kwargs):
    tb_id = LaunchConfiguration('turtlebot4_id').perform(context)

    return [
        GroupAction([
            PushROSNamespace(tb_id),

            # Bridge TF from Gazebo
            Node(
                package='ros_gz_bridge',
                name='robot_pose_tf_bridge',
                executable='parameter_bridge',
                arguments=[f'/model/{tb_id}/turtlebot4/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
                remappings=[(f'/model/{tb_id}/turtlebot4/pose', '/tf')],
                respawn=True,
                respawn_delay=1,
                parameters=[{'use_sim_time': True}]
            ),

            # Bridge PoseArray
            Node(
                package='ros_gz_bridge',
                name='robot_pose_poseArray_bridge',
                executable='parameter_bridge',
                arguments=[f'/model/{tb_id}/turtlebot4/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V'],
                parameters=[{'use_sim_time': True}]
            ),

            # LIGHT SENSORS -------------------------------------------------------------

            Node(
                package='tf2_ros',
                name='static_tf_lightSensorFrontL',
                executable='static_transform_publisher',
                arguments=[
                    '--x', '0.09', '--y', '0.08', '--z', '0.35',
                    '--yaw', '0', '--pitch', '0', '--roll', '0',
                    '--frame-id', f'{tb_id}/turtlebot4', '--child-frame-id', f'{tb_id}/light_sensor_front_left'
                ],
                parameters=[{'use_sim_time': True}]
            ),

            Node(
                package='tf2_ros',
                name='static_tf_lightSensorFrontR',
                executable='static_transform_publisher',
                arguments=[
                    '--x', '0.09', '--y', '-0.08', '--z', '0.35',
                    '--yaw', '0', '--pitch', '0', '--roll', '0',
                    '--frame-id', f'{tb_id}/turtlebot4', '--child-frame-id', f'{tb_id}/light_sensor_front_right'
                ],
                parameters=[{'use_sim_time': True}]
            ),

            Node(
                package='tf2_ros',
                name='static_tf_lightSensorBack',
                executable='static_transform_publisher',
                arguments=[
                    '--x', '-0.11', '--y', '0', '--z', '0.35',
                    '--yaw', '0', '--pitch', '0', '--roll', '0',
                    '--frame-id', f'{tb_id}/turtlebot4', '--child-frame-id', f'{tb_id}/light_sensor_back'
                ],
                parameters=[{'use_sim_time': True}]
            ),
        ])
    ]

# Final launch description
def generate_launch_description():
    return LaunchDescription([
        turtlebot4_id_arg,
        OpaqueFunction(function=launch_setup)
    ])
