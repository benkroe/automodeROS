from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def include_all(context, *args, **kwargs):
    robot_names_raw = LaunchConfiguration('robot_names').perform(context)
    robot_names = [n.strip() for n in robot_names_raw.split(',') if n.strip()]
    includes = []
    try:
        nav_pkg = get_package_share_directory('nav2_minimal_tb4_sim')
        nav_launch = os.path.join(nav_pkg, 'launch', 'multi_robot_simulation.launch.py')
        includes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments=[('rviz_for_all_robots', LaunchConfiguration('rviz_for_all_robots'))]
            )
        )
    except Exception:
        pass
    try:
        am_pkg = get_package_share_directory('automode_controller')
        automode_launch_path = os.path.join(am_pkg, 'launch', 'automode_tb4.launch.py')
        previous_waiter = None
        for i, ns in enumerate(robot_names):
            include_action = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(automode_launch_path),
                launch_arguments=[
                    ('robot_namespace', TextSubstitution(text=ns)),
                    ('config_file', LaunchConfiguration('config_file'))
                ]
            )
            waiter_cmd = "/bin/bash -lc 'until ros2 node list | grep -q \"/{ns}/controller\"; do sleep 0.25; done'".replace('{ns}', ns)
            waiter = ExecuteProcess(cmd=['/bin/bash', '-c', waiter_cmd], output='screen')
            if i == 0:
                includes.append(include_action)
                includes.append(waiter)
            else:
                reh = RegisterEventHandler(
                    OnProcessExit(
                        target_action=previous_waiter,
                        on_exit=[include_action, waiter]
                    )
                )
                includes.append(reh)
            previous_waiter = waiter
    except Exception:
        pass
    position_collector_node = Node(
        package='automode_tools',
        executable='position_collector',
        name='position_collector',
        output='screen',
    )
    includes.append(position_collector_node)
    return includes

def generate_launch_description():
    robot_names_arg = DeclareLaunchArgument(
        'robot_names', default_value='tb1,tb2,tb3,tb4',
        description='Comma-separated robot namespaces to launch'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz_for_all_robots', default_value='False',
        description='Pass through to nav2 launch whether to use combined rviz'
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value='config_flocking.yaml',
        description='YAML config file name in launch/ directory'
    )
    return LaunchDescription([
        robot_names_arg,
        rviz_arg,
        config_file_arg,
        OpaqueFunction(function=include_all),
    ])