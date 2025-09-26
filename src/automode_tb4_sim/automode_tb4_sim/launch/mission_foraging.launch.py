from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Launch arguments for configuration
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='mission',
        description='Simulation world name'
    )
    
    sim_delay_arg = DeclareLaunchArgument(
        'sim_delay',
        default_value='5.0',
        description='Delay before starting simulation (seconds)'
    )
    
    tb1_delay_arg = DeclareLaunchArgument(
        'tb1_delay',
        default_value='20.0',  # sim_delay + controller_delay
        description='Total delay before starting tb1 controller (seconds)'
    )
    
    tb2_delay_arg = DeclareLaunchArgument(
        'tb2_delay',
        default_value='25.0',  # sim_delay + controller_delay + robot_spacing
        description='Total delay before starting tb2 controller (seconds)'
    )

    # Get launch configurations
    world = LaunchConfiguration('world')
    sim_delay = LaunchConfiguration('sim_delay')
    tb1_delay = LaunchConfiguration('tb1_delay')
    tb2_delay = LaunchConfiguration('tb2_delay')
    
    # FSM configuration string
    fsm_config = "--fsm-config --nstates 2 --s0 0 --rwm0 0 --n0 1 --n0x0 0 --c0x0 0 --p0x0 0 --s1 2 --n1 1 --n1x0 0 --c1x0 2 --p1x0 0"

    # 1. Start simulation with delay
    simulation_launch = TimerAction(
        period=sim_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('automode_tb4_sim'),
                        'launch',
                        'turtlebot4_simulation.launch.py'
                    ])
                ]),
                launch_arguments={
                    'world': world
                }.items()
            )
        ]
    )

    # 2. Start first robot controller (tb1)
    tb1_controller_launch = TimerAction(
        period=tb1_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('automode_controller'),
                        'launch',
                        'automode_tb4.launch.py'
                    ])
                ]),
                launch_arguments={
                    'fsm_config': fsm_config,
                    'robot_namespace': 'tb1'
                }.items()
            )
        ]
    )

    # 3. Start second robot controller (tb2)
    tb2_controller_launch = TimerAction(
        period=tb2_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('automode_controller'),
                        'launch',
                        'automode_tb4.launch.py'
                    ])
                ]),
                launch_arguments={
                    'fsm_config': fsm_config,
                    'robot_namespace': 'tb2'
                }.items()
            )
        ]
    )

    return LaunchDescription([
        # Launch arguments
        world_arg,
        sim_delay_arg,
        tb1_delay_arg,
        tb2_delay_arg,
        
        # Launch actions with timing
        simulation_launch,
        tb1_controller_launch,
        tb2_controller_launch,
    ])