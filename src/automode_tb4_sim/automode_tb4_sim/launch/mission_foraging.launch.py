from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction,
    RegisterEventHandler
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart, OnProcessExit
import os

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
    
    controller_delay_arg = DeclareLaunchArgument(
        'controller_delay',
        default_value='15.0',
        description='Delay before starting first controller (seconds)'
    )
    
    robot_spacing_arg = DeclareLaunchArgument(
        'robot_spacing',
        default_value='5.0',
        description='Delay between starting each robot controller (seconds)'
    )

    # Get launch configurations
    world = LaunchConfiguration('world')
    sim_delay = LaunchConfiguration('sim_delay')
    controller_delay = LaunchConfiguration('controller_delay')
    robot_spacing = LaunchConfiguration('robot_spacing')
    
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

    # 2. Start first robot controller (tb1) after simulation + controller_delay
    tb1_controller_launch = TimerAction(
        period=[sim_delay, ' + ', controller_delay],  # Sum of delays
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

    # 3. Start second robot controller (tb2) after additional robot_spacing delay
    tb2_controller_launch = TimerAction(
        period=[sim_delay, ' + ', controller_delay, ' + ', robot_spacing],
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
        controller_delay_arg,
        robot_spacing_arg,
        
        # Launch actions with timing
        simulation_launch,
        tb1_controller_launch,
        tb2_controller_launch,
    ])