from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Configuration - edit these values as needed
    WORLD = 'mission'
    SIM_DELAY = 5.0          # Delay before starting simulation
    TB2_ROBOT_DELAY = 15.0   # When to add tb2 robot  
    TB1_CONTROLLER_DELAY = 20.0  # When to start tb1 controller
    TB2_CONTROLLER_DELAY = 25.0  # When to start tb2 controller
    
    # FSM configuration string
    fsm_config = "--fsm-config --nstates 2 --s0 0 --rwm0 0 --n0 1 --n0x0 0 --c0x0 0 --p0x0 0 --s1 2 --n1 1 --n1x0 0 --c1x0 2 --p1x0 0"

    # 1. Start simulation with delay
    simulation_launch = TimerAction(
        period=SIM_DELAY,
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
                    'world': WORLD
                }.items()
            )
        ]
    )

    # 2. Add SECOND robot to existing simulation
    add_tb2_robot = TimerAction(
        period=TB2_ROBOT_DELAY,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('automode_tb4_sim'),
                        'launch',
                        'add_turtlebot4_simulation.launch.py'
                    ])
                ]),
                launch_arguments={
                    'world': WORLD,
                    'turtlebot4_id': 'tb2'
                }.items()
            )
        ]
    )

    # 3. Start first robot controller (tb1)
    tb1_controller_launch = TimerAction(
        period=TB1_CONTROLLER_DELAY,
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

    # 4. Start second robot controller (tb2)
    tb2_controller_launch = TimerAction(
        period=TB2_CONTROLLER_DELAY,
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
        simulation_launch,
        add_tb2_robot,
        tb1_controller_launch,
        tb2_controller_launch,
    ])