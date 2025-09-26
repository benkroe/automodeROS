from setuptools import find_packages, setup

package_name = 'automode_tb4_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch', [
            'automode_tb4_sim/launch/turtlebot4_simulation.launch.py',
            'automode_tb4_sim/launch/tf_broadcaster_launch.py',
            'automode_tb4_sim/launch/add_turtlebot4_simulation.launch.py',
        ]),
        # Install worlds
        ('share/' + package_name + '/worlds', [
            'automode_tb4_sim/worlds/white.sdf',
            'automode_tb4_sim/worlds/mission.sdf',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ben',
    maintainer_email='ben@todo.todo',
    description='Simulation and sensor nodes for TurtleBot4 in Gazebo',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Sensor nodes
            'light_sensors_node = automode_tb4_sim.tb4_control.sensor_nodes.light_sensors_node:main',
            'cliff_sensors_node = automode_tb4_sim.tb4_control.sensor_nodes.cliff_sensors_node:main',
            'ir_sensors_node = automode_tb4_sim.tb4_control.sensor_nodes.ir_sensors_node:main',
            'ground_sensor_node = automode_tb4_sim.tb4_control.sensor_nodes.ground_sensor_node:main',
            'neighbour_sensor_node = automode_tb4_sim.tb4_control.sensor_nodes.neighbour_sensor_node:main',
        ],
    },
)