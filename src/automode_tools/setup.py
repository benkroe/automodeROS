from setuptools import find_packages, setup

package_name = 'automode_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        ('share/' + package_name + '/launch', [
            'launch/start_all.launch.py',
            'launch/start_foraging.launch.py',
            'launch/start_aggregation.launch.py',
            'launch/start_flocking.launch.py',
            'launch/start_dispersion.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ben',
    maintainer_email='ben.kroener@uni-kn.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_test_node = automode_tools.behavior_test_node:main',
            'condition_test_node = automode_tools.condition_test_node:main',
            'robotState_test_pub = automode_tools.robotState_test_pub:main',
            'categories_creator = automode_tools.categories_creator:main',
            'vicon_terminal_viz_node = automode_tools.vicon_terminal_viz_node:main',
            'group_evaluator = automode_tools.group_evaluator:main',
            'position_collector = automode_tools.position_collector:main',
            'scorer_node_foraging = automode_tools.scorer_node_foraging:main',
            'scorer_node_aggregation = automode_tools.scorer_node_aggregation:main',
            'timer_shutdown = automode_tools.timer_shutdown:main',
        ],
    },
)
