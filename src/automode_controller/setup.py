from setuptools import find_packages, setup

package_name = 'automode_controller'

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
            'launch/automode_tb4.launch.py',
            'launch/config_turt_gz_foraging.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ben',
    maintainer_email='ben.kr@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_node = automode_controller.behavior_node:main',
            'condition_node = automode_controller.condition_node:main',
            'controller_node = automode_controller.controller_node:main',
            'ref_model_turtlebot4_gz = automode_controller.ref_model_turtlebot4_gz:main',
            'ref_model_turtlebot4_wb = automode_controller.ref_model_turtlebot4_wb:main',
            'ref_model_turtlebot4_real = automode_controller.ref_model_turtlebot4_real:main',
            'ref_model_turtlebot4_gz_nav = automode_controller.ref_model_turtlebot4_gz_nav:main',
        ],
    },
)