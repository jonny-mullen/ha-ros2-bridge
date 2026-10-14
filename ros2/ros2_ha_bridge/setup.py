# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""ament_python packaging for ros2_ha_bridge."""

from glob import glob

from setuptools import find_packages, setup

package_name = 'ros2_ha_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    # The vendored contract schemas travel with the installed package so that
    # colcon test can validate against them; scripts/sync_contract.py keeps them
    # byte-identical to contract/schemas.
    package_data={package_name: ['schemas/*.schema.json']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files and the example parameter files, so that
        # "ros2 launch ros2_ha_bridge bridge.launch.py" works from an install.
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    # colcon only runs the test/ folder with pytest when the package declares pytest as a
    # test requirement; without this line it falls back to unittest and runs nothing.
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='jonnytinkererofthings',
    maintainer_email='jonathanmullen1@gmail.com',
    description='ROS 2 side of ha-ros2-bridge: a per-robot MQTT bridge node for Home Assistant.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = ros2_ha_bridge.bridge_node:main',
        ],
        # Executor plugins are found by name in this group (the "executor" parameter);
        # a dotted "package.module:Class" works without an entry point.
        'ros2_ha_bridge.executors': [
            'fake = ros2_ha_bridge.executors.fake:FakeExecutor',
            # Needs moveit_msgs at run time (package.xml, conditional); the module itself imports
            # rclpy and moveit_msgs lazily, so listing it here costs nothing without MoveIt.
            'moveit = ros2_ha_bridge.executors.moveit:MoveItExecutor',
        ],
    },
)
