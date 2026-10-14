# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""ament_python packaging for ros2_ha_bridge."""

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO maintainer name',
    maintainer_email='TODO-maintainer@example.com',
    description='ROS 2 side of ha-ros2-bridge: a per-robot MQTT bridge node for Home Assistant.',
    license='Apache-2.0',
    entry_points={
        # bridge_node.py arrives in phase 2; the entry point is declared now so the
        # installed layout never changes.
        'console_scripts': [
            'bridge_node = ros2_ha_bridge.bridge_node:main',
        ],
    },
)
