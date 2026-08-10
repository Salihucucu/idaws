from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'idaws_nodes'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/templates', glob('templates/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IDAWS Team',
    maintainer_email='team@idaws.dev',
    description='IDAWS USV autonomy nodes for TEKNOFEST 2026',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pymavlink_controller_node = idaws_nodes.pymavlink_controller_node:main',
            'yolo_vision_node = idaws_nodes.yolo_vision_node:main',
            'mission_manager_node = idaws_nodes.mission_manager_node:main',
            'collision_avoidance_node = idaws_nodes.collision_avoidance_node:main',
            'mission_logger_node = idaws_nodes.mission_logger_node:main',
            'lidar_logger_node = idaws_nodes.lidar_logger_node:main',
            'web_ui_node = idaws_nodes.web_ui_node:main',
        ],
    },
)
