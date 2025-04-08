from setuptools import setup
import os
from glob import glob

package_name = 'camera_recorder_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='brl',
    author_email='brl@todo.todo',
    maintainer='brl',
    maintainer_email='brl@todo.todo',
    description='ROS2 package for recording multiple USB cameras',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_recorder_node = camera_recorder_ros.camera_recorder_node:main',
            'camera_recorder_service_node = camera_recorder_ros.camera_recorder_service_node:main',
        ],
    },
) 