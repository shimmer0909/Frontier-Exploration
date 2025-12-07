from setuptools import setup
import os
from glob import glob

package_name = 'frontier_exploration'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ashi Gupta',
    maintainer_email='contact.ashi.gupta@gmail.com',
    description='Frontier-based exploration system for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_listener = frontier_explorer.map_listener:main',
            'frontier_detector = frontier_explorer.frontier_detector:main',
            'frontier_selector = frontier_explorer.frontier_selector:main',
        ],
    },
)
