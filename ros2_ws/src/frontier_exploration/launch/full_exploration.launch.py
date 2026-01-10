#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('frontier_exploration'),
                'launch',
                'simulation.launch.py'
            )
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('frontier_exploration'),
                'launch',
                'navigation.launch.py'
            )
        )
    )

    frontier = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('frontier_exploration'),
                'launch',
                'frontier_explorer.launch.py'
            )
        )
    )

    return LaunchDescription([
        simulation,
        navigation,
        frontier,
    ])
