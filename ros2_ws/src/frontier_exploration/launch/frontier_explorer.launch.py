#!/usr/bin/env python3
"""
Frontier Exploration Launch File
Runs:
- map_listener
- frontier_detector
- frontier_selector
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # ------------------------------------------
        # 1. MAP LISTENER (optional helper)
        # ------------------------------------------
        Node(
            package='frontier_exploration',
            executable='map_listener',
            name='map_listener',
            output='screen',
        ),

        # ------------------------------------------
        # 2. FRONTIER DETECTOR
        # ------------------------------------------
        Node(
            package='frontier_exploration',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
        ),

        # ------------------------------------------
        # 3. FRONTIER SELECTOR
        # ------------------------------------------
        Node(
            package='frontier_exploration',
            executable='frontier_selector',
            name='frontier_selector',
            output='screen',
        ),
    ])
