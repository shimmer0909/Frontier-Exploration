#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_house.launch.py'
            )
        )
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_description'),
                'launch',
                'robot_state_publisher.launch.py'
            )
        )
    )

    fake_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_fake_node'),
                'launch',
                'turtlebot3_fake_node.launch.py'
            )
        )
    )

    return LaunchDescription([
        turtlebot3_gazebo,
        robot_state_publisher,
        fake_odom,
    ])
