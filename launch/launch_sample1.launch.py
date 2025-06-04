#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: launch_sample1.launch.py
Author: Tomoaki Fujino
"""

# Import modules (ROS2 related)
from launch import LaunchDescription
from launch_ros.actions import Node


# Import launch module
def generate_launch_description():
    """Method to generate launch description information

    Returns:
        launch.LaunchDescription: Object containing launch description information
    """
    return LaunchDescription(
        [
            # Configuration for talker node in lecture01_pkg
            Node(
                package="lecture01_pkg",  # Package to which the node belongs
                executable="talker",  # Executable file of the node
                name="talker",  # Name of the node
                output="screen",  # Output destination
            ),
            # Configuration for listener node in lecture01_pkg
            Node(
                package="lecture01_pkg",  # Package to which the node belongs
                executable="listener",  # Executable file of the node
                name="listener",  # Name of the node
                output="screen",  # Output destination
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
