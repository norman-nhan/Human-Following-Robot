#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: launch_sample2.launch.py
Author: Tomoaki Fujino
"""

# Import standard modules
import os

# Import ROS2 related modules
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


# Import launch module
def generate_launch_description():
    """Method to generate launch description information for launching multiple nodes

    Returns:
        launch.LaunchDescription: Object containing launch description information
    """

    # LaunchConfig settings (map)
    map_file = LaunchConfiguration(
        variable_name="map",  # Variable name
        # Default value (absolute path to map.yaml)
        default=os.path.join(
            os.getenv("HOME"),
            "ros2_lecture_ws",
            "map.yaml"
        )
    )

    # LaunchConfig settings (use_sim_time)
    use_sim_time = LaunchConfiguration(
        variable_name="use_sim_time",  # Variable name
        default="false"  # Default value
    )

    return LaunchDescription(
        [
            # Define argument (map)
            DeclareLaunchArgument(
                name="map",  # Key name
                default_value=os.path.join(
                    os.getenv("HOME"),
                    "ros2_lecture_ws",
                    "map.yaml"
                ),
                description="Full path to map file to load",  # Description
            ),
            # Define argument (use_sim_time)
            DeclareLaunchArgument(
                name="use_sim_time",  # Key name
                default_value="false",  # Default value
                description="Use simulation (Gazebo) clock if true",  # Description
            ),
            # navigation2.launch.py settings
            IncludeLaunchDescription(
                launch_description_source = PythonLaunchDescriptionSource(
                    # Path to the launch file to execute
                    os.path.join(
                        get_package_share_directory("turtlebot3_navigation2"),
                        "launch",
                        "navigation2.launch.py"
                    )
                ),
                launch_arguments={
                    "map": map_file,  # Path to map file
                    "use_sim_time": use_sim_time,  # Whether to use simulator time
                }.items(),
            ),
            # yasmin_viewer_node settings
            Node(
                package="yasmin_viewer",  # Package to which the node belongs
                executable="yasmin_viewer_node",  # Executable file of the node
                name="yasmin_viewer_node",  # Name of the node
                parameters=[{"use_sim_time": use_sim_time}],  # Parameter settings
                output="screen",  # Output destination
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
