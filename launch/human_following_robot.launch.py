# Import standard modules
import os

# Import ROS2 related modules
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),

        # State machine node
        Node(
            package="lecture04_pkg",
            executable="sm_main",
            name="state_machine_node",
            output="screen",
        ),

        # Yasmin viewer node
        Node(
            package="yasmin_viewer",
            executable="yasmin_viewer_node",
            name="yasmin_viewer_node",
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])