# Import standard modules
import os

# Import ROS2 related modules
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # LaunchConfig settings (use_sim_time)
    use_sim_time = LaunchConfiguration(
        variable_name="use_sim_time",  # Variable name
        default="false"  # Default value
    )
    return LaunchDescription([
        Node(
            package='lecture04_pkg',
            executable='sm_main',
            name='state_machine_node',
            output='screen'
        ),
        # yasmin_viewer_node settings
        Node(
            package="yasmin_viewer",  # Package to which the node belongs
            executable="yasmin_viewer_node",  # Executable file of the node
            name="yasmin_viewer_node",  # Name of the node
            parameters=[{"use_sim_time": use_sim_time}],  # Parameter settings
            output="screen",  # Output destination
        ),
    ])
