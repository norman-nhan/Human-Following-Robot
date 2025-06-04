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

    # Get path to default param file
    default_params_path = os.path.join(
        get_package_share_directory("lecture04_pkg"),
        "config",
        "nav2_params_odom.yaml"
    )

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            name="params_file",
            default_value=default_params_path,
            description="Full path to the ROS2 parameters file to use",
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

        # Nav2 nodes
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "bt_navigator"
                ]
            }],
            output="screen",
        ),
    ])
