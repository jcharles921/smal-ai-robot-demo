"""
Start the explorer node.

    ros2 launch home_explorer explore.launch.py              # simulation (default)
    ros2 launch home_explorer explore.launch.py sim:=false   # real robot
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory("home_explorer"), "config", "explorer.yaml")
    sim = ParameterValue(LaunchConfiguration("sim"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="true",
                              description="Use Gazebo's simulated clock (false on a real robot)"),
        Node(
            package="home_explorer",
            executable="explorer_node",
            name="explorer",
            output="screen",
            parameters=[params, {"use_sim_time": sim}],
        ),
    ])
