"""Launch script for the Twist Multiplexer (twist_mux) node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for twist_mux."""
    pkg_share = get_package_share_directory("rover")
    config_path = os.path.join(pkg_share, "config", "twist_mux.yaml")

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[
            config_path,
            {"use_sim_time": False},
        ],
        remappings=[("/cmd_vel_out", "/cmd_vel_mux")],
    )

    return LaunchDescription([twist_mux_node])
