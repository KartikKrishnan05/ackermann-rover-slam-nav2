"""
Launch script for initializing the hardware-specific motor driver nodes.

Provides a flexible way to swap between different PCA9685 driver
implementations (Adafruit vs. Voidpp).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for motor drivers."""
    # Configuration Path
    config_path = os.path.join(
        get_package_share_directory("rover"),
        "config",
        "motor_driver_config.yaml",
    )

    voidpp_node = Node(
        package="rover",
        executable="motor_driver_voidpp",
        name="motor_driver_voidpp",
        parameters=[config_path]
    )

    return LaunchDescription(
        [voidpp_node]
    )
