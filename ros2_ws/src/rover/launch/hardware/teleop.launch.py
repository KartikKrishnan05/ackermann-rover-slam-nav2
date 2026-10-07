"""
Launch Description for selecting between Keyboard or Joystick teleop.

Manages the lifecycle and configuration of the input nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for teleoperation."""
    config_file = os.path.join(
        get_package_share_directory("rover"),
        "config",
        "teleop_config.yaml",
    )

    teleop_mode = LaunchConfiguration("mode")
    declare_mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="keyboard",
        description='Selection for input device: "joystick" or "keyboard"',
    )

    joystick_node = Node(
        package="rover",
        executable="teleop_joystick",
        name="joystick_teleop",
        parameters=[config_file],
        condition=IfCondition(
            PythonExpression(["'", teleop_mode, "' == 'joystick'"])
        ),
    )

    keyboard_node = Node(
        package="rover",
        executable="teleop_keyboard",
        name="keyboard_teleop",
        parameters=[config_file],
        prefix="xterm -e",
        condition=IfCondition(
            PythonExpression(["'", teleop_mode, "' == 'keyboard'"])
        ),
    )

    return LaunchDescription(
        [declare_mode_arg, joystick_node, keyboard_node]
    )
