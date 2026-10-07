"""
Launch file for the Robot State Publisher (URDF/Xacro).

This script processes the robot's Xacro description and publishes the
static transform tree required for visualization and simulation:

* Loads the *rover* package path.
* resolving the Xacro file path.
* Processes the Xacro file into a raw URDF XML
    string using the ``xacro`` command.
* Launches the ``robot_state_publisher`` node with the parsed URDF.

This ensures that all downstream nodes (RViz, Gazebo, Nav2) have access
to the robot's kinematic chain.
"""

import os

# -------------------------------------------------------------------------
# ROS 2 and launch imports
# -------------------------------------------------------------------------
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for the robot state publisher.
    """

    # -----------------------------------------------------------------
    # Resolve locations of important packages
    # -----------------------------------------------------------------
    pkg_rover = get_package_share_directory("rover")

    # -----------------------------------------------------------------
    # Launch arguments
    # -----------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # -----------------------------------------------------------------
    # File paths and Xacro processing
    # -----------------------------------------------------------------
    # Path to the Xacro file definition
    xacro_path = os.path.join(pkg_rover, "urdf", "rover_gzplugin.urdf.xacro")

    # Process Xacro to URDF using the Command substitution.
    # This ensures macros are expanded before
    #   the XML is passed to the publisher.
    robot_description_config = Command(["xacro ", xacro_path])

    # -----------------------------------------------------------------
    # Nodes
    # -----------------------------------------------------------------
    # robot_state_publisher: Publishes the static TFs (links/joints)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description_config,
            }
        ],
    )

    # -----------------------------------------------------------------
    # Return Launch Description
    # -----------------------------------------------------------------
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            rsp_node,
        ]
    )
