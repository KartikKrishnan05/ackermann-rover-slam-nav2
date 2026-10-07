"""
Launch file for the Robot State Publisher (URDF / Xacro).

This launch script:

* Resolves the path to the rover package.
* Reads the Xacro file *rover_roscontrol.urdf.xacro*, expands all
  macros and supplies the resulting URDF to the **robot_state_publisher**
  node.
* Declares the ``use_sim_time`` argument so that all TF timestamps can be
  based on the Gazebo simulation clock (when the simulation is used).
* Starts the ``robot_state_publisher`` node with the generated
  ``robot_description`` parameter and the chosen time-source.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Build and return a LaunchDescription that launches the
    robot_state_publisher with the URDF generated from the Xacro file.
    """
    # -----------------------------------------------------------------
    # Resolve the absolute path to the rover package's share directory.
    # This is the root for all package-local resources (URDF, meshes,
    # configuration files, …).
    # -----------------------------------------------------------------
    pkg_rover = get_package_share_directory("rover")

    # -----------------------------------------------------------------
    # Declare a launch argument that decides whether nodes should use the
    # simulation clock (published by Gazebo) or the system clock.
    # The default is ``true`` because the typical use-case is a Gazebo
    # simulation.
    # -----------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # -----------------------------------------------------------------
    # Path to the Xacro file that defines the robot description.
    # The file lives in ``rover/urdf/rover_roscontrol.urdf.xacro``.
    # -----------------------------------------------------------------
    xacro_path = os.path.join(pkg_rover, "urdf", "rover_roscontrol.urdf.xacro")

    # -----------------------------------------------------------------
    # Convert the Xacro file to a plain URDF string at launch time.
    # ``Command`` executes the shell command ``xacro <path>`` and substitutes
    # the resulting XML into the ``robot_description`` parameter.
    # -----------------------------------------------------------------
    robot_description_config = Command(["xacro ", xacro_path])

    # -----------------------------------------------------------------
    # Assemble the launch description:
    #   1. Declare the ``use_sim_time`` argument (so it appears in
    #      ``ros2 launch ... --show-args``).
    #   2. Start the ``robot_state_publisher`` node with two parameters:
    #        * ``use_sim_time`` - propagated from the launch argument.
    #        * ``robot_description`` - the URDF string generated above.
    # -----------------------------------------------------------------
    return LaunchDescription(
        [
            # -----------------------------------------------------------------
            # Launch argument declaration
            # -----------------------------------------------------------------
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            # -----------------------------------------------------------------
            # robot_state_publisher node
            # -----------------------------------------------------------------
            Node(
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
            ),
        ]
    )
