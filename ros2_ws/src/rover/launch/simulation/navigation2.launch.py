"""
Nav2 launch file for the Rover platform.

This launch script starts the full ROS 2 Navigation2 (Nav2) stack together
with an RViz2 instance that visualizes the robot, the map and the planned
paths.  All paths (URDF, map, parameter files, RViz configuration) are
resolved relative to the installed ROS 2 packages so the launch file works
both from a workspace and from an installed distribution.

Key responsibilities of this file
---------------------------------
1. **Expose launch arguments** that can be overridden from the command line:
   * ``map`` - absolute path to the occupancy-grid map (YAML format).
   * ``params_file`` - absolute path to the Nav2 parameter YAML file.
   * ``use_sim_time`` - if ``true`` the Nav2 nodes will subscribe to the
     Gazebo simulation clock instead of the system clock.
2. **Include Nav2's own bring-up launch file** (``bringup_launch.py``) and pass
   the arguments defined above to it.
3. **Start RViz2** with a predefined view configuration so that developers can
   immediately inspect the robot, the map and the planner output.
4. Keep all imports and variable names explicit for readability and easy
   maintenance.
"""

import os

# -------------------------------------------------------------------------
# ROS-2 and launch imports
# -------------------------------------------------------------------------
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Build and return a ``LaunchDescription`` that launches Nav2 and RViz2.
    """

    # -----------------------------------------------------------------
    # Resolve the location of the relevant ROS 2 packages.
    #   pkg_rover        - contains the map and custom Nav2 parameter file.
    #   nav2_bringup_dir - contains Nav2’s own launch files and RViz config.
    # -----------------------------------------------------------------
    pkg_rover = get_package_share_directory("rover")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # -----------------------------------------------------------------
    # Launch arguments (can be overridden at runtime)
    # -----------------------------------------------------------------
    # use_sim_time - tells all nodes to use the Gazebo simulation clock.
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # map - full path to the YAML map file.
    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(pkg_rover, "maps", "my_office_map291225.yaml"),
    )

    # params_file - full path to the Nav2 parameter file.
    param_file_name = "accerPlanner.yaml"
    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(pkg_rover, "config", param_file_name),
    )

    # -----------------------------------------------------------------
    # Paths needed for the included Nav2 launch and the RViz configuration.
    # -----------------------------------------------------------------
    # Nav2’s bring-up launch file resides in the ``launch`` folder of the
    # ``nav2_bringup`` package.
    nav2_launch_file_dir = os.path.join(nav2_bringup_dir, "launch")

    # The default RViz view that ships with Nav2.
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, "rviz", "nav2_default_view.rviz"
    )

    # -----------------------------------------------------------------
    # Assemble the LaunchDescription.
    # The order of entries matters only insofar that arguments are declared
    # before they are used.
    # -----------------------------------------------------------------
    return LaunchDescription(
        [
            # ---- Declare launch arguments ----------------------------------
            DeclareLaunchArgument(
                "map",
                default_value=map_dir,
                description="Absolute path to the map file (YAML).",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=param_dir,
                description="Absolute path to the Nav2 parameter file (YAML).",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="If true, Nav2 nodes use simulation time.",
            ),
            # ---- Include Nav2's bring-up launch file -----------------------
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    # ``bringup_launch.py`` is the entry point for the Nav2.
                    [nav2_launch_file_dir, "/bringup_launch.py"]
                ),
                launch_arguments={
                    "map": map_dir,
                    "use_sim_time": use_sim_time,
                    "params_file": param_dir,
                }.items(),
            ),
            # ---- Start RViz2 -----------------------------------------------
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                # ``-d`` tells RViz2 to load the specified configuration.
                arguments=["-d", rviz_config_dir],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
