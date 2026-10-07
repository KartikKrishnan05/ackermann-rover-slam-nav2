"""
Launch file for the main Rover simulation in Gazebo.

This script orchestrates the entire simulation lifecycle using a specific
Gazebo-Plugin based workflow:

* Sets up environment variables for Gazebo resource paths.
* Launches Gazebo with a specific SDF world.
* Spawns the rover entity into the simulation.
* Configures the ROS-Gazebo bridge, automatically disabling the static
  map TF if SLAM is enabled.
* Launches the Robot State Publisher to broadcast the URDF transforms.
* Starts RViz for visualization.
* Optionally launches the SLAM Toolbox for mapping.
* Fixes the Gazebo LiDAR frame hierarchy with a static transform.
* Launches a Teleop node (either standard keyboard or custom) in a
  separate terminal window.

All behavior can be altered at launch time through the arguments
``use_sim_time``, ``slam``, ``rviz_config``, and ``use_custom_teleop``.
"""

import os

# -------------------------------------------------------------------------
# ROS 2 and launch imports
# -------------------------------------------------------------------------
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for the rover simulation.
    """

    # -----------------------------------------------------------------
    # Resolve locations of important packages
    # -----------------------------------------------------------------
    pkg_rover = get_package_share_directory("rover")
    pkg_slam_toolbox = get_package_share_directory("slam_toolbox")

    # Directory containing auxiliary launch files
    launch_file_dir = os.path.join(pkg_rover, "launch/gz_plugin")

    # -----------------------------------------------------------------
    # File paths
    # -----------------------------------------------------------------
    world_path = os.path.join(pkg_rover, "world", "world.sdf")

    default_rviz_config_path = os.path.join(
        pkg_rover, "rviz", "my_rviz_config.rviz"
    )

    slam_params_file = os.path.join(
        pkg_rover, "config", "mapper_params_online_async.yaml"
    )

    # -----------------------------------------------------------------
    # Launch Configuration Variables
    # -----------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_file = LaunchConfiguration("rviz_config")
    use_slam = LaunchConfiguration("slam")
    use_custom_teleop = LaunchConfiguration("use_custom_teleop")

    # -----------------------------------------------------------------
    # Environment Variables required by Gazebo
    # -----------------------------------------------------------------
    # GZ_SIM_RESOURCE_PATH: Points to the package parent and world dir
    # so Gazebo can find the mesh/SDF assets.
    pkg_rover_share_parent = os.path.dirname(pkg_rover)

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            pkg_rover_share_parent,
            ":",
            os.path.join(pkg_rover, "world"),
            ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ],
    )

    # -----------------------------------------------------------------
    # Nodes and Processes
    # -----------------------------------------------------------------

    # 1. Teleoperation Nodes (Conditional)
    #    Spawns a new terminal window (xterm) to capture keyboard input.

    # 1a. Default Teleop (runs if use_custom_teleop is 'false')
    default_teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",
        condition=UnlessCondition(use_custom_teleop),
    )

    # 1b. Custom Teleop (runs if use_custom_teleop is 'true')
    custom_teleop_node = Node(
        package="rover",
        executable="teleop",
        name="custom_teleop",
        output="screen",
        prefix="xterm -e",
        condition=IfCondition(use_custom_teleop),
    )

    # 2. Gazebo Simulator
    #    Runs Gazebo with the specified world file.
    gazebo_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "4", world_path],
        output="screen",
        # -r -> run simulation immediately
        # -v 4 -> verbosity level 4
    )

    # 3. Spawn Robot Entity
    #    Injects the robot model into the running Gazebo instance.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "rover",
            "-topic", "robot_description",  # Reads URDF from this topic
            "-x", "-5.0",
            "-y", "2.0",
            "-z", "0.2",
        ],
    )

    # 4. ROS-Gazebo Bridge
    #    Logic: If SLAM is enabled ('true'), we do NOT want the bridge
    #    to publish the map TF (SLAM handles it).
    should_publish_map_tf = PythonExpression(
        ["'false' if '", use_slam, "'.lower() == 'true' else 'true'"]
    )

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "ros_gz_bridge_gzplugin.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "publish_map_tf": should_publish_map_tf,
        }.items(),
    )

    # 5. Robot State Publisher
    #    Publishes static TFs from the Xacro/URDF.
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_file_dir,
                "robot_state_publisher_gzplugin.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # 6. RViz Visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # 7. SLAM Toolbox (Optional)
    #    Only launches if 'slam' argument is true.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, "launch", "online_async_launch.py")
        ),
        condition=IfCondition(use_slam),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    # 8. Static TF Fix for Gazebo LiDAR
    #    Connects the internal Gazebo lidar frame to the robot frame.
    lidar_tf_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_tf_fix",
        arguments=[
            "0", "0", "0", "0", "0", "0",
            "lidar_link", "rover/base_link/lidar_2d_v2"
        ],
        output="screen",
    )

    # -----------------------------------------------------------------
    # Assemble everything into a LaunchDescription
    # -----------------------------------------------------------------
    return LaunchDescription(
        [
            # -----------------------------------------------------------------
            # Launch Arguments
            # -----------------------------------------------------------------
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config_path,
                description="Path to the RViz configuration file",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="false",
                description="Enable SLAM Toolbox (disables bridge map TF)",
            ),
            DeclareLaunchArgument(
                "use_custom_teleop",
                default_value="false",
                description="Use custom teleop node",
            ),
            # -----------------------------------------------------------------
            # Environment Setup
            # -----------------------------------------------------------------
            gz_resource_path,

            # -----------------------------------------------------------------
            # Core Simulation Nodes
            # -----------------------------------------------------------------
            gazebo_sim,
            robot_state_publisher_launch,
            spawn_entity,
            bridge_launch,
            lidar_tf_fix,

            # -----------------------------------------------------------------
            # Tools and Navigation
            # -----------------------------------------------------------------
            rviz_node,
            slam_launch,

            # -----------------------------------------------------------------
            # Teleoperation (Pop-up Terminal)
            # -----------------------------------------------------------------
            custom_teleop_node,
            default_teleop_node,
        ]
    )
