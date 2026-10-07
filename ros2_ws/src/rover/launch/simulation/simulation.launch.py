"""
Launch file for the main Rover simulation in Gazebo.

This launch script wires together all components required to run a
complete simulation of the *rover* robot:

* Sets the environment variables that Gazebo uses to locate the rover's
  world files and the ROS-2-Control system plugins.
* Starts Gazebo with the specified SDF world.
* Spawns the rover model in the simulation (via ros_gz_sim → create).
* Launches the robot_state_publisher to publish TF frames from the URDF.
* Starts the ROS-Gazebo bridge and decides whether the bridge should publish
  the ``map`` TF (only when both SLAM and Nav2 are disabled).
* Spawns the ROS-Control controllers:
  - joint_state_broadcaster
  - ackermann_steering_controller (with its controller-manager yaml).
* Optionally launches the SLAM Toolbox (online-async) and/or the Nav2 stack,
  depending on the launch arguments ``slam`` and ``nav2``.
* Always runs ``twist_mux`` and a custom ``twist_stamper`` node to
  multiplex and timestamp velocity commands.
* Publishes a static transform that fixes the LiDAR frame hierarchy.

All behaviour can be altered at launch time through the arguments
``use_sim_time``, ``slam`` and ``nav2``.
"""

import os

# -------------------------------------------------------------------------
# ROS 2 and launch imports
# -------------------------------------------------------------------------
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)  # noqa: E402
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Build and return a ``LaunchDescription`` that contains every action,
    node and environment setting required for the rover simulation.
    """

    # -----------------------------------------------------------------
    # Resolve locations of important packages
    # -----------------------------------------------------------------
    # rover               - our own package containing the robot description,
    #                       configuration files and additional launch files.
    # slam_toolbox        - external package that provides the SLAM Toolbox.
    # launch_file_dir     - directory that holds the rover's auxiliary launch
    #                       files (e.g. robot_state_publisher.launch.py).
    # -----------------------------------------------------------------
    pkg_rover = get_package_share_directory("rover")
    pkg_slam_toolbox = get_package_share_directory("slam_toolbox")
    launch_file_dir = os.path.join(pkg_rover, "launch", "simulation")

    # -----------------------------------------------------------------
    # Launch arguments (can be overridden on the command line)
    # -----------------------------------------------------------------
    # use_sim_time - make all nodes use simulation time from Gazebo.
    # slam         - start the SLAM Toolbox (True/False).
    # nav2         - start the Nav2 stack (True/False).
    # -----------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_slam = LaunchConfiguration("slam")
    use_nav2 = LaunchConfiguration("nav2")

    # -----------------------------------------------------------------
    # File paths that are later passed to nodes or launch files
    # -----------------------------------------------------------------
    twist_mux_config = os.path.join(pkg_rover, "config", "twist_mux.yaml")
    world_path = os.path.join(pkg_rover, "world", "world.sdf")
    slam_config_file = os.path.join(
        pkg_rover, "config", "mapper_params_online_async.yaml"
    )
    default_rviz_config_path = os.path.join(
        pkg_rover, "rviz", "my_rviz_config.rviz"
    )

    # -----------------------------------------------------------------
    # Set environment variables required by Gazebo
    # -----------------------------------------------------------------
    # GZ_SIM_RESOURCE_PATH   - where Gazebo looks for SDF/URDF files.
    # GZ_SIM_SYSTEM_PLUGIN_PATH - where Gazebo looks for system plugins
    #                          (e.g. the gz_ros2_control plugin library).
    # -----------------------------------------------------------------
    pkg_rover_share_parent = os.path.dirname(pkg_rover)
    pkg_gz_ros2_control = get_package_prefix("gz_ros2_control")
    gz_ros2_control_lib_path = os.path.join(pkg_gz_ros2_control, "lib")

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            pkg_rover_share_parent,
            ":",
            os.path.join(pkg_rover, "world"),
            ":",
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ],
    )

    gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            gz_ros2_control_lib_path,
            ":",
            os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
        ],
    )

    # -----------------------------------------------------------------
    # -----------------------------------------------------------------
    # Definition of the nodes / processes that will be launched
    # -----------------------------------------------------------------

    # 1. Start Gazebo simulator with the specified world file.
    gazebo_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "4", world_path],
        output="screen",
        # -r   → run the simulation immediately
        # -v 4 → set verbosity level to 4 (debug output)
    )

    # 2. Spawn the robot URDF into Gazebo.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "rover",
            "-topic", "robot_description",   # URDF is read from this topic
            "-x", "-5.0",
            "-y", "2.0",
            "-z", "0.2",
        ],
    )

    # 3. Launch robot_state_publisher (provides TF frames from the URDF).
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # 4. ROS ↔ Gazebo bridge.
    #    The bridge publishes the map TF only when both SLAM and Nav2 are
    #    disabled.  This logic is expressed with a PythonExpression that
    #    evaluates to the strings "true" or "false".
    should_publish_map_tf = PythonExpression(
        [
            "'true' if (not '",
            use_slam,
            "'.lower() == 'true') and (not '",
            use_nav2,
            "'.lower() == 'true') else 'false'",
        ]
    )

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "ros_gz_bridge.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "publish_map_tf": should_publish_map_tf,
        }.items(),
    )

    # 5. ROS-Control controllers.
    #    a) joint_state_broadcaster - publishes joint states for TF.
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    #    b) ackermann_steering_controller - receives velocity commands and
    #       translates them to wheel steering angles.
    ackermann_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--param-file",
            os.path.join(pkg_rover, "config", "ros2_controllers.yaml"),
            "--controller-manager-timeout",
            "60",  # seconds to wait for the controller manager to be ready
        ],
        output="screen",
    )

    # 6. Optional navigation components.

    #    a) SLAM Toolbox - starts the online-async SLAM node.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, "launch", "online_async_launch.py")
        ),
        condition=IfCondition(use_slam),  # Only launch when slam == True
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_config_file,
        }.items(),
    )

    #    b) Nav2 - starts the standard Navigation2 stack.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "navigation2.launch.py")
        ),
        condition=IfCondition(use_nav2),  # Only launch when nav2 == True
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # 7. twist_mux - multiplexes several /cmd_vel sources into a single
    #    ``cmd_vel_mux`` topic.
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        parameters=[twist_mux_config],
        remappings=[("cmd_vel_out", "cmd_vel_mux")],
    )

    # 8. twist_stamper - adds a (simulation) timestamp to incoming Twist
    #    messages so that downstream nodes see the correct time.
    twist_stamper_node = Node(
        package="rover",
        executable="twist_stamper",
        name="twist_stamper",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/cmd_vel_mux", "/cmd_vel_mux"),
            (
                "/ackermann_steering_controller/reference",
                "/ackermann_steering_controller/reference",
            ),
        ],
    )

    # 9. Static TF publisher that fixes the LiDAR frame hierarchy.
    #    All arguments are zero - the transform is effectively an identity,
    #    but it creates the missing link between ``lidar_link`` and the
    #    ``rover/base_footprint/lidar_2d_v2`` frame.
    lidar_tf_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",  # x translation
            "0",  # y translation
            "0",  # z translation
            "0",  # roll
            "0",  # pitch
            "0",  # yaw
            "lidar_link",
            "rover/base_footprint/lidar_2d_v2",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # -----------------------------------------------------------------
    # Assemble everything into a LaunchDescription
    # -----------------------------------------------------------------
    return LaunchDescription(
        [
            # -----------------------------------------------------------------
            # Declare launch arguments (with sensible defaults and description)
            # -----------------------------------------------------------------
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time published by Gazebo.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config_path,
                description="Path to the default RViz configuration file.",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="False",
                description="Activate the SLAM Toolbox.",
            ),
            DeclareLaunchArgument(
                "nav2",
                default_value="True",
                description="Activate the Nav2 navigation stack.",
            ),
            # -----------------------------------------------------------------
            # Environment variables required by Gazebo
            # -----------------------------------------------------------------
            gz_resource_path,
            gz_plugin_path,
            # -----------------------------------------------------------------
            # Nodes / processes (order matters for deterministic startup)
            # -----------------------------------------------------------------
            gazebo_sim,
            robot_state_publisher_launch,
            spawn_entity,
            bridge_launch,
            slam_launch,
            joint_state_broadcaster,
            ackermann_controller,
            lidar_tf_fix,
            nav2_launch,
            twist_mux_node,
            twist_stamper_node,
        ]
    )
