import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    pkg_rover = get_package_share_directory("rover")
    pkg_slam_toolbox = get_package_share_directory("slam_toolbox")
    pkg_rplidar = get_package_share_directory("rplidar_ros")
    pkg_nav2 = get_package_share_directory("nav2_bringup")

    # Launch Configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav_mode = LaunchConfiguration("mode")

    # Hardcoded Paths
    xacro_path = os.path.join(pkg_rover, "urdf", "rover_roscontrol.urdf.xacro")
    default_slam_config = os.path.join(
        pkg_rover, "config", "mapper_params_online_async.yaml"
    )
    static_map_path = os.path.join(pkg_rover, "maps", "3rdfloor_office.yaml")
    nav_params_path = os.path.join(pkg_rover, "config", "accerPlanner_hardware.yaml")
    # Nav2 params for slam_nav mode — no AMCL, no static map server
    slam_nav_params_path = os.path.join(pkg_rover, "config", "nav2_slam_hardware.yaml")

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": Command(["xacro ", xacro_path]),
            }
        ],
    )

    # 2. LiDAR Driver
    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar, "launch", "rplidar_a3_launch.py")
        )
    )

    # 3. RF2O Laser Odometry
    rf2o_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="log",
        arguments=["--ros-args", "--log-level", "fatal"],
        parameters=[
            {
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom",
                "publish_tf": True,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 20.0,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # 4. SLAM Toolbox — builds the live map and publishes map→odom TF.
    #    Active for both 'mapping' (map-only) and 'slam_nav' (map + navigate).
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": default_slam_config,
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", nav_mode, "' == 'mapping' or '", nav_mode, "' == 'slam_nav'"]
            )
        ),
    )

    # 5. Nav2 — classic mode: pre-saved static map + AMCL localization.
    #    Use when the environment is already mapped (mode: nav).
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "bringup_launch.py")
        ),
        condition=IfCondition(PythonExpression(["'", nav_mode, "' == 'nav'"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav_params_path,
            "map": static_map_path,
            "autostart": "true",
        }.items(),
    )

    # 6. Nav2 — slam_nav mode: navigation-only stack, no AMCL, no map_server.
    #    slam_toolbox (above) owns the /map topic and the map→odom TF.
    #    navigation_launch.py starts only planners, controllers, costmaps,
    #    and behavior trees — everything needed to accept and execute RViz goals.
    nav2_navigation_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "navigation_launch.py")
        ),
        condition=IfCondition(PythonExpression(["'", nav_mode, "' == 'slam_nav'"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": slam_nav_params_path,
            "autostart": "true",
        }.items(),
    )

    # 7. Static TF: base_link → laser
    lidar_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0.25", "3.14159", "0", "0", "base_link", "laser"],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="idle",
                description=(
                    "Select robot state: "
                    "'mapping' = SLAM only (build+save map); "
                    "'nav' = Nav2+AMCL with a pre-saved static map; "
                    "'slam_nav' = live SLAM + Nav2 simultaneously "
                    "(unknown area, send goals immediately); "
                    "'idle' = sensors+odometry only"
                ),
            ),
            DeclareLaunchArgument(
                "slam_params_file", default_value=default_slam_config
            ),
            robot_state_publisher,
            lidar_driver,
            lidar_tf_publisher,
            rf2o_node,
            slam_toolbox,
            nav2_stack,
            nav2_navigation_only,
        ]
    )
