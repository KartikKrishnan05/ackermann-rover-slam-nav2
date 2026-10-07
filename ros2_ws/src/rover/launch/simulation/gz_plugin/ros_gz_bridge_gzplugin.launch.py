"""
Launch file for the ROS 2 - Gazebo Bridge.

This script manages the communication layer between ROS 2 and Gazebo
(Ignition) Transport by creating a parameter bridge.

* Defines a list of topics to bridge, specifying the ROS type and the
  Gazebo message type.
* Configures the direction of data flow
    (ROS->Gazebo, Gazebo->ROS, or Bi-directional).
* Optionally publishes a static TF transform between ``map`` and ``odom``
  if SLAM is not active (controlled via ``publish_map_tf``).

Arguments:
    use_sim_time: Sync with simulation clock.
    publish_map_tf: Boolean to enable/disable the static map->odom transform.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for the bridge and static TF.
    """

    # -----------------------------------------------------------------
    # Launch Configurations
    # -----------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    publish_map_tf = LaunchConfiguration("publish_map_tf")

    # -----------------------------------------------------------------
    # Bridge Argument Definition
    # -----------------------------------------------------------------
    # Syntax: "Topic@ROS_Type[Gazebo_Type"
    #   @ = Bidirectional
    #   [ = Gazebo -> ROS
    #   ] = ROS -> Gazebo
    # -----------------------------------------------------------------
    bridge_arguments = [
        # Velocity Command (ROS -> Gazebo)
        "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",

        # Odometry (Gazebo -> ROS)
        "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",

        # TF / Pose (Gazebo -> ROS)
        "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",

        # Clock (Gazebo -> ROS)
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

        # Joint States (Gazebo -> ROS)
        "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",

        # Lidar (Gazebo -> ROS)
        "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",

        # IMU (Gazebo -> ROS)
        "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
    ]

    # -----------------------------------------------------------------
    # Nodes
    # -----------------------------------------------------------------

    # 1. Parameter Bridge Node
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=bridge_arguments,
        output="screen",
    )

    # 2. Static transform from map to odom
    #    Only runs if publish_map_tf is true (i.e., when SLAM is disabled).
    #    Arguments: x y z yaw pitch roll parent_frame child_frame
    map_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"],
        condition=IfCondition(publish_map_tf),
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
            DeclareLaunchArgument(
                "publish_map_tf",
                default_value="true",
                description="Publish static map->odom tf (disable with SLAM)",
            ),
            bridge_node,
            map_static_tf,
        ]
    )
