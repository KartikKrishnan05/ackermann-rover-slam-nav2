"""
Launch file for the ROS 2 ↔ Gazebo (Ignition) Bridge.

This launch script creates a **parameter_bridge** node that forwards
selected topics between ROS 2 and Gazebo Transport.
In addition, it can (optionally) publish a static transform
*map → odom* - a transform that is required when the SLAM stack is
disabled and the map frame is generated elsewhere.

Key features
------------
* **Bidirectional, ROS-to-Gazebo and Gazebo-to-ROS topic mapping** - defined
  in the ``bridge_arguments`` list.
* **Simulation time handling** - all nodes can be instructed to use the
  Gazebo clock via the ``use_sim_time`` launch argument.
* **Conditional static TF** - the static *map-to-odom* transform is only
  started when the ``publish_map_tf`` argument is true (typically disabled
  when SLAM publishes its own transform).

All launch-time parameters can be overridden from the command line, e.g.:
ros2 launch rover ros_gz_bridge.launch.py
- use_sim_time:=true
- publish_map_tf:=false
"""
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Build and return a ``LaunchDescription`` containing:
    1. The ``parameter_bridge`` node with all topic mappings.
    2. A conditional static transform publisher (map → odom).
    3. Declaration of the launch arguments used by the above nodes.
    """

    # -----------------------------------------------------------------
    # Launch arguments (with default values)
    # -----------------------------------------------------------------
    # use_sim_time  - tells nodes to use the simulation clock published by
    #                 Gazebo instead of the system clock.
    # publish_map_tf - controls whether the static map-to-odom TF is launched.
    # -----------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    publish_map_tf = LaunchConfiguration("publish_map_tf", default="true")

    # -----------------------------------------------------------------
    # Definition of the topic bridges.
    # Syntax for each entry:  "<topic>@<ROS_msg_type>[<Gazebo_msg_type>"
    #   '['  → direction Gazebo → ROS
    #   ']'  → direction ROS → Gazebo
    #   '@'  → bidirectional (both directions)
    #
    # The list below covers the most common sensors and bookkeeping
    # topics required for a mobile robot simulation.
    # -----------------------------------------------------------------
    bridge_arguments = [
        # ---- TF / Pose (Gazebo → ROS) ---------------------------------
        "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        # ---- Simulation clock (Gazebo → ROS) -------------------------
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        # ---- Joint states (Gazebo → ROS) ------------------------------
        "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        # ---- 2-D Laser scanner (Gazebo → ROS) -------------------------
        "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        # ---- IMU (Gazebo → ROS) ----------------------------------------
        "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        # (Add more bridges here if additional sensors or command topics
        #  are needed, e.g. velocity commands, odometry, etc.)
    ]

    # -----------------------------------------------------------------
    # Node: ros_gz_bridge → parameter_bridge
    # -----------------------------------------------------------------
    # * package   - the ROS 2 package that provides the bridge executable.
    # * executable - the binary that implements the generic parameter bridge.
    # * parameters - we only pass the ``use_sim_time`` flag so the node
    #                subscribes to the /clock topic when required.
    # * arguments  - the list defined above; each element becomes a
    #                command-line argument for the bridge.
    # * output    - print logs to the console (screen) for easier debugging.
    # -----------------------------------------------------------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=bridge_arguments,
        output="screen",
    )

    # -----------------------------------------------------------------
    # Node: static_transform_publisher (map → odom)
    # -----------------------------------------------------------------
    # This node publishes a **fixed** transform between the ``map`` and
    # ``odom`` frames.  All translation and rotation components are set to
    # zero, i.e. the frames are coincident.  The node is only started when
    # the launch argument ``publish_map_tf`` evaluates to true - this is
    # the behaviour expected when SLAM is *not* running (the map frame is
    # otherwise provided by the SLAM toolbox).
    # -----------------------------------------------------------------
    map_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "0.0",   # x-translation
            "0.0",   # y-translation
            "0.0",   # z-translation
            "0.0",   # roll
            "0.0",   # pitch
            "0.0",   # yaw
            "map",   # parent frame
            "odom",  # child frame
        ],
        condition=IfCondition(publish_map_tf),  # launch only if true
    )

    # -----------------------------------------------------------------
    # Assemble the full launch description
    # -----------------------------------------------------------------
    return LaunchDescription(
        [
            # ----- Declare launch arguments (users can override them) -----
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="If true, nodes will use simulation time.",
            ),
            DeclareLaunchArgument(
                "publish_map_tf",
                default_value="true",
                description="Publish static map->odom TF.",
            ),
            # ----- Core nodes ------------------------------------------------
            bridge,          # ROS ↔ Gazebo bridge
            map_static_tf,   # Optional static transform
        ]
    )
