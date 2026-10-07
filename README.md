# Ackermann Rover — SLAM & Nav2

A small car-like (Ackermann-steered) indoor robot running **ROS 2 Jazzy**, with the same code base driving both a **Gazebo Harmonic simulation** and the **real robot** on an NVIDIA Jetson.

- **Simulation:** URDF/xacro model, `ros2_control` Ackermann steering controller, Gazebo world of an office floor
- **Mapping and navigation:** SLAM Toolbox for building maps, Nav2 for autonomous navigation, `rf2o` laser odometry from a 2D RPLidar
- **Hardware:** keyboard and joystick teleop, `twist_mux` for command arbitration, PWM motor drivers (Adafruit and VoidPP boards)
- **Wi-Fi heatmap:** drive the robot through a building, log signal strength along the way, and plot it on top of the SLAM map
- **One Docker image** for every machine: `build.sh` picks the full desktop image on workstations and a lean ROS base image on the robot

Jump to: [Containerization](#containerization) · [Simulation](#simulation) · [Hardware](#deployment-hardware) · [SLAM](#run-slam) · [Wi-Fi heatmap](#wifi-heatmap) · [Project structure](#project-structure)

## Containerization

**Docker (tested/implemented on a Ubuntu 22.04/24.04 Base)**

Container preconfigured with: 
- ROS 2 Jazzy 
- Gazebo Harmonic
- SLAM Toolbox
- Nav2


```bash
chmod +x build.sh run.sh
```

To build the image run: \
```bash
 ./build.sh 
 ```

To start the container run: \
```bash
 ./run.sh 
 ```

To enter the container run: \
```bash
 docker exec -it silnav2-container bash
 ```

To stop the container: \
```bash
 docker stop silnav2-container
 ```

To remove the container: \
```bash
 docker rm silnav2-container 
 ```

Dockerfile:
- Hybrid Base Image: Dynamically swaps between `osrf/ros:jazzy-desktop-full` (for Simulation/GUIs) and `ros:jazzy-ros-base` (for Hardware) via build arguments.
- Core Stack: Installs essential utilities (`tmux`, `gdb`, `pip`) and the full navigation suite: Nav2, SLAM Toolbox, `ros2_control`, and `ros_gz bridges`.
- User Mapping: Creates `dockeruser` with UID/GID matching the host user to prevent permission conflicts when editing files in the mounted workspace.
- Environment Configuration:
  - `ROS_LOCALHOST_ONLY=1` & `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`: Restricts ROS 2 traffic to the internal machine.
  - `GZ_PARTITION=docker_sim`: Isolates Gazebo discovery so multiple users on the same network don't cross-talk.
  - Auto-Sourcing: Automatically sources the global ROS installation, the local workspace (`/workspace/ros2_ws`), and adds `officesim` to the Gazebo resource path.

Build file (build.sh):
- Host Detection: Automatically identifies the environment via `$HOSTNAME`:
- `rover-robot` (Robot): Selects the lean `ros-base` image.
- `sim-workstation` (Workstation): Selects `desktop-full` and grabs current User IDs.
- `Laptop` (Default): Uses standard ID `1000`.
- Build Args: Dynamically passes the base image selection and user IDs to the Docker engine.
- Tagging: Generates image tagged as `silnav2:jammy-jazzy-harmonic`.

Run file (run.sh):
- Hardware Passthrough: If running on `rover-robot`, it automatically adds:
  - `--runtime nvidia`: Enables GPU acceleration for Orin/Jetson hardware.
  - `--device=/dev/ttyUSB0`: Maps the physical Lidar/Microcontroller serial port.
  - `--group-add dialout`: Grants the container user permissions for serial communication.
- Volume Mounting: * Maps local `ros2_ws` to `/workspace/ros2_ws` (Live code editing).
  - Mounts `/etc/localtime` so logs match the physical robot's time.
- GUI/X11 Support: Maps `.X11-unix` and `.Xauthority` to allow Gazebo and RViz to render on the host screen.
- Performance: Uses `--net=host` and `--ipc=host` for high-performance, zero-latency DDS communication and shared memory access.


## Compiling the Package
Once inside the container shell:
```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## General Configuration

**Twist Multiplexer** Command Priority 
- allows multiple sources (joystick/keyboard or Nav2) to send commands to the robot simultaneously, giving priority to the joystick/keyboard commands 
- Configured here ` rover/config/twist_mux.yaml `
- Multiplexes velocity commands: ` cmd_vel_nav  ` and ` cmd_vel_teleop ` to ` cmd_vel_out ` and ` cmd_vel_mux `
- Twist Stamper: custom helper node that adds valid simulation timestamps to command messaged
- Why? The ackermann_steering_controller is strict. If it receives a command without a timestamp, or with an old timestamp, it might reject it as "stale" or "unsafe."

**SLAM Toolbox**
- configures the Simultaneous Localization and Mapping engine 
- Configured here ` rover/config/mapper_params_online_async.yaml `
- Key Parameters
  1. Connectivity & Frames
    - `odom_frame` / `map_frame` / `base_frame`: Defines the TF tree hierarchy (`map` -> `odom` -> `base_lin`).
    - `scan_topic`: Subscribes to `/scan`. Ensure your Lidar publishes here.
    - `mode: mapping`: Set to `mapping` to build new maps. Change to `localization` to run against an existing `.posegraph` map file.
  2. Map Quality
    - `resolution: 0.05`: The map is built with a 5cm grid size.
    - `max_laser_range: 20.0`: Trust Lidar returns up to 20 meters (matches the simulated sensor).
    - `map_update_interval: 5.0`: The map visual updates every 5 seconds (saves CPU).
  3. Performance & System
    - `transform_publish_period: 0.02`: Publishes the `map->odom` transform at 50Hz for smooth navigation.
    - `stack_size_to_use: 40000000`: Increases the program stack size to handle large map serialization without crashing.
  4. Loop Closure (Drift Correction)
    - `do_loop_closing: true`: The robot will actively look for places it has been before.
    - `loop_search_maximum_distance: 3.0`: If the robot returns to a location within 3 meters of a previous spot, it attempts to "snap" the map together to fix drift.
    - `minimum_travel_distance: 0.5`: The robot must move at least 0.5m before processing a new scan (prevents jitter when stationary).

**NAV2**
- tuned for Ackermann Car, uses SmacHybrid and PurePursuit because it know the robot cannot turn in place compared to differential drive
- Key Parameters
  1. Localization (AMCL)
    - robot_model_type: nav2_amcl::DifferentialMotionModel. Sets the motion model for the particle filter.
    - initial_pose: x: -5.0, y: 2.0. Hardcoded start position matching the Gazebo spawn point.
    - scan_topic: /scan. The topic AMCL listens to for laser data to match against the map.
    - min_particles / max_particles: 500 / 2000. The filter uses up to 2000 particles to estimate position; more particles = more precision but higher CPU usage.
  2. The Planner (Global Path Planning)
    - planner_plugins: GridBased using nav2_smac_planner::SmacPlannerHybrid. A hybrid-A* planner specifically designed for Ackermann (car-like) vehicles.
    - motion_model_for_search: REEDS_SHEPP. Allows the planner to generate complex maneuvers like reversing and 3-point turns.
    - minimum_turning_radius: 0.4. The planner will never generate a path curve tighter than 0.4 meters.
    - smooth_path: true. Post-processes the path to remove jagged edges for smoother steering.
  3. The Controller (Local Path Following)
    - controller_plugins: FollowPath using nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController. A controller that calculates steering curvature based on a lookahead point.
    - desired_linear_vel: 0.5. The standard cruising speed for the robot (m/s).
    - lookahead_dist: 0.6. The robot aims for a point 0.6m ahead on the path to calculate steering.
    - max_angular_accel: 2.0. Limits how fast the wheels turn to prevent physical damage or simulation instability.
  4. Costmaps (Perception)
    - global_costmap: global_frame: map. The static map used for long-term planning. Updates at 1Hz.
    - local_costmap: global_frame: odom. A rolling 3x3m window around the robot used for obstacle avoidance. Updates at 5Hz.
    - footprint: [[0.25, 0.15]...]. Defines the physical rectangular boundary of the robot for collision checking.
    - inflation_radius: 0.55. Adds a buffer zone around obstacles that the robot will try to avoid.
  5. Safety Systems
    - collision_monitor: Defines a FootprintApproach polygon. If an obstacle enters this zone, the robot triggers an immediate "Stop" action.
    - velocity_smoother: Limits acceleration (max_accel: 2.5) to prevent wheel slip or tipping.
    - behavior_server: Manages recovery behaviors (Backup, Wait, Spin) if the robot gets stuck.

## Simulation
**ROS Distribution:** Jazzy Jalisco  
**Simulator:** Gazebo Harmonic (gz-sim)  

**Run Simulation** 
` ros2 launch rover simulation.launch.py [slam:=True] [nav2:=True] `
default values: 
- use_sim_time: True
- slam: False
- nav2: True
- rviz_config: path to ` rviz/my_rviz_config.rviz `

### Simulation Entry Point: 
File: ` launch/simulation.launch.py `

### System Architecture: 
1. Simulation Environment 
- Launches ` gz sim rover/world/world.sdf `
- Sets Paths for ` GZ_SIM_RESOURCE_PATH ` and ` GZ_SIM_SYSTEM_PLUGIN_PATH `
- Spawn rover at ` (x = -0.5m y = -2.0, z = 0.2) ` 
- launch ` robot_state_publisher ` so that ROS knows where the parts are connected to each other 

2. ROS 2 Control 
- Configures Ackermann Steering Controller that translates simple cmd_vel into complex math required to steer a the rover
- Configured here: ` rover/config/ros2_controllers.yaml `
- ` joint_state_broadcaster `: Published the state of all joints to ` /joint_states `
- ` ackermann_steering_controller `: translated drive/Steering commands into wheel moments
- Ackermann Controller Parameters
  1. Kinematics (The Physical Robot): 
    - `wheelbase: 0.312`: Distance between the front and rear axles (meters).
    - `traction_track_width: 0.2`: Distance between the left and right rear wheels.
    - `steering_wheels_radius: 0.05`: Radius of the wheels (5cm). Used to convert "meters per second" into "rotations per minute."
  2. Control Logic:
    - `open_loop: true`: The robot calculates its position (odometry) based on the commands it sent to the wheels, rather than reading encoders. This is common in simulation or for simple robots without wheel encoders.
    - `enable_odom_tf`: false: This controller will NOT publish the odom → base_footprint transform.
    - `use_stamped_vel: false`: The controller expects standard Twist messages.
  3. Joint Mapping:
    - Steering: Maps the concept of "steering" to the specific URDF joints `front_left_steering_joint` and `front_right_steering_joint`.
    - Traction: Maps the concept of "throttle" to `rear_left_wheel_joint` and `rear_right_wheel_joint`.



3. Navigation & Mapping
- If ` slam:= True ` launch SLAM Toolbox
- If ` nav2:= True ` launch Nav2 bringup stack

4. Twist MUX
- launches twist mux node 

5. Logic Gate: should_public_map_tf
- creates a boolean variable and sets it to true if SLAM and Nav2 are disabled
- Why? SLAM and NAV2 create a transform between map -> odom but if both are disabled ` ros_gz_bridge ` needs to publish a fake static transform

6. lidar_tf_fix
- static transform between rover/base_footprint/lidar_2d_v2 and lidar_link for Rviz

7. Bridge Launch
- launches file ` launch/ros_gz_bridge.launch.py `
- gives should_public_map_tf bool variable as parameter


**Robot State Publisher**

launched by simulation.launch.py like this: 
` ros2 launch rover robot_state_publisher.launch.py [use_sim_time:=true] `

1. Xacro Processing: 
- Locates the robot description file: ` rover/urdf/rover_roscontrol.urdf.xacro `
- Starts ` robot_state_publisher ` node
- Feed the generated URDF string into the node's robot_description
- published fulls static TF tree to ` /tf_static ` and the robot description ` /robot_description `

**ROS - GZ Bridge**

launched by simulation.launch.py like this: 
` ros2 launch rover ros_gz_bridge.launch.py [publish_map_tf:=true] `

1. Maps topics: 
- Sync ROS nodes to the simulation time
- `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`
- Dynamic transforms
- `/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V`
- Static transform (lidar position)
- `/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V`
- Position/Velocity of wheel joints
- `/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model`
- 2D Lidar data
- `/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan`
- `/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked`
- Inertial measurement data
- `/imu@sensor_msgs/msg/Imu[gz.msgs.IMU`


### Understanding the syntax: 
- ` @ ` Bidirectional
- ` [ ` Gazebo -> ROS
- ` ] ` ROS -> Gazebo 

2. Conditional Map Frame
- ` IfCondition(publish_map_tf) `
- SLAM is on: SLAM Toolbox publishes the transform from map to odom -> node does nothing
- SLAM is off: no map frame: node launches a static transform publisher that locks map to odom



## Gazebo Simulation World
- File ` world/office_environment.sdf `
- A realistic office layout designed
- Layout: A multi room environment featuring a main corridor, a row of three workspaces and a dedicated corner office setup 
- Static object: tall shelving units, a 3d printer station, a whiteboard presentation board and a plant
- Dynamic Elements: Four blue office chairs (physics-enabled)
 


## Deployment (Hardware)

### Hardware Entry Point: 
`ros2 launch rover hardware.launch.py [use_sim_time:=false] [slam_params_file:=path/to/config]`

This launch script is the physical counterpart to the simulation, responsible for bridging the ROS 2 software stack with the actual rover hardware.

Hardware Architecture:
1. Robot State Publisher
- Loads the physical robot URDF (rover_roscontrol.urdf.xacro).
- Publishes the static transform tree (TF) based on the actual physical dimensions of the chassis.
2. RPLidar A3 Driver
- Initializes the serial connection to the laser scanner.
- Publishes raw distance data to the /scan topic.
3. RF2O Laser Odometry (Virtual Encoder)
- Purpose: Since the hardware may lack high-resolution wheel encoders, - RF2O provides planar odometry by comparing successive laser scans (Scan Matching).
- Function: Estimates the robot's displacement and publishes the crucial odom → base_link transform.
- Configuration: Updates at 10Hz to provide smooth movement data for Nav2.
4. SLAM Toolbox (Hardware Mode)
- Runs the online_async mapping node using real-world data.
- Uses the laser scans and the RF2O-generated odometry to build a persistent map and - handle loop closures.
5. Static Transform (Lidar Fix)
- Manually defines the offset between the center of the robot (base_link) and the optical center of the LiDAR (laser).
- Current offset: X=0.1m,Z=0.2m (Adjustable in the launch file).


### Teleop Overview 

Two possible teleop nodes: teleop_keyboard and teleop_joystick
- both publish messages to cmd_vel (which is subscribed by twist_mux)

How to run teleop: 
Window 1: `ros2 launch rover twist_mux.launch.py`
Window 2: `ros2 launch rover motor_driver.launch.py driver:=[arg]` and arg can be: `adafruit` or `voidpp` (adafruit is default)
Window 3: `ros2 launch rover teleop.launch.py mode:=[arg]` and arg can be: `keyboard` or `joystick` (keyboard is default)

#### teleop_keyboard: 
- uses `termios` & `tty`: puts terminal into raw mode to capture individual keyboard inputs directly without needing to press enter 
- infinite loop to check if and what was pressed
  - w / s: forward and backwards driving: update x linear velocity in twist message
  - a / d: left and right: update z angular velocity in twist message 
- max limit of 1.0
- CTRL + C to send a final command of 0.0 velocity (reset rover to prevent uncontrolled spinning at the end)

#### teleop_joystick:
- Uses struct & /dev/input/js0: Accesses the Linux joystick driver directly to read 8-byte binary packets representing hardware events (axis moves or button presses).
- Timer-based Polling (100Hz): Instead of a blocking loop, it samples the controller state every 10ms to ensure smooth, real-time response without lagging the ROS 2 executor.
- Axis Mapping & Normalization:
  - Axis 1 (Vertical): Maps stick position to x linear velocity.
  - Axis 2 (Horizontal): Maps stick position to z angular velocity.
  - Scaling: Divides raw 16-bit integers ($\pm32767$) (read from the controller angle of joystick) by $32767.0$ to produce a clean float between -1.0 and 1.0.
- Instant Velocity Updates: Unlike the keyboard's "step-by-step" increment, the robot's speed directly mirrors the physical angle of the analog stick.
- Hardware Emergency Stop: if any button is pressed, it immediately resets the Twist message to $0.0$ and publishes it to halt the rover.


Two possible motor_controller which take the output of the twist_mux and converts them into Pulse Width Modulation (PWM) signals: motor_driver_adafruit and motor_driver_voidpp:


#### motor_driver_adafruit: 
- Uses adafruit_servokit & I2C: Communicates over the I2C bus (Address 0x42) to control the PCA9685 PWM driver board
- Translates cmd_vel_mux to PWM
  - Linear X (Throttle): Converts a -1.0 to 1.0 value into a hardware-specific PWM frequency. It uses a deadzone check ($< 0.02$) to prevent motor hum when idle.
  - Non-Linear Mapping: It applies specific "Forward" vs. "Reverse" ranges ($1640 $ - $2030$ for forward, $1430 $ - $1395$ for reverse) to account for motor controller bias.
  - Angular Z (Steering): Directly maps rotation commands to the steering servo channel.
- Signal Normalization: Re-scales the calculated PWM values back to a $-1.0$ to $1.0$ float required by the continuous_servo library using a center-point of $1530.0$.
- Hardware Safety Halt: Includes a stop_motors() function that forces all PWM outputs to neutral ($0.0$ throttle) if the ROS 2 node is shut down or interrupted.

#### PWM Transformation Logic
The transformation follows this mathematical flow:
1. Input: v (linear velocity from Twist)
2. Raw PWM:
   1. If $v > 0$ : $PWM = 1640 + (v \cdot 390)$
   2. If $v < 0$ : $PWM = 1430 + (|v| \cdot -35)$
3. Library Output: $Throttle = \frac{PWM - 1530}{500}$

#### motor_driver_voidpp: 
- uses pca9685_driver (https://github.com/voidpp/PCA9685-driver): A direct hardware interface for the PCA9685 chip
- Microsecond-to-Tick Conversion:
  -  Since the PCA9685 is a 12-bit controller, it divides one PWM period into 4,096 steps (ticks).
  -  The $\mu s$_to_ticks function calculates: $\text{ticks} = \left( \frac{\text{microseconds}}{20,000} \right) \times 4096$
- Precision Control Logic: 
  - Linear X (Throttle): Maps values to a calibrated range. If $v > 0$, it scales between $1640\mu s$ and $2030\mu s$. If $v < 0$, it scales between $1430\mu s$ and $1395\mu s$.
  - Angular Z (Steering): Centers the wheels at $1900\mu s$ and allows a swing of $\pm 500\mu s$ based on the steering input.
- Deadzone Handling: Ignores any throttle inputs smaller than 0.02 to protect the motors from high-frequency jitter or stalling.
- Safe Shutdown: Upon node termination, stop_motors() is triggered to return both the throttle and steering to their respective neutral/center positions ($1530\mu s$ and $1900\mu s$).


#### Twist_stamper for Simulation
- Translates geometry_msgs/Twist into geometry_msgs/Twist_Stamped
  - Adds following metadata: 
    - stamp: contains the ROS 2 system time at the exact moment the message was created
    - frame_id: identifies the coordinate frame to which the velocity applies
- Gets the cmd_vel_mux from the twist_mux and sends the new message to ros_control



### Run SLAM: 
Command: `ros2 launch rover hardware.launch.py mode:=nav`. Mode can be `idle|nav|mapping`. Default is `idle`.
- IMPORTANT: (??? ZH: not sure)  This launches `twist_mux.launch.py`, no need to rerun it when starting teleop

- launches script which initializes the rover's physical hardware and core navigation stack. It handles everything from reading the 3D model to starting the laser sensors and generating movement data. 

- Core Components: 
  - Robot State Publisher: Processes the Xacro/URDF file to publish the robot's physical structure
  - RPLidar A3 Driver: The hardware that tells the physical laser scanner to start spinning and emitting data 
  - RF2O Laser Odometry: Estimates how much the robot has moved by looking at changes in the LiDAR scans 
  - SLAM Toolbox: the mapping engine. It take LiDAR and Odometry data to build a 2D map and locate the robot within it
  - Static Transform: A fixed offset that tells ROS exactly where the LiDAR is mounted on the robot 


Why use `rf20_laser_odometry`: 
- normally robots use wheel encoders to calculate odometry to see how far they actually have traveled. We don't have any encoders meaning if it slips on the floor or is stuck the odometry will be broken
  - The virtual Encoder: rf20 performs PLanar Laser Scan Matching. It compares a scan at a Time A to a scan at Time B, By seeing how the "walls" have moved between the scans it calculates exactly how the robot must have moves
  - provides a very stable `odom -> base_link` transform without needing any physical sensors on the wheels


Data Flow: 
`Physical World -> rplidar_ros -> /scan -> rf20_laser_odometry -> /odom`
`/scan + /odom -> SLAM Toolbox -> Live 2D Map`


## Wifi HeatMap

Inside of the Docker container run following code: 
```sudo setcap -r /usr/bin/python3.12```
```sudo setcap cap_net_admin+ep /usr/sbin/iwconfig```
```sudo setcap cap_net_admin+ep /usr/sbin/iw```


To start measuring the values: ``` python3 wifi_signal_logger_ros2.py --interface wlan0 --interval 1.0 --output wifi_signal_log.csv ```

! Check wifi name: ``` iwconfig 2>&1 ```

To generate the map use following command: 
```bash python3 wifi_heatmap.py wifi_signal_log.csv \
    --map /workspace/ros2_ws/install/rover/share/rover/maps/3rd_floor_run1.pgm \
    --yaml /workspace/ros2_ws/install/rover/share/rover/maps/3rd_floor_run1.yaml \
    --radius 0.5 \
    --save wifi_heatmap.png 
```
To view the image: 
Copy from docker container into jetson:
``` docker cp <container_name>:/workspace/ros2_ws/src/rover/rover/wifi_mapper/wifi_heatmap.png ~/wifi_heatmap.png ```


To copy from jetson onto your device: 
``` scp <user>@<jetson-ip>:~/wifi_heatmap.png ~/wifi_heatmap.png ```


Open image: ``` xdg-open ~/wifi_heatmap.png ```


## Project Structure

```text
.
├── Dockerfile
├── README.md
├── build.sh
├── ros2_ws
│   └── src
│       ├── rf2o_laser_odometry
│       ├── rover
│       │   ├── config
│       │   │   ├── accerPlanner.yaml
│       │   │   ├── mapper_params_online_async.yaml
│       │   │   ├── ros2_controllers.yaml
│       │   │   └── twist_mux.yaml
│       │   ├── launch
│       │   │   ├── hardware
│       │   │   │   └── hardware.launch.py
│       │   │   |   ├── twist_mux.launch.py
│       │   │   |   ├── teleop.launch.py
│       │   │   |   └── motor_driver.launch.py
│       │   │   └── simulation
│       │   │       ├── gz_plugin
│       │   │       │   ├── robot_state_publisher_gzplugin.launch.py
│       │   │       │   ├── ros_gz_bridge_gzplugin.launch.py
│       │   │       │   └── simulation_gzplugin.launch.py
│       │   │       ├── navigation2.launch.py
│       │   │       ├── robot_state_publisher.launch.py
│       │   │       ├── ros_gz_bridge.launch.py
│       │   │       └── simulation.launch.py
│       │   ├── maps
│       │   │   ├── empty_world.pgm
│       │   │   ├── empty_world.yaml
│       │   │   ├── my_office_map291225.pgm
│       │   │   ├── my_office_map291225.yaml
│       │   │   ├── office_map.pgm
│       │   │   └── office_map.yaml
│       │   ├── meshes
│       │   │   ├── MainBody.dae
│       │   │   ├── Wheel.dae
│       │   │   └── lidar_2d_v2.dae
│       │   ├── package.xml
│       │   ├── resource
│       │   │   └── rover
│       │   ├── rover
│       │   │   ├── __init__.py
│       │   │   ├── hardware
│       │   │   │   ├── teleop_joystick.py
│       │   │   │   ├── teleop_joystick_voidpp.py
│       │   │   │   └── teleop_keyboard.py
│       │   │   ├── teleop.py
│       │   │   ├── teleop_roscontrol.py
│       │   │   └── twist_stamper.py
│       │   ├── rviz
│       │   │   └── my_rviz_config.rviz
│       │   ├── setup.cfg
│       │   ├── setup.py
│       │   ├── test
│       │   │   ├── test_copyright.py
│       │   │   ├── test_flake8.py
│       │   │   └── test_pep257.py
│       │   ├── urdf
│       │   │   ├── gazebo_plugins.xacro
│       │   │   ├── lidar.xacro
│       │   │   ├── ros2_control.xacro
│       │   │   ├── rover_core.xacro
│       │   │   ├── rover_gzplugin.urdf.xacro
│       │   │   └── rover_roscontrol.urdf.xacro
│       │   └── world
│       │       └── world.sdf
│       └── rplidar_ros
└── run.sh
```

