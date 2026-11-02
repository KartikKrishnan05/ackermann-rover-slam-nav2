# Ackermann Rover Project

An ROS 2 stack for an Ackermann-steered (car-like) rover, supporting both Gazebo
simulation and real hardware: SLAM mapping, Nav2 navigation, teleoperation,
autonomous frontier exploration, and WiFi heatmapping.

**ROS Distribution:** Jazzy Jalisco &nbsp;•&nbsp; **Simulator:** Gazebo Harmonic (gz-sim)

---

## Table of Contents

- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Clone the Repository](#clone-the-repository)
- [Containerization](#containerization)
- [Compiling the Package](#compiling-the-package)
- [General Configuration](#general-configuration)
  - [Twist Multiplexer](#twist-multiplexer)
  - [SLAM Toolbox](#slam-toolbox)
  - [Nav2](#nav2)
- [Simulation](#simulation)
  - [System Architecture](#system-architecture)
  - [Robot State Publisher](#robot-state-publisher)
  - [ROS ↔ GZ Bridge](#ros--gz-bridge)
  - [Gazebo Simulation World](#gazebo-simulation-world)
- [Deployment (Hardware)](#deployment-hardware)
  - [Hardware Bill of Materials](#hardware-bill-of-materials)
  - [Connecting to the Robot](#connecting-to-the-robot)
  - [Hardware Architecture](#hardware-architecture)
  - [Teleoperation](#teleoperation)
  - [Motor Driver](#motor-driver)
  - [Running SLAM on Hardware](#running-slam-on-hardware)
  - [Saving a Map](#saving-a-map)
- [Autonomous Exploration (Frontier Explorer)](#autonomous-exploration-frontier-explorer)
- [WiFi Heatmap](#wifi-heatmap)
- [Project Structure](#project-structure)

---

## Getting Started

New to this project? Start here. The entire stack runs inside Docker, so you do
**not** install ROS 2, Gazebo, or Nav2 on the host — only the prerequisites below.

### Prerequisites

| Requirement | Notes |
| --- | --- |
| **OS** | Linux (Ubuntu 22.04 / 24.04 tested). The GUI tooling (Gazebo, RViz) relies on X11; macOS/Windows are not supported out of the box. |
| **Docker** | [Install Docker Engine](https://docs.docker.com/engine/install/). Add your user to the `docker` group (`sudo usermod -aG docker $USER`, then re-login) so you can run it without `sudo`. |
| **Git** | To clone the repo. |
| **NVIDIA Container Toolkit** | **Only** needed where `run.sh` uses `--runtime nvidia` / `--gpus all` — i.e. the robot (Jetson) and any GPU workstation. [Install guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). A plain laptop running simulation does not need it (the hardware flags are skipped off `rover-robot`). |
| **X11 display access** | Gazebo and RViz render on the host display. Before starting the container, grant the container access to your X server:<br>`xhost +local:docker`<br>Without this, GUIs fail with an opaque `cannot open display` error. |

### Clone the Repository

```bash
git clone https://github.com/KartikKrishnan05/ackermann-rover-slam-nav2.git
cd ackermann-rover-slam-nav2
```

The workspace bundles two vendored upstream packages directly under
`ros2_ws/src/` — [`rplidar_ros`](ros2_ws/src/rplidar_ros) (LiDAR driver) and
[`rf2o_laser_odometry`](ros2_ws/src/rf2o_laser_odometry) (laser odometry). They
are committed in-tree, so a plain `git clone` is all you need — no submodule
init step.

The directory you clone into is bind-mounted into the container at
`/workspace/ros2_ws`, so any code edits on the host are live inside the container
(see [Run Script](#run-script-runsh)). Run `build.sh` / `run.sh` from the repo
root.

> **Build vs. runtime dependencies:** all ROS 2 dependencies (Nav2, SLAM
> Toolbox, `ros_gz`, `ros2_control`, …) are installed by the
> [Dockerfile](#dockerfile), **not** declared in `package.xml`. This means
> `rosdep install` will *not* pull them — always build and run inside the
> container.

---

## Containerization

**Docker** (tested/implemented on an Ubuntu 22.04 / 24.04 base).

The container comes preconfigured with:

| Component | Purpose |
| --- | --- |
| ROS 2 Jazzy | Core middleware |
| Gazebo Harmonic | Physics simulation |
| SLAM Toolbox | Mapping & localization |
| Nav2 | Navigation stack |
| `numpy` & `scipy` | Used by the autonomous frontier explorer |

### Usage

```bash
# One-time: make the helper scripts executable
chmod +x build.sh run.sh

./build.sh                              # Build the image
./run.sh                               # Start the container
docker exec -it silnav2-container bash  # Enter the container
docker stop silnav2-container          # Stop the container
docker rm silnav2-container            # Remove the container
```

### Dockerfile

- **Hybrid Base Image:** Dynamically swaps between `osrf/ros:jazzy-desktop-full` (Simulation/GUIs) and `ros:jazzy-ros-base` (Hardware) via build arguments.
- **Core Stack:** Installs essential utilities (`tmux`, `gdb`, `pip`) and the full navigation suite: Nav2, SLAM Toolbox, `ros2_control`, and `ros_gz` bridges.
- **User Mapping:** Creates `dockeruser` with UID/GID matching the host user to prevent permission conflicts when editing files in the mounted workspace.
- **Environment Configuration:**
  - `ROS_LOCALHOST_ONLY=1` & `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` — restricts ROS 2 traffic to the internal machine.
  - `GZ_PARTITION=docker_sim` — isolates Gazebo discovery so multiple users on the same network don't cross-talk.
  - **Auto-Sourcing** — sources the global ROS installation, the local workspace (`/workspace/ros2_ws`), and adds `officesim` to the Gazebo resource path.

### Build Script (`build.sh`)

- **Host Detection** via `$HOSTNAME`:
  - `rover-robot` (Robot) — selects the lean `ros-base` image.
  - `sim-workstation` (Workstation) — selects `desktop-full` and grabs current user IDs.
  - Laptop (Default) — uses standard ID `1000`.
- **Build Args** — dynamically passes the base-image selection and user IDs to the Docker engine.
- **Tagging** — generates an image tagged `silnav2:jammy-jazzy-harmonic`.

### Run Script (`run.sh`)

- **Hardware Passthrough** (when running on `rover-robot`):
  - `--runtime nvidia` — enables GPU acceleration for Orin/Jetson hardware.
  - `--device=/dev/ttyUSB0` — maps the physical Lidar/microcontroller serial port.
  - `--group-add dialout` — grants the container user serial-communication permissions.
- **Volume Mounting** — maps local `ros2_ws` to `/workspace/ros2_ws` (live code editing) and `/etc/localtime` so logs match the physical robot's time.
- **GUI / X11 Support** — maps `.X11-unix` and `.Xauthority` so Gazebo and RViz can render on the host screen.
- **Performance** — uses `--net=host` and `--ipc=host` for high-performance, zero-latency DDS communication and shared-memory access.

---

## Compiling the Package

Once inside the container shell:

```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## General Configuration

### Twist Multiplexer

Allows multiple sources (joystick/keyboard or Nav2) to send commands to the robot
simultaneously, giving **priority to the joystick/keyboard** commands.

- **Config:** `rover/config/twist_mux.yaml`
- **Multiplexes:** `cmd_vel_nav` and `cmd_vel_teleop` → `cmd_vel_out` / `cmd_vel_mux`
- **Twist Stamper:** a custom helper node that adds valid simulation timestamps to command messages.
  - *Why?* The `ackermann_steering_controller` is strict — a command without a timestamp, or with an old one, may be rejected as "stale" or "unsafe."

### SLAM Toolbox

Configures the Simultaneous Localization and Mapping engine.

- **Config:** `rover/config/mapper_params_online_async.yaml`

**Connectivity & Frames**

| Parameter | Description |
| --- | --- |
| `odom_frame` / `map_frame` / `base_frame` | Defines the TF tree hierarchy (`map` → `odom` → `base_link`). |
| `scan_topic` | Subscribes to `/scan`. Ensure your Lidar publishes here. |
| `mode: mapping` | `mapping` builds new maps; change to `localization` to run against an existing `.posegraph` map file. |

**Map Quality**

| Parameter | Description |
| --- | --- |
| `resolution: 0.05` | Map built with a 5 cm grid size. |
| `max_laser_range: 20.0` | Trust Lidar returns up to 20 m (matches the simulated sensor). |
| `map_update_interval: 5.0` | Map visual updates every 5 s (saves CPU). |

**Performance & System**

| Parameter | Description |
| --- | --- |
| `transform_publish_period: 0.02` | Publishes the `map→odom` transform at 50 Hz for smooth navigation. |
| `stack_size_to_use: 40000000` | Increases the program stack size to handle large map serialization without crashing. |

**Loop Closure (Drift Correction)**

| Parameter | Description |
| --- | --- |
| `do_loop_closing: true` | The robot actively looks for places it has been before. |
| `loop_search_maximum_distance: 3.0` | Within 3 m of a previous spot, it attempts to "snap" the map together to fix drift. |
| `minimum_travel_distance: 0.5` | Must move at least 0.5 m before processing a new scan (prevents jitter when stationary). |

### Nav2

Tuned for an Ackermann car — uses **SmacHybrid** and **Pure Pursuit** because the
robot cannot turn in place like a differential-drive robot.

**Localization (AMCL)**

| Parameter | Description |
| --- | --- |
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` — motion model for the particle filter. |
| `initial_pose` | `x: -5.0, y: 2.0` — hardcoded start position matching the Gazebo spawn point. |
| `scan_topic` | `/scan` — laser data AMCL matches against the map. |
| `min_particles` / `max_particles` | `500` / `2000` — more particles = more precision but higher CPU usage. |

**Global Planner**

| Parameter | Description |
| --- | --- |
| `planner_plugins` | `GridBased` using `nav2_smac_planner::SmacPlannerHybrid` — hybrid-A* planner for car-like vehicles. |
| `motion_model_for_search` | `REEDS_SHEPP` — allows complex maneuvers like reversing and 3-point turns. |
| `minimum_turning_radius` | `0.4` — never generates a path curve tighter than 0.4 m. |
| `smooth_path` | `true` — post-processes the path to remove jagged edges. |

**Local Controller**

| Parameter | Description |
| --- | --- |
| `controller_plugins` | `FollowPath` using `RegulatedPurePursuitController` — steering curvature from a lookahead point. |
| `desired_linear_vel` | `0.5` — standard cruising speed (m/s). |
| `lookahead_dist` | `0.6` — aims for a point 0.6 m ahead on the path. |
| `max_angular_accel` | `2.0` — limits how fast the wheels turn to prevent damage/instability. |

**Costmaps (Perception)**

| Parameter | Description |
| --- | --- |
| `global_costmap` | `global_frame: map` — static map for long-term planning. Updates at 1 Hz. |
| `local_costmap` | `global_frame: odom` — rolling 3×3 m window for obstacle avoidance. Updates at 5 Hz. |
| `footprint` | `[[0.25, 0.15]...]` — physical rectangular boundary for collision checking. |
| `inflation_radius` | `0.55` — buffer zone around obstacles the robot tries to avoid. |

**Safety Systems**

| System | Description |
| --- | --- |
| `collision_monitor` | A `FootprintApproach` polygon — triggers an immediate "Stop" if an obstacle enters the zone. |
| `velocity_smoother` | Limits acceleration (`max_accel: 2.5`) to prevent wheel slip or tipping. |
| `behavior_server` | Manages recovery behaviors (Backup, Wait, Spin) if the robot gets stuck. |

---

## Simulation

```bash
ros2 launch rover simulation.launch.py [slam:=True] [nav2:=True]
```

**Entry point:** `launch/simulation/simulation.launch.py`

| Argument | Default |
| --- | --- |
| `use_sim_time` | `True` |
| `slam` | `False` |
| `nav2` | `True` |
| `rviz_config` | `rviz/my_rviz_config.rviz` |

### System Architecture

1. **Simulation Environment**
   - Launches `gz sim rover/world/world.sdf`.
   - Sets `GZ_SIM_RESOURCE_PATH` and `GZ_SIM_SYSTEM_PLUGIN_PATH`.
   - Spawns the rover at `(x = -0.5, y = -2.0, z = 0.2)`.
   - Launches `robot_state_publisher` so ROS knows how the parts connect.
2. **ROS 2 Control** — see [Ackermann Controller](#ackermann-controller-parameters) below.
3. **Navigation & Mapping**
   - If `slam:=True`, launches SLAM Toolbox.
   - If `nav2:=True`, launches the Nav2 bringup stack.
4. **Twist MUX** — launches the twist-mux node.
5. **Logic Gate (`should_publish_map_tf`)**
   - Set to `true` only when both SLAM and Nav2 are disabled.
   - *Why?* SLAM and Nav2 create the `map→odom` transform; if both are off, `ros_gz_bridge` must publish a fake static transform instead.
6. **`lidar_tf_fix`** — static transform between `rover/base_footprint/lidar_2d_v2` and `lidar_link` for RViz.
7. **Bridge Launch** — launches `launch/simulation/ros_gz_bridge.launch.py`, passing the `should_publish_map_tf` bool.

#### Ackermann Controller Parameters

Configures the Ackermann Steering Controller that translates a simple `cmd_vel`
into the math required to steer the rover.

- **Config:** `rover/config/ros2_controllers.yaml`
- `joint_state_broadcaster` — publishes the state of all joints to `/joint_states`.
- `ackermann_steering_controller` — translates drive/steering commands into wheel motion.

**Kinematics (Physical Robot)**

| Parameter | Description |
| --- | --- |
| `wheelbase: 0.312` | Distance between the front and rear axles (m). |
| `traction_track_width: 0.2` | Distance between the left and right rear wheels. |
| `steering_wheels_radius: 0.05` | Wheel radius (5 cm). Converts m/s into RPM. |

**Control Logic**

| Parameter | Description |
| --- | --- |
| `open_loop: true` | Computes odometry from issued commands rather than encoders (common in sim / encoder-less robots). |
| `enable_odom_tf: false` | This controller does **not** publish the `odom → base_footprint` transform. |
| `use_stamped_vel: false` | Expects standard `Twist` messages. |

**Joint Mapping**

| Concept | URDF Joints |
| --- | --- |
| Steering | `front_left_steering_joint`, `front_right_steering_joint` |
| Traction | `rear_left_wheel_joint`, `rear_right_wheel_joint` |

### Robot State Publisher

```bash
ros2 launch rover robot_state_publisher.launch.py [use_sim_time:=true]
```

- Locates the robot description: `rover/urdf/rover_roscontrol.urdf.xacro`.
- Starts the `robot_state_publisher` node and feeds it the generated URDF string.
- Publishes the full static TF tree to `/tf_static` and the description to `/robot_description`.

### ROS ↔ GZ Bridge

```bash
ros2 launch rover ros_gz_bridge.launch.py [publish_map_tf:=true]
```

**Mapped topics**

| Purpose | Topic & Type |
| --- | --- |
| Sync ROS nodes to simulation time | `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock` |
| Dynamic transforms | `/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V` |
| Static transform (lidar position) | `/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V` |
| Wheel joint position/velocity | `/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model` |
| 2D Lidar data | `/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan` |
| Lidar point cloud | `/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked` |
| Inertial measurement data | `/imu@sensor_msgs/msg/Imu[gz.msgs.IMU` |

**Bridge syntax**

| Symbol | Meaning |
| --- | --- |
| `@` | Bidirectional |
| `[` | Gazebo → ROS |
| `]` | ROS → Gazebo |

**Conditional Map Frame** — `IfCondition(publish_map_tf)`:

- SLAM **on** → SLAM Toolbox publishes `map→odom`, so this node does nothing.
- SLAM **off** → no map frame, so the node launches a static transform publisher that locks `map` to `odom`.

### Gazebo Simulation World

- **File:** `world/world.sdf`
- A realistic office layout: a multi-room environment featuring a main corridor, a row of three workspaces, and a dedicated corner-office setup.
- **Static objects:** tall shelving units, a 3D printer station, a whiteboard/presentation board, and a plant.
- **Dynamic elements:** four blue office chairs (physics-enabled).

### Alternative: Gazebo-Plugin Simulation (`gz_plugin/`)

A second, **experimental** simulation path lives under
[`launch/simulation/gz_plugin/`](ros2_ws/src/rover/launch/simulation/gz_plugin).
It mirrors the main flow (Gazebo + spawn + bridge + RViz + optional SLAM) but is
wired for a Gazebo-plugin-based control workflow rather than the default
`ros2_control` + `ros_gz_bridge` path, and it auto-launches RViz and a teleop
pop-up itself.

```bash
ros2 launch rover simulation_gzplugin.launch.py [slam:=True] [use_custom_teleop:=True]
```

> ⚠️ **Known packaging issue:** the main flow described above is the supported
> one. [`setup.py`](ros2_ws/src/rover/setup.py) installs these files from a
> `launch/gz_plugin/*` glob, but they actually live in
> `launch/simulation/gz_plugin/`, so they currently install to a different
> location than the launch file's internal `launch/gz_plugin` lookup expects.
> Treat this variant as legacy/experimental and verify install paths before
> relying on it.

---

## Deployment (Hardware)

```bash
ros2 launch rover hardware.launch.py [use_sim_time:=false] [mode:=arg] [slam_params_file:=path/to/config]
```

The physical counterpart to the simulation — it bridges the ROS 2 software stack
with the actual rover hardware.

**Modes** (`mode:=arg`, default `idle`):

| Mode | Description |
| --- | --- |
| `idle` | Sensors + odometry only (Robot State Publisher, RPLidar, RF2O). No mapping or navigation. |
| `mapping` | SLAM Toolbox only — drive the rover (teleop) and build a map to save. |
| `nav` | Nav2 + AMCL localizing against a pre-saved static map (`maps/3rdfloor_office.yaml`). Uses `config/accerPlanner_hardware.yaml`. |
| `slam_nav` | Live SLAM **and** Nav2 simultaneously (no AMCL, no static map server). SLAM Toolbox owns `/map` and the `map→odom` TF while Nav2 plans into unknown space. Uses `config/nav2_slam_hardware.yaml`. Used with the autonomous [Frontier Explorer](#autonomous-exploration-frontier-explorer). |

### Hardware Bill of Materials

The physical rover is an Ackermann-steered (car-like) RC platform. The table
below is what the software expects to find — use it to identify components and
their connections when picking up the hardware cold.

| Component | Role | Connection / Interface |
| --- | --- | --- |
| **Compute (Jetson, `rover-robot`)** | NVIDIA Jetson SBC running the container with GPU passthrough (`--runtime nvidia`). | — |
| **RPLidar A3** | 2D laser scanner → `/scan`. | USB serial, `/dev/ttyUSB0` (needs `dialout` group). Driver: [`rplidar_ros`](ros2_ws/src/rplidar_ros) via `rplidar_a3_launch.py`. |
| **PCA9685 PWM board** | 12-bit servo/ESC driver that converts `cmd_vel` into PWM. | I²C bus `1` (`/dev/i2c-1`), address `0x42` (non-default due to address-jumper soldering). |
| **ESC + drive motor** | Forward/reverse traction. | PCA9685 **channel 0** (throttle). |
| **Steering servo** | Front-wheel steering. | PCA9685 **channel 1** (steering). |
| **Gamepad / joystick** | Optional manual teleop. | Linux joystick `/dev/input/js0` (needs `input` group). |

**PWM calibration** lives in
[`config/motor_driver_config.yaml`](ros2_ws/src/rover/config/motor_driver_config.yaml)
and is tuned per vehicle (neutral, forward/reverse start & max, steering center
in microseconds). If the rover creeps when idle, drives the wrong direction, or
won't track straight, **re-tune these values first** — they are deliberately
offset from the textbook 1500 µs center for this specific ESC/servo build.

Device access is wired up automatically by [`run.sh`](run.sh) when it detects
host `rover-robot`: it `chmod`s `/dev/i2c-1` and `/dev/gpiochip*`, passes through
`/dev/ttyUSB0`, `/dev/input/js0`, and the I²C/GPIO devices, and adds the
`dialout` / `input` groups. If a device is missing at boot the script prints a
warning rather than failing.

### Connecting to the Robot

The build/run scripts branch on `$HOSTNAME`, so the same repo behaves
differently depending on where it runs (see [Build Script](#build-script-buildsh)
and [Run Script](#run-script-runsh)):

| Host | Role | What the scripts select |
| --- | --- | --- |
| `rover-robot` | **The robot** (Jetson). | Lean `ros:jazzy-ros-base` image, full hardware passthrough (NVIDIA runtime, serial, I²C, GPIO, joystick). |
| `sim-workstation` | GPU **workstation** for simulation. | `osrf/ros:jazzy-desktop-full`, host user IDs. |
| anything else | Generic **laptop**. | `desktop-full`, default UID/GID `1000`, no hardware flags. |

To work on the physical robot you build and run **on the Jetson itself**, not
remotely:

```bash
# 1. SSH into the robot (find its IP on your network)
ssh <user>@<jetson-ip>

# 2. On the Jetson: get the code and build/run the container
cd ~/ackermann-rover-slam-nav2  # wherever the repo lives on the robot
./build.sh                 # auto-selects the ros-base hardware image
./run.sh                   # auto-adds --runtime nvidia + device passthrough
docker exec -it silnav2-container bash

# 3. Inside the container: compile and launch (see below)
cd /workspace/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch rover hardware.launch.py mode:=mapping
```

> **Tip:** because the hostname drives the configuration, renaming the robot
> away from `rover-robot` (or a workstation away from `sim-workstation`) will silently
> change which image and device flags are used. Adjust `build.sh`/`run.sh` if
> your host names differ.

### Hardware Architecture

1. **Robot State Publisher** — loads the physical robot URDF (`rover_roscontrol.urdf.xacro`) and publishes the static TF tree from the chassis dimensions.
2. **RPLidar A3 Driver** — initializes the serial connection to the laser scanner and publishes raw distances to `/scan`.
3. **RF2O Laser Odometry (Virtual Encoder)**
   - *Purpose:* the hardware may lack high-resolution wheel encoders, so RF2O provides planar odometry by comparing successive laser scans (scan matching).
   - *Function:* estimates displacement and publishes the crucial `odom → base_link` transform.
   - *Configuration:* updates at 10 Hz for smooth movement data into Nav2.
4. **SLAM Toolbox (Hardware Mode)** — runs the `online_async` mapping node on real data; uses laser scans + RF2O odometry to build a persistent map and handle loop closures.
5. **Static Transform (Lidar Fix)** — defines the offset between the robot center (`base_link`) and the LiDAR optical center (`laser`). Current offset `X=0.1, Z=0.2` (adjustable in the launch file).

### Teleoperation

Two teleop nodes — `teleop_keyboard` and `teleop_joystick` — both publish to
`cmd_vel` (subscribed by `twist_mux`). Run each command in its own window:

```bash
# Window 1 — command multiplexer
ros2 launch rover twist_mux.launch.py

# Window 2 — motor driver (motor_driver_voidpp; calibration from config/motor_driver_config.yaml)
ros2 launch rover motor_driver.launch.py

# Window 3 — teleop input (mode: keyboard [default] or joystick; limits & device from config/teleop_config.yaml)
ros2 launch rover teleop.launch.py mode:=keyboard
```

**`teleop_keyboard`**

- Uses `termios` & `tty` to put the terminal into raw mode and capture key presses directly (no Enter needed).
- Infinite loop checking which key was pressed:
  - `w` / `s` — forward / backward (updates linear-x velocity).
  - `a` / `d` — left / right (updates angular-z velocity).
- Max limit of 1.0.
- `Ctrl + C` sends a final 0.0 velocity command (resets the rover to prevent uncontrolled spinning).

**`teleop_joystick`**

- Uses `struct` & `/dev/input/js0` to read 8-byte binary packets from the Linux joystick driver (axis moves / button presses).
- **Timer-based polling (100 Hz)** — samples controller state every 10 ms for smooth, real-time response without lagging the ROS 2 executor.
- **Axis mapping & normalization:**
  - Axis 1 (vertical) → linear-x velocity.
  - Axis 2 (horizontal) → angular-z velocity.
  - Scaling — divides raw 16-bit integers ($\pm32767$) by $32767.0$ to produce a clean float between −1.0 and 1.0.
- **Instant velocity updates** — speed directly mirrors the physical stick angle (unlike the keyboard's stepwise increments).
- **Hardware emergency stop** — any button press immediately resets the `Twist` to $0.0$ and publishes it to halt the rover.

### Motor Driver

The motor controller takes the `twist_mux` output and converts it into Pulse Width
Modulation (PWM) signals. The active driver is **`motor_driver_voidpp`**; all
calibration values (channels, neutral/forward/reverse pulse widths, steering center)
live in `config/motor_driver_config.yaml` so they can be tuned without touching code.

**`motor_driver_voidpp`**

- Uses [`pca9685_driver`](https://github.com/voidpp/PCA9685-driver) — a direct hardware interface for the PCA9685 chip (I2C address `0x42`, bus `1`).
- **Microsecond-to-tick conversion** — the PCA9685 is a 12-bit controller (4,096 ticks per PWM period):
  $$\text{ticks} = \left( \frac{\text{microseconds}}{20{,}000} \right) \times 4096$$
- **Precision control logic** (all values from `motor_driver_config.yaml`):
  - Linear X (throttle) — if $v > 0$, scales between `fwd_start_us` and `fwd_max_us`; if $v < 0$, between `rev_start_us` and `rev_max_us`.
  - Angular Z (steering) — centers wheels at `steer_center_us` and swings based on the steering input.
- **Deadzone handling** — ignores throttle inputs smaller than 0.02 to protect motors from jitter/stalling.
- **Safe shutdown** — `stop_motors()` returns throttle and steering to their neutral/center positions (`neutral_us`, `steer_center_us`) on node termination.

> **Note:** `teleop_joystick`, `teleop_keyboard`, and the motor driver are now
> config-driven. The previous standalone Adafruit motor driver has been retired,
> and earlier implementations moved to `rover/rover/legacy/`.

**Twist Stamper (Simulation)**

- Translates `geometry_msgs/Twist` into `geometry_msgs/TwistStamped`, adding:
  - `stamp` — the ROS 2 system time at the moment the message was created.
  - `frame_id` — the coordinate frame the velocity applies to.
- Reads `cmd_vel_mux` from `twist_mux` and forwards the stamped message to `ros2_control`.

### Running SLAM on Hardware

```bash
ros2 launch rover hardware.launch.py mode:=mapping   # mode: idle | mapping | nav | slam_nav (default idle)
```

> **Note:** `hardware.launch.py` does **not** start `twist_mux`. Run
> `twist_mux.launch.py` (plus a `motor_driver` / `teleop`) separately to drive the
> rover while mapping.

This initializes the rover's physical hardware and core navigation stack — from
reading the 3D model to starting the laser sensors and generating movement data.

**Core components:**

- **Robot State Publisher** — processes the Xacro/URDF to publish the robot's structure.
- **RPLidar A3 Driver** — spins the physical laser scanner and emits data.
- **RF2O Laser Odometry** — estimates motion from changes in the LiDAR scans.
- **SLAM Toolbox** — the mapping engine; builds a 2D map and localizes the robot within it.
- **Static Transform** — a fixed offset describing where the LiDAR is mounted.

**Why `rf2o_laser_odometry`?**

- Robots normally use wheel encoders to measure travel; this rover has none, so the odometry would break on slips or stalls.
- The **virtual encoder** performs planar laser scan matching — comparing a scan at time A to one at time B and inferring motion from how the "walls" moved.
- This yields a very stable `odom → base_link` transform with no wheel sensors.

**Data flow:**

```text
Physical World → rplidar_ros → /scan → rf2o_laser_odometry → /odom
/scan + /odom → SLAM Toolbox → Live 2D Map
```

### Saving a Map

Once you have driven (or [autonomously explored](#autonomous-exploration-frontier-explorer))
the area and the live map looks complete, persist it to disk. With the SLAM
stack still running (`mode:=mapping` or `slam_nav`), open another container
shell (`docker exec -it silnav2-container bash`) and run **one** of:

```bash
# Option A — Nav2 map_saver: writes the .pgm + .yaml pair used for nav/AMCL mode.
ros2 run nav2_map_server map_saver_cli -f ~/my_map
# produces ~/my_map.pgm and ~/my_map.yaml

# Option B — SLAM Toolbox serialization: also writes a .posegraph + .data pair
# that can be reloaded by SLAM Toolbox to continue mapping or localize.
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/workspace/ros2_ws/src/rover/maps/my_map'}}"
```

To use a saved map for localization, copy the `.pgm`/`.yaml` pair into
[`maps/`](ros2_ws/src/rover/maps), rebuild (`colcon build --symlink-install`),
and point `mode:=nav` at it. Note the hardware launch currently hardcodes
`maps/3rdfloor_office.yaml` as the static map
([`hardware.launch.py`](ros2_ws/src/rover/launch/hardware/hardware.launch.py)) —
update that path (or swap the file) to load a different map.

> **Map directory:** [`maps/`](ros2_ws/src/rover/maps) holds many historical
> captures; superseded ones live under `maps/outdated/`. The canonical static
> map for `mode:=nav` is `3rdfloor_office.yaml`. Prefer descriptive, dated names
> for new maps so the "current" one stays obvious.

---

## Autonomous Exploration (Frontier Explorer)

Automatically scans an entire unknown area without any teleop: the rover repeatedly
drives to the edges of the known map (the "frontiers" between free and unknown
space) until everything reachable has been mapped, then returns to its start
position.

- **Node:** `rover/frontier_explorer.py`

### How to run

```bash
# 1. Start the live SLAM + navigation stack
ros2 launch rover hardware.launch.py mode:=slam_nav

# 2. Start the explorer
ros2 run rover frontier_explorer

# 3. (Optional) Force an early return to start at any time
ros2 topic pub --once /frontier_explorer/return_home std_msgs/msg/Empty {}
```

It depends on a Nav2 stack that accepts `NavigateToPose` goals and a SLAM node
publishing `/map` and the `map → odom` TF — exactly what `mode:=slam_nav`
provides. It can also be used in simulation against a SLAM + Nav2 launch.

### How it works

1. **Startup delay** — waits `nav2_startup_delay` seconds for Nav2 costmaps to come up, records the start pose (the return-home target), then begins. *(An optional startup "wiggle" — drive forward/back to give SLAM initial motion — exists in the code but is currently commented out.)*
2. **Frontier detection** — every tick the occupancy grid is scanned for cells where free space borders unknown space. Adjacent frontier cells are grouped into clusters (BFS connected components); clusters smaller than `min_frontier_size` cells are ignored.
3. **Goal-point selection** — for each cluster a safe, reachable goal is chosen: a confirmed-free cell within `goal_max_dist_cells` of the frontier, picking the cell with the largest clearance (the most "open" spot, away from walls / the Nav2 inflation layer).
4. **Scoring** — each candidate is scored by `distance × goal_clearance³ × path_freeness`, where `path_freeness` is the fraction of sampled points along the straight line to the goal that are free and clear of walls. The highest-scoring, non-blacklisted frontier wins. If a frontier disappeared mid-navigation, frontiers near where it vanished get a proximity bonus.
5. **Navigation & monitoring** — the goal is sent to Nav2. While driving, the node:
   - cancels and replans if the target frontier **disappears** (the map filled in), after a short 0.5 s pause;
   - **blacklists** a goal and replans on a `nav_timeout` timeout;
   - runs its own **stuck detection** (no movement > `stuck_dist_m` within `stuck_timeout_s`) and replans — Nav2's default recovery (Spin) is wrong for an Ackermann vehicle that can't turn in place.
6. **Completion & return home** — when no actionable frontiers remain for `done_ticks_threshold` consecutive ticks (or all remaining frontiers are too close / blacklisted), exploration is declared complete and the rover navigates back to its recorded start pose.

### Key Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `min_frontier_size` | `10` | Minimum cluster size (cells) for a frontier to be considered. |
| `min_frontier_distance` | `1.0 m` | Ignore frontiers closer than this to the rover. |
| `goal_max_dist_cells` | `10` | How far (pixels) the goal point may sit from the frontier boundary. |
| `goal_blacklist_radius` | `0.5 m` | Matches goals against the blacklist and detects whether the active frontier still exists. |
| `nav_timeout` | `90 s` | Cancel + blacklist a goal that takes too long. |
| `stuck_dist_m` / `stuck_timeout_s` | `0.15 m` / `8 s` | Stuck-detection thresholds. |
| `done_ticks_threshold` | `5` | Consecutive empty ticks before exploration is declared finished. |
| `nav2_startup_delay` | `3.0 s` | Startup grace period. |
| `direction_switch_pause` | `3.0 s` | Cooldown between goals. |

### Interface

| Direction | Topic / Action | Type |
| --- | --- | --- |
| Subscribe | `/map` | `nav_msgs/OccupancyGrid` |
| Subscribe | `/frontier_explorer/return_home` | `std_msgs/Empty` |
| Publish | `cmd_vel` | `geometry_msgs/Twist` (used only by the optional wiggle) |
| Action client | `navigate_to_pose` | `nav2_msgs/NavigateToPose` |
| TF | `map → base_link` | Robot pose lookup |

---

## WiFi Heatmap

**1. Grant network capabilities** (inside the Docker container):

```bash
sudo setcap -r /usr/bin/python3.12
sudo setcap cap_net_admin+ep /usr/sbin/iwconfig
sudo setcap cap_net_admin+ep /usr/sbin/iw
```

**2. Log signal values** (check the WiFi interface name first with `iwconfig 2>&1`):

```bash
python3 wifi_signal_logger_ros2.py --interface wlan0 --interval 1.0 --output wifi_signal_log.csv
```

**3. Generate the heatmap:**

```bash
python3 wifi_heatmap.py wifi_signal_log.csv \
    --map  /workspace/ros2_ws/install/rover/share/rover/maps/3rd_floor_run1.pgm \
    --yaml /workspace/ros2_ws/install/rover/share/rover/maps/3rd_floor_run1.yaml \
    --radius 0.5 \
    --save wifi_heatmap.png
```

**4. View the image** — copy it out of the container, then onto your device:

```bash
# Container → Jetson
docker cp <container_name>:/workspace/ros2_ws/src/rover/rover/wifi_mapper/wifi_heatmap.png ~/wifi_heatmap.png

# Jetson → your machine
scp <user>@<jetson-ip>:~/wifi_heatmap.png ~/wifi_heatmap.png

# Open it
xdg-open ~/wifi_heatmap.png
```

---

## Project Structure

```text
.
├── Dockerfile
├── Dockerfile.laptop
├── README.md
├── build.sh
├── run.sh
├── VideosForPresentation
├── slides_build
└── ros2_ws
    └── src
        ├── rf2o_laser_odometry
        ├── rplidar_ros
        └── rover
            ├── config
            │   ├── accerPlanner.yaml
            │   ├── accerPlanner_hardware.yaml
            │   ├── mapper_params_online_async.yaml
            │   ├── motor_driver_config.yaml
            │   ├── nav2_slam_hardware.yaml
            │   ├── ros2_controllers.yaml
            │   ├── teleop_config.yaml
            │   └── twist_mux.yaml
            ├── launch
            │   ├── hardware
            │   │   ├── hardware.launch.py
            │   │   ├── motor_driver.launch.py
            │   │   ├── teleop.launch.py
            │   │   └── twist_mux.launch.py
            │   └── simulation
            │       ├── gz_plugin
            │       │   ├── robot_state_publisher_gzplugin.launch.py
            │       │   ├── ros_gz_bridge_gzplugin.launch.py
            │       │   └── simulation_gzplugin.launch.py
            │       ├── navigation2.launch.py
            │       ├── robot_state_publisher.launch.py
            │       ├── ros_gz_bridge.launch.py
            │       └── simulation.launch.py
            ├── maps                       # saved .pgm/.png/.yaml maps (+ outdated/)
            ├── meshes
            │   ├── MainBody.dae
            │   ├── Wheel.dae
            │   └── lidar_2d_v2.dae
            ├── rover
            │   ├── __init__.py
            │   ├── frontier_explorer.py   # autonomous full-map exploration
            │   ├── motor_driver_voidpp.py # PCA9685 PWM motor driver
            │   ├── teleop_joystick.py
            │   ├── teleop_keyboard.py
            │   ├── twist_stamper.py
            │   ├── legacy                 # retired teleop / driver implementations
            │   │   ├── teleop.py
            │   │   ├── teleop_joystick.py
            │   │   ├── teleop_joystick_voidpp.py
            │   │   ├── teleop_keyboard.py
            │   │   └── teleop_roscontrol.py
            │   └── wifi_mapper            # WiFi signal logger + heatmap tools
            │       ├── wifi_heatmap.py
            │       └── wifi_signal_logger_ros2.py
            ├── urdf
            │   ├── gazebo_plugins.xacro
            │   ├── lidar.xacro
            │   ├── ros2_control.xacro
            │   ├── rover.urdf.xacro
            │   ├── rover_core.xacro
            │   ├── rover_core_hardware.xacro
            │   ├── rover_gzplugin.urdf.xacro
            │   └── rover_roscontrol.urdf.xacro
            ├── rviz
            │   └── my_rviz_config.rviz
            ├── world
            │   └── world.sdf
            ├── test
            │   ├── test_copyright.py
            │   ├── test_flake8.py
            │   └── test_pep257.py
            ├── package.xml
            ├── pyproject.toml             # black / isort / pylint config
            ├── setup.cfg
            └── setup.py
```
