# ==============================================================================
# Dockerfile for Rover Project (Hybrid: Simulation & Hardware)
# ==============================================================================
# 1. Allow the base image to be swapped by the build script.
#    - Simulation: uses 'osrf/ros:jazzy-desktop-full' (includes Gazebo/RViz)
#    - Hardware:   uses 'ros:jazzy-ros-base' (leaner, no GUI tools installed)
ARG BASE_IMAGE=osrf/ros:jazzy-desktop-full
FROM ${BASE_IMAGE}

# Zero interaction while installing or upgrading the system via apt.
ENV DEBIAN_FRONTEND=noninteractive

# ------------------------------------------------------------------------------
# System Dependencies
# ------------------------------------------------------------------------------
# Install essential build tools, networking utilities, and Python prerequisites.
RUN apt-get update && apt-get -y upgrade && \
    apt-get -y install apt-utils pciutils psmisc \
    git build-essential python3-pip \
    wget curl vim nano catimg tmux xterm \
    iputils-ping net-tools iproute2 \
    python3-setuptools python3-wheel x11-apps python3.12-venv \
    gdb \
    cmake \
    unzip \
    pkg-config \
    python3-dev \
    libeigen3-dev \
    apt-transport-https \
    ca-certificates \
    software-properties-common \
    libi2c-dev i2c-tools libffi-dev \
    # --- WiFi Signal Mapper Tools ---
    wireless-tools \
    iw && \
    rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------------------
# Locale Configuration
# ------------------------------------------------------------------------------
# ROS 2 requires a UTF-8 locale to function correctly.
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 && \
    dpkg-reconfigure --frontend=noninteractive locales && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Remove the default ubuntu user to avoid UID/GID conflicts later
RUN userdel -r ubuntu || true

WORKDIR /

# ------------------------------------------------------------------------------
# ROS 2 Package Installation
# ------------------------------------------------------------------------------
# Install the core navigation, control, and simulation packages.
# Note: $ROS_DISTRO is inherited from the base image (jazzy).
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop-full \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-message-filters \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-topic-tools \
    ros-$ROS_DISTRO-tf2-tools \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-gz-ros2-control \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ackermann-steering-controller \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-eigen \
    ros-$ROS_DISTRO-twist-mux && \
    rm -rf /var/lib/apt/lists/*


RUN pip install --break-system-packages \
    adafruit-circuitpython-servokit \
    Jetson.GPIO \
    PCA9685-driver \
    # --- WiFi Signal Mapper Python Dependencies ---
    pandas \
    matplotlib \
    scipy \
    pyyaml

WORKDIR /

# ------------------------------------------------------------------------------
# User Configuration
# ------------------------------------------------------------------------------
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG GROUP_NAME=dockeruser

# 1. Create the groups first
RUN groupadd -g ${GROUP_ID} -o ${GROUP_NAME} && \
    groupadd -f -r -g 997 gpio

# 2. Create the user and assign ALL groups at once
RUN useradd -m -r -u ${USER_ID} -g ${GROUP_ID} -o -s /bin/bash \
    -G adm,dialout,cdrom,floppy,sudo,audio,dip,video,plugdev,gpio,${GROUP_NAME} \
    dockeruser && \
    echo "dockeruser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    passwd -d dockeruser

# 3. (Optional but recommended) Install udev rules
RUN wget https://raw.githubusercontent.com/NVIDIA/jetson-gpio/master/lib/python/Jetson/GPIO/99-gpio.rules -P /etc/udev/rules.d/

# ------------------------------------------------------------------------------
# Shell Environment Setup (.bashrc)
# ------------------------------------------------------------------------------
# Configure the shell prompt, ROS domain ID, and source the workspace setup files
# automatically when a new terminal is opened.
RUN echo 'PS1="(nav2-docker) $PS1"' >> /home/dockeruser/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /home/dockeruser/.bashrc && \
    echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" >> /home/dockeruser/.bashrc && \
    echo "export GZ_PARTITION=docker_sim" >> /home/dockeruser/.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/dockeruser/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/dockeruser/.bashrc && \
    echo "source /workspace/ros2_ws/install/setup.bash" >> /home/dockeruser/.bashrc && \
    # Append the Officesim path so Gazebo can find the world assets
    echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:/workspace/ros2_ws/src/rover/officesim" >> /home/dockeruser/.bashrc && \
    # WiFi mapper on PATH
    echo "export PATH=\$PATH:/home/wifi_mapper" >> /home/dockeruser/.bashrc && \
    echo "set -g mouse on" > /home/dockeruser/.tmux.conf