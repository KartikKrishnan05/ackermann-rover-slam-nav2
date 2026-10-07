#!/bin/bash
# ==============================================================================
# Docker Run Script
# 
# Launches the container with specific configurations for Hardware or Simulation.
# Handles mounting volumes, X11 forwarding, and device access.
# ==============================================================================

ROS_WS_LOCAL="$PWD/ros2_ws"
ROS_WS_CONTAINER="/workspace/ros2_ws"
IMAGE_NAME=${1:-'silnav2:jammy-jazzy-harmonic'}
CONTAINER_NAME=${2:-'silnav2-container'}

# Create a runtime directory for XDG (necessary for some GUI applications in Docker)
XDG_RUNTIME_DIR_HOST="/tmp/runtime-dockeruser"
mkdir -p $XDG_RUNTIME_DIR_HOST
chmod 700 $XDG_RUNTIME_DIR_HOST

# Initialize empty hardware arguments
HW_ARGS=""


# --- Hardware Specific Flags (rover-robot) ---
if [ "$HOSTNAME" = "rover-robot" ]; then
    echo ">>> Hardware Setup Detected (rover-robot)"
    
    # Ensure I2C permissions on the host/container bridge
    if [ -e /dev/i2c-1 ]; then
        echo ">>> Setting I2C permissions..."
        sudo chmod 666 /dev/i2c-1
        I2C_DEV="--device=/dev/i2c-1"
    fi

    if [ -e /dev/input/js0 ]; then
        JS_DEV="--device=/dev/input/js0"
    else
        echo ">>> WARNING: No joystick detected at /dev/input/js0. Controller will not be available!"
    fi

    if ls /dev/gpiochip* > /dev/null 2>&1; then
        sudo chmod 666 /dev/gpiochip*
    fi

    GPIO_DEV=""
    [ -e /dev/gpiochip0 ] && GPIO_DEV="--device=/dev/gpiochip0"

    HW_ARGS="--runtime nvidia \
             --privileged \
             --gpus all \
             --device=/dev/ttyUSB0 \
             $JS_DEV \
             $I2C_DEV \
             $GPIO_DEV \
             --group-add dialout \
             --group-add input \
             --group-add 997 \
             --mount type=bind,src=/etc/localtime,dst=/etc/localtime,readonly \
             --mount type=bind,src=/etc/timezone,dst=/etc/timezone,readonly"
else
    echo ">>> Simulation/Laptop Setup Detected"
fi

set -x

# --- Run Command ---
docker run -d -it \
    --name=$CONTAINER_NAME \
    --user=dockeruser \
    --net=host \
    --ipc=host \
    --mount type=bind,src=$ROS_WS_LOCAL,dst=$ROS_WS_CONTAINER \
    --mount type=bind,src=/tmp/.X11-unix/,dst=/tmp/.X11-unix \
    --mount type=bind,src=$HOME/.Xauthority,dst=/home/dockeruser/.Xauthority \
    --mount type=bind,src=$XDG_RUNTIME_DIR_HOST,dst=$XDG_RUNTIME_DIR_HOST \
    --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR_HOST \
    --env DISPLAY=$DISPLAY \
    --env TERM \
    $HW_ARGS \
    $IMAGE_NAME \
    tail -f /dev/null

set +x