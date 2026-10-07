#!/bin/bash
# ==============================================================================
# Docker Build Script
# 
# Automatically detects the build environment (Hardware Robot vs. Simulation Laptop)
# and configures the build arguments accordingly.
# ==============================================================================

IMAGE_NAME=${1:-'silnav2:jammy-jazzy-harmonic'}
DOCKERFILE="Dockerfile"

set -x -e

# --- Configuration Logic based on Hostname ---

if [ "$HOSTNAME" = "rover-robot" ]; then
    echo ">>> Hardware Setup Detected (rover-robot)"
    
    # 1. Hardware Base Image
    #    Use the leaner 'ros-base' image. We don't need Gazebo/RViz installed 
    #    on the physical robot (rover-robot), saving disk space and build time.
    BASE_IMAGE="ros:jazzy-ros-base"
    
    # 2. Hardware User IDs
    #    Match the current user on the robot so mounted files have correct ownership.
    USER_ID="$(id -u)"
    GROUP_ID="$(id -g)"
    GROUP_NAME="$(id -gn)"
else
    # --- Simulation / Laptop Logic ---
    if [ "$HOSTNAME" = "sim-workstation" ]; then
         echo ">>> Simulation Setup Detected (sim-workstation)"
         USER_ID="$(id -u)"
         GROUP_ID="$(id -g)"
         GROUP_NAME="$(id -gn)"
    else
         echo ">>> Personal Laptop Detected"
         # Default fallback for generic personal laptops
         USER_ID=1000
         GROUP_ID=1000
         GROUP_NAME=dockeruser
    fi

    # 1. Simulation Base Image
    #    Use 'desktop-full' to include Gazebo, RViz, and GUI tools.
    BASE_IMAGE="osrf/ros:jazzy-desktop-full"
fi

echo "----------------------------------------------------"
echo "Building with Base Image : $BASE_IMAGE"
echo "----------------------------------------------------"

# --- Build Command ---
# --network host: Allows the build process to access the internet via the host's stack
docker build \
    --network host \
    --build-arg BASE_IMAGE="$BASE_IMAGE" \
    --build-arg USER_ID=$USER_ID \
    --build-arg GROUP_ID=$GROUP_ID \
    --build-arg GROUP_NAME=$GROUP_NAME \
    -t $IMAGE_NAME \
    -f $DOCKERFILE \
    .

set +x +e