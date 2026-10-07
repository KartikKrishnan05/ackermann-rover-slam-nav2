"""
Setup configuration for the rover package.

This script defines the ROS 2 package structure and installation rules:

* **Package Identity:** Sets the package name, version, and maintainer info.
* **Data Files:** Maps source files (launch, config, maps, URDFs) to their
    installation destinations in the ``share/`` directory.
* **Recursive Copying:** Automatically traverses specific directories (like
    meshes and worlds) to ensure all nested assets are installed.
* **Entry Points:** Registers the executable nodes (`teleop`, `twist_stamper`)
    so they can be run via ``ros2 run``.
"""

import os
from glob import glob
from setuptools import setup

PACKAGE_NAME = "rover"

# -------------------------------------------------------------------------
# Define Data Files for Installation
# -------------------------------------------------------------------------
# The data_files list maps source files to their destination in the install
# directory (usually install/rover/share/rover/...).
# -------------------------------------------------------------------------
data_files = [
    # 1. Marker file for ament index to find the package
    (
        os.path.join("share", "ament_index", "resource_index", "packages"),
        ["resource/" + PACKAGE_NAME],
    ),
    # 2. Package configuration (package.xml)
    (os.path.join("share", PACKAGE_NAME), ["package.xml"]),
    # 3. Launch files (organized by category)
    (
        os.path.join("share", PACKAGE_NAME, "launch", "simulation"),
        glob("launch/simulation/*.launch.py"),
    ),
    (
        os.path.join("share", PACKAGE_NAME, "launch", "hardware"),
        glob("launch/hardware/*.launch.py"),
    ),
    (
        os.path.join(
            "share",
            PACKAGE_NAME,
            "launch",
            "simulation",
            "gz_plugin"
        ),
        glob("launch/gz_plugin/*.launch.py"),
    ),
    # 4. RViz configurations
    (os.path.join("share", PACKAGE_NAME, "rviz"), glob("rviz/*.rviz")),
]

# -------------------------------------------------------------------------
# Recursive Directory Copying
# -------------------------------------------------------------------------
# ROS 2 setup.py doesn't support recursive copying by default like CMake.
# We must manually walk through these folders to ensure nested files (like
# mesh textures or map files) are installed correctly.
# -------------------------------------------------------------------------
FOLDERS_TO_COPY = ["maps", "config", "urdf", "meshes", "world"]

for folder in FOLDERS_TO_COPY:
    for path, _, filenames in os.walk(folder):
        if filenames:
            # Construct the destination path relative to the share directory
            destination_path = os.path.join("share", PACKAGE_NAME, path)
            # Create list of full source paths
            source_files = [os.path.join(path, f) for f in filenames]
            data_files.append((destination_path, source_files))

# -------------------------------------------------------------------------
# Setup Configuration
# -------------------------------------------------------------------------
setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=[PACKAGE_NAME],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="Rover Package",
    license="TODO",
    tests_require=["pytest"],
    # -----------------------------------------------------------------
    # Entry Points (Executable Scripts)
    # -----------------------------------------------------------------
    # This section registers the Python functions that become ROS 2 nodes.
    # Format: "node_name = package_name.module_name:function_name"
    # -----------------------------------------------------------------
    entry_points={
        "console_scripts": [
            "teleop_keyboard = rover.teleop_keyboard:main",
            "teleop_joystick = rover.teleop_joystick:main",
            "motor_driver_adafruit = rover.motor_driver_adafruit:main",
            "motor_driver_voidpp = rover.motor_driver_voidpp:main",
            "twist_stamper = rover.twist_stamper:main",
        ],
    },
)
