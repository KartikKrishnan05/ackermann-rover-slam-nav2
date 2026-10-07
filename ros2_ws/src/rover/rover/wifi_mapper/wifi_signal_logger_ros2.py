#!/usr/bin/env python3
"""
WiFi Signal Strength Logger — ROS2 / Nav2 Edition
===================================================
A ROS2 node that subscribes to Nav2's AMCL pose and continuously
logs WiFi signal strength (RSSI) alongside the rover's map position.

Subscribes to:
  - /amcl_pose  (geometry_msgs/PoseWithCovarianceStamped)  ← primary
  - /robot_pose (geometry_msgs/PoseStamped)                ← fallback

Publishes (optional, for RViz overlay):
  - /wifi_signal (std_msgs/Float32) — current RSSI in dBm

Usage:
  # Make sure your Nav2 stack is running, then:
  ros2 run <your_pkg> wifi_signal_logger_ros2.py

  # Or run directly:
  python3 wifi_signal_logger_ros2.py

  # With options:
  python3 wifi_signal_logger_ros2.py --interface wlan0 --interval 1.0 --output wifi_signal_log.csv

  # Run in background:
  python3 wifi_signal_logger_ros2.py &
"""

import argparse
import csv
import re
import subprocess
import signal
import sys
import time
import threading
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Float32

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
DEFAULT_INTERFACE = "wlan0"
DEFAULT_INTERVAL = 1.0
DEFAULT_OUTPUT = "wifi_signal_log.csv"

CSV_FIELDS = [
    "timestamp",
    "x", "y", "z",
    "orientation_z", "orientation_w",
    "ssid",
    "frequency_mhz",
    "signal_dbm",
    "link_quality",
    "link_quality_max",
    "noise_dbm",
    "bitrate_mbps",
    "tx_power_dbm",
]

# ---------------------------------------------------------------------------
# WiFi signal readers (same multi-backend approach)
# ---------------------------------------------------------------------------

def read_iwconfig(interface: str) -> dict:
    result = subprocess.run(
        ["iwconfig", interface],
        capture_output=True, text=True, timeout=5,
    )
    out = result.stdout
    info = {}

    m = re.search(r'ESSID:"([^"]*)"', out)
    if m: info["ssid"] = m.group(1)

    m = re.search(r"Frequency[:\s]*([\d.]+)\s*GHz", out)
    if m: info["frequency_mhz"] = int(float(m.group(1)) * 1000)

    m = re.search(r"Signal level[=:]\s*(-?\d+)\s*dBm", out)
    if m: info["signal_dbm"] = int(m.group(1))

    m = re.search(r"Link Quality[=:]\s*(\d+)/(\d+)", out)
    if m:
        info["link_quality"] = int(m.group(1))
        info["link_quality_max"] = int(m.group(2))

    m = re.search(r"Noise level[=:]\s*(-?\d+)\s*dBm", out)
    if m: info["noise_dbm"] = int(m.group(1))

    m = re.search(r"Bit Rate[=:]\s*([\d.]+)\s*Mb/s", out)
    if m: info["bitrate_mbps"] = float(m.group(1))

    m = re.search(r"Tx-Power[=:]\s*(\d+)\s*dBm", out)
    if m: info["tx_power_dbm"] = int(m.group(1))

    return info


def read_iw(interface: str) -> dict:
    info = {}
    try:
        result = subprocess.run(
            ["iw", "dev", interface, "link"],
            capture_output=True, text=True, timeout=5,
        )
        out = result.stdout

        m = re.search(r"SSID:\s*(.+)", out)
        if m: info["ssid"] = m.group(1).strip()

        m = re.search(r"freq:\s*(\d+)", out)
        if m: info["frequency_mhz"] = int(m.group(1))

        m = re.search(r"signal:\s*(-?\d+)\s*dBm", out)
        if m: info["signal_dbm"] = int(m.group(1))

        m = re.search(r"tx bitrate:\s*([\d.]+)\s*MBit/s", out)
        if m: info["bitrate_mbps"] = float(m.group(1))
    except Exception:
        pass
    return info


def get_signal(interface: str) -> dict:
    for reader in (read_iwconfig, read_iw):
        try:
            info = reader(interface)
            if "signal_dbm" in info:
                return info
        except Exception:
            continue
    return {}


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class WifiSignalLogger(Node):
    def __init__(self, interface: str, interval: float, output_path: str):
        super().__init__("wifi_signal_logger")
        self.interface = interface
        self.interval = interval
        self.output_path = output_path

        # Current pose (updated by subscriber callbacks)
        self._pose_lock = threading.Lock()
        self._current_pose = {
            "x": 0.0, "y": 0.0, "z": 0.0,
            "orientation_z": 0.0, "orientation_w": 1.0,
        }
        self._pose_received = False

        # QoS for Nav2 topics (transient local + reliable for amcl_pose)
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1,
        )

        # Subscribe to AMCL pose (primary — Nav2's localized position)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_pose_cb,
            amcl_qos,
        )

        # Subscribe to /robot_pose as fallback
        self.create_subscription(
            PoseStamped,
            "/robot_pose",
            self._robot_pose_cb,
            best_effort_qos,
        )

        # Also try /pose from slam_toolbox if used
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/pose",
            self._amcl_pose_cb,
            best_effort_qos,
        )

        # Publisher for RViz / debugging
        self.signal_pub = self.create_publisher(Float32, "/wifi_signal", 10)

        # Open CSV
        file_exists = Path(self.output_path).exists()
        self._csvfile = open(self.output_path, "a", newline="")
        self._writer = csv.DictWriter(
            self._csvfile, fieldnames=CSV_FIELDS, extrasaction="ignore",
        )
        if not file_exists:
            self._writer.writeheader()
            self._csvfile.flush()

        # Logging timer
        self._timer = self.create_timer(self.interval, self._log_sample)
        self._sample_count = 0

        self.get_logger().info(
            f"WiFi logger started — interface={self.interface}, "
            f"interval={self.interval}s, output={self.output_path}"
        )
        self.get_logger().info(
            "Waiting for pose on /amcl_pose, /robot_pose, or /pose ..."
        )

    # --- Pose callbacks -------------------------------------------------

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        with self._pose_lock:
            self._current_pose = {
                "x": p.position.x,
                "y": p.position.y,
                "z": p.position.z,
                "orientation_z": p.orientation.z,
                "orientation_w": p.orientation.w,
            }
            if not self._pose_received:
                self._pose_received = True
                self.get_logger().info(
                    f"Pose locked! First position: "
                    f"({p.position.x:.2f}, {p.position.y:.2f})"
                )

    def _robot_pose_cb(self, msg: PoseStamped):
        p = msg.pose
        with self._pose_lock:
            self._current_pose = {
                "x": p.position.x,
                "y": p.position.y,
                "z": p.position.z,
                "orientation_z": p.orientation.z,
                "orientation_w": p.orientation.w,
            }
            if not self._pose_received:
                self._pose_received = True
                self.get_logger().info(
                    f"Pose locked (robot_pose)! First position: "
                    f"({p.position.x:.2f}, {p.position.y:.2f})"
                )

    # --- Periodic sampling ----------------------------------------------

    def _log_sample(self):
        sig = get_signal(self.interface)

        with self._pose_lock:
            pose = dict(self._current_pose)

        ts = datetime.now(timezone.utc).isoformat()
        row = {"timestamp": ts, **pose, **sig}

        self._writer.writerow(row)
        self._csvfile.flush()
        self._sample_count += 1

        # Publish for RViz
        if "signal_dbm" in sig:
            msg = Float32()
            msg.data = float(sig["signal_dbm"])
            self.signal_pub.publish(msg)

        dbm = sig.get("signal_dbm", "N/A")
        self.get_logger().debug(
            f"#{self._sample_count}  pos=({pose['x']:.2f}, {pose['y']:.2f})  "
            f"signal={dbm} dBm"
        )

    def destroy_node(self):
        self._csvfile.close()
        self.get_logger().info(
            f"Stopped. {self._sample_count} samples → {self.output_path}"
        )
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="ROS2 WiFi signal logger using Nav2 pose",
    )
    parser.add_argument("-i", "--interface", default=DEFAULT_INTERFACE)
    parser.add_argument("-t", "--interval", type=float, default=DEFAULT_INTERVAL)
    parser.add_argument("-o", "--output", default=DEFAULT_OUTPUT)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = WifiSignalLogger(args.interface, args.interval, args.output)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()