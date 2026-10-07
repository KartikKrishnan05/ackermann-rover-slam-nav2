#!/usr/bin/env python3
"""
Node interfacing with Linux joystick devices for robot control.
"""

import os
import struct

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class JoystickTeleop(Node):
    """ROS2 Node for Joystick Teleoperation."""

    def __init__(self):
        super().__init__("joystick_teleop")

        self.declare_parameter("joy_dev", "/dev/input/js0")
        self.declare_parameter("backward_limit", 0.4)
        self.declare_parameter("forward_limit", 0.65)
        self.declare_parameter("max_joystick_value", 32767.0)

        self.joy_dev = self.get_parameter("joy_dev").value
        self.b_limit = self.get_parameter("backward_limit").value
        self.f_limit = self.get_parameter("forward_limit").value
        self.max_joy = self.get_parameter("max_joystick_value").value

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.event_size = 8

        if not os.path.exists(self.joy_dev):
            self.get_logger().error(f"Joystick not found at {self.joy_dev}")
            return

        self.jsdev = open(self.joy_dev, "rb")
        self.get_logger().info(f"Joystick Node Started on {self.joy_dev}")

        self.create_timer(0.01, self.read_joystick)
        self.twist = Twist()

    def read_joystick(self):
        """Read and parse binary joystick data."""
        try:
            evbuf = self.jsdev.read(self.event_size)
            if evbuf:
                _, val, ev_type, num = struct.unpack("IhBB", evbuf)

                # Axis Logic (0x02)
                if ev_type & 0x02:
                    norm_val = val / self.max_joy
                    if num == 1:  # Throttle
                        target_linear = -norm_val
                        if target_linear < 0:
                            self.twist.linear.x = target_linear * self.b_limit
                        else:
                            self.twist.linear.x = target_linear * self.f_limit
                    elif num == 2:  # Steering
                        if self.twist.linear.x < 0:
                            self.twist.angular.z = norm_val * 0.5
                        else:
                            self.twist.angular.z = -norm_val * 0.5

                    self.pub.publish(self.twist)

                # Button Logic (0x01)
                if (ev_type & 0x01) and val == 1:
                    self.get_logger().warn("EMERGENCY STOP")
                    self.twist = Twist()
                    self.pub.publish(self.twist)
        except (IOError, struct.error):
            pass


def main(args=None):
    """Node entry point."""
    rclpy.init(args=args)
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
