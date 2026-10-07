#!/usr/bin/env python3
"""
Node capturing keyboard input for velocity commands.
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# Constants
LIN_STEP = 0.05
ANG_STEP = 0.1
MAX_VAL = 1.0

HELP_MSG = """
Reading from keyboard...
---------------------------
w/s : Throttle | a/d : Steering
space, k : stop | CTRL-C : quit
"""


class KeyboardTeleop(Node):
    """ROS2 Node for Keyboard Teleoperation."""

    def __init__(self):
        super().__init__("keyboard_teleop")

        self.declare_parameter("backward_limit", 0.4)
        self.declare_parameter("forward_limit", 0.65)

        self.b_limit = self.get_parameter("backward_limit").value
        self.f_limit = self.get_parameter("forward_limit").value

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.linear = 0.0
        self.angular = 0.0

        self.get_logger().info("Keyboard Teleop Started")
        print(HELP_MSG)
        self.create_timer(0.1, self.run_loop)

    def get_key(self):
        """Read single character without newline."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def stop_robot(self):
        """Send zero velocity for safety."""
        self.pub.publish(Twist())

    def run_loop(self):
        """Process keys and apply scaling."""
        key = self.get_key()

        if key == "w":
            self.linear = min(self.linear + LIN_STEP, MAX_VAL)
        elif key == "s":
            self.linear = max(self.linear - LIN_STEP, -MAX_VAL)
        elif key == "a":
            self.angular = min(self.angular + ANG_STEP, MAX_VAL)
        elif key == "d":
            self.angular = max(self.angular - ANG_STEP, -MAX_VAL)
        elif key in [" ", "k"]:
            self.linear, self.angular = 0.0, 0.0
        elif key == "\x03":
            raise KeyboardInterrupt

        twist = Twist()
        if self.linear > 0:
            twist.linear.x = self.linear * self.f_limit
        else:
            twist.linear.x = self.linear * self.b_limit

        twist.angular.z = self.angular
        self.pub.publish(twist)
        print(f"Lin: {twist.linear.x:.2f} | Ang: {twist.angular.z:.2f}",
              end="\r")


def main(args=None):
    """Node entry point."""
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)


if __name__ == "__main__":
    main()
