#!/usr/bin/env python3
"""
Motor Driver node for PCA9685 PWM controller using voidpp driver.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    from pca9685_driver import Device  # type: ignore
except ImportError:
    from PCA9685 import PCA9685 as Device  # type: ignore


class VoidppMotorDriver(Node):
    """ROS2 Node for Voidpp-style Motor Control."""

    def __init__(self):
        super().__init__("motor_driver_voidpp")

        # Parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("i2c_address", 0x42),
                ("i2c_bus", 1),
                ("throttle_channel", 0),
                ("steering_channel", 1),
                ("neutral_us", 1530.0),
                ("fwd_start_us", 1680.0),
                ("fwd_max_us", 2030.0),
                ("rev_start_us", 1420.0),
                ("rev_max_us", 1200.0),
                ("steer_center_us", 1900.0),
            ],
        )

        addr = self.get_parameter("i2c_address").value
        bus = self.get_parameter("i2c_bus").value
        self.device = Device(addr, bus)
        self.device.set_pwm_frequency(50)

        self.sub = self.create_subscription(
            Twist, "cmd_vel_mux", self.callback, 10
        )
        self.get_logger().info("Voidpp Driver Node Ready")

    def us_to_ticks(self, us):
        """Convert microseconds to 12-bit register ticks."""
        return int((us / 20000.0) * 4096)

    def callback(self, msg):
        """Translate Twist to PWM pulses."""
        neutral = self.get_parameter("neutral_us").value
        f_s = self.get_parameter("fwd_start_us").value
        f_m = self.get_parameter("fwd_max_us").value
        r_s = self.get_parameter("rev_start_us").value
        r_m = self.get_parameter("rev_max_us").value
        s_c = self.get_parameter("steer_center_us").value

        # s_us = s_c + (msg.angular.z * 500.0)
        # Nav2 return us the angular velocity (Steering values) ONLY while driving backwards 
        # Meaning we have to reverse the +- Sign in the front to counter that

        if abs(msg.linear.x) < 0.005:
            t_us = neutral
            s_us = max(s_c + (msg.angular.z * 1266.0), 0)
        elif msg.linear.x > 0:
            t_us = max(f_s + (abs(msg.linear.x) * (f_m - f_s)), 0)
            s_us = max(s_c + (msg.angular.z * 1266.0), 0)
        else:
            t_us = max(r_s + (abs(msg.linear.x) * (r_m - r_s)), 0)
            s_us = max(s_c + (msg.angular.z * -1266.0), 0)

        t_ch = self.get_parameter("throttle_channel").value
        s_ch = self.get_parameter("steering_channel").value
        self.device.set_pwm(t_ch, self.us_to_ticks(t_us))
        self.device.set_pwm(s_ch, self.us_to_ticks(s_us))

    def stop_motors(self):
        """Reset throttle to neutral."""
        t_ch = self.get_parameter("throttle_channel").value
        neutral = self.get_parameter("neutral_us").value
        self.device.set_pwm(t_ch, self.us_to_ticks(neutral))


def main(args=None):
    """Node entry point."""
    rclpy.init(args=args)
    node = VoidppMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
