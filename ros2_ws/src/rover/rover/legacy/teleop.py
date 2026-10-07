#!/usr/bin/env python3
"""
Teleoperation node for the Rover inside of the simulation.

The node lets a user drive the rover from a keyboard while applying a
simple smoothing filter to avoid abrupt accelerations.  It publishes
geometry_msgs/Twist messages on ``/cmd_vel`` (or any other topic
configured via ``TOPIC_NAME``).  The script is completely self-contained
and can be launched with::

    ros2 run rover teleop_simple.py

Key bindings
------------
* **w / s** - accelerate forward / backward (held for continuous motion)
* **a / d** - steer left / right (incremental)
* **SPACE** - emergency stop (zero speed and steering)
* **CTRL-C** - exit the program
"""

# -------------------------------------------------------------------------
# Standard‑library imports
# -------------------------------------------------------------------------
import select                 # non‑blocking stdin polling
import sys                    # access to stdin / stdout
import termios                # terminal raw‑mode handling
import tty                    # helper for raw mode

# -------------------------------------------------------------------------
# ROS 2 imports
# -------------------------------------------------------------------------
import rclpy                                 # core ROS‑2 Python API
from geometry_msgs.msg import Twist         # velocity command message type
from rclpy.node import Node                 # base class for ROS‑2 nodes

# -------------------------------------------------------------------------
# Configuration constants (tunable at the top of the file)
# -------------------------------------------------------------------------
TOPIC_NAME = "/cmd_vel"          # ROS topic for velocity commands
MAX_SPEED = 0.5                  # [m s⁻¹] absolute linear speed limit
MAX_STEER = 0.52                 # [rad] absolute steering angle limit
STEER_STEP = 0.05                # [rad] steering increment per key press

# Smoothing settings
SMOOTH_STEP = 0.15               # speed change per loop iteration

# Human‑readable help shown on the console
INSTRUCTIONS = """
---------------------------
ROS 2 Smoothed Teleop
---------------------------
    w
a   s   d

Hold w/s: Accelerate Forward/Backward
Release:  Decelerate gently to stop
a/d:      Steer Left/Right
CTRL-C:   Quit

Target: {:.2f} | Current Speed: {:.2f} | Steer: {:.2f}
"""


class RoverTeleop(Node):
    """
    ROS 2 node that publishes ``Twist`` commands generated from keyboard input.
    """

    def __init__(self):
        """Create the node and initialise the velocity publisher."""
        super().__init__("rover_teleop_node")
        self.publisher_ = self.create_publisher(Twist, TOPIC_NAME, 10)

    def send_command(self, speed: float, steer: float) -> None:
        """
        Build and publish a ``Twist`` message.

        Parameters
        ----------
        speed : float
            Desired linear velocity in metres per second.
        steer : float
            Desired angular velocity around the Z-axis (radians per second).
        """
        twist = Twist()
        twist.linear.x = float(speed)
        twist.angular.z = float(steer)
        self.publisher_.publish(twist)


# -------------------------------------------------------------------------
# Utility function - non‑blocking single‑key reader
# -------------------------------------------------------------------------
def get_key(settings) -> str:
    """
    Capture one character from *stdin* without blocking indefinitely.

    The terminal is switched to raw mode for the short ``select`` timeout,
    then restored to its original settings.

    Parameters
    ----------
    settings : list
        Original terminal attributes obtained via ``termios.tcgetattr``.

    Returns
    -------
    str
        The read character, or an empty string if the timeout expires.
    """
    tty.setraw(sys.stdin.fileno())
    # 0.1 s timeout defines the update rate of the smoothing loop
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# -------------------------------------------------------------------------
# Main entry point - orchestrates the teleoperation loop
# -------------------------------------------------------------------------
def main(args=None) -> None:
    """
    Initialise ROS, create the teleop node and run the interactive loop.

    The loop repeatedly:
    1. Reads a key (non-blocking).
    2. Updates the *target* linear speed and steering angle.
    3. Smooths the command speed toward the target.
    4. Publishes the resulting ``Twist`` message.
    5. Prints a live status line on the console.
    """
    rclpy.init(args=args)
    node = RoverTeleop()

    # Preserve the original terminal settings to restore them on exit
    settings = termios.tcgetattr(sys.stdin)

    # -----------------------------------------------------------------
    # State variables - keep track of user intent and smoothed commands
    # -----------------------------------------------------------------
    steer = 0.0               # current steering angle request
    target_speed = 0.0        # speed that the user wants (set by keys)
    control_speed = 0.0       # speed actually sent to the robot (smoothed)

    try:
        # Print the help / status line before entering the loop
        print(INSTRUCTIONS.format(target_speed, control_speed, steer))

        # -------------------------------------------------------------
        # Main control loop - runs until ROS is shut down or Ctrl‑C pressed
        # -------------------------------------------------------------
        while rclpy.ok():
            key = get_key(settings)

            # ---------------------------------
            # 1️⃣  DETERMINE TARGETS
            # ---------------------------------
            if key == "w":
                target_speed = MAX_SPEED
            elif key == "s":
                target_speed = -MAX_SPEED
            elif key == "a":
                steer = min(MAX_STEER, steer + STEER_STEP)
            elif key == "d":
                steer = max(-MAX_STEER, steer - STEER_STEP)
            elif key == " ":
                target_speed = 0.0
                control_speed = 0.0
                steer = 0.0
            elif key == "\x03":        # Ctrl‑C
                break
            else:
                # No relevant key pressed → target speed collapses to zero
                target_speed = 0.0

            # ---------------------------------
            # 2️⃣  SMOOTHING - move control speed toward target
            # ---------------------------------
            if control_speed < target_speed:
                control_speed = min(
                    target_speed, control_speed + SMOOTH_STEP
                )
            elif control_speed > target_speed:
                control_speed = max(
                    target_speed, control_speed - SMOOTH_STEP
                )

            # If we are very close, snap to the exact target value
            if abs(control_speed - target_speed) < 0.01:
                control_speed = target_speed

            # ---------------------------------
            # 3️⃣  SEND COMMAND
            # ---------------------------------
            node.send_command(control_speed, steer)

            # Update the on‑screen status line (single line, carriage return)
            sys.stdout.write(
                f"\rTarget: {target_speed:.2f} | "
                f"Current Speed: {control_speed:.2f} | "
                f"Steer: {steer:.2f}   "
            )
            sys.stdout.flush()

    except Exception as error:          # pylint: disable=broad-except
        # Any unexpected exception is printed before shutdown
        print(error)

    finally:
        # ---------------------------------------------------------
        # Clean shutdown - ensure robot receives a final stop command
        # and the terminal is restored to its original state.
        # ---------------------------------------------------------
        node.send_command(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


# -------------------------------------------------------------------------
# Script entry point - executed only when the file is run directly
# -------------------------------------------------------------------------
if __name__ == "__main__":
    main()
