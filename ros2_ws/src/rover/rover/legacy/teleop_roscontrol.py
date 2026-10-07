#!/usr/bin/env python3
"""
Teleoperation node for the Rover (Twist-Mux edition) with a safety Parking
Brake.

Purpose
-------
Provides a tiny keyboard-driven teleoperation interface for a
Ackermann-steered rover in ROS 2.  The node publishes
``geometry_msgs/msg/Twist`` messages on the ``/cmd_vel_teleop`` topic.  A
``twist_mux`` node can then arbitrate between this manual source and other
controllers (e.g. Nav2).

Features
--------
* Ackermann kinematics - converts a linear speed and a steering angle to the
  angular-z component required by an Ackermann vehicle model.
* Input smoothing - ramps speed up/down in small steps to avoid jerky motion.
* Parking brake (TAB) - when active a constant zero-velocity command is
  published, locking the ``twist_mux`` in the “brake” position.
* Idle state - when no key is pressed the node stops publishing, allowing
  Nav2 (or any other controller) to take control via the ``twist_mux``.
* On-screen status - shows target speed, smoothed command speed, steering
  angle and the current mode (DRIVING / PARKING BRAKE / STOPPED / IDLE).
"""

# -------------------------------------------------------------------------
# Standard library imports
# -------------------------------------------------------------------------
import math                 # trigonometric functions for Ackermann conversion
import select               # non-blocking stdin polling
import sys                  # access to stdin / stdout
import termios              # terminal control (raw mode handling)
import tty                  # helper for raw mode

# -------------------------------------------------------------------------
# ROS-2 imports
# -------------------------------------------------------------------------
import rclpy                            # core ROS-2 Python API
from geometry_msgs.msg import Twist    # velocity command message type
from rclpy.node import Node            # base class for ROS-2 nodes
from rclpy.parameter import Parameter  # used to set the ``use_sim_time`` flag

# -------------------------------------------------------------------------
# Configuration constants (tunable at the top of the file)
# -------------------------------------------------------------------------
TOPIC_NAME = "/cmd_vel"          # topic on which commands are published

# Speed limits (hard caps for safety)
MAX_SPEED = 2.0            # [m/s]   maximum absolute linear speed
MAX_STEER_ANGLE = 0.65     # [rad]   maximum absolute steering angle (~37°)

# Increment steps when a key is pressed
STEER_STEP = 0.05          # [rad]   steering change per key press
SPEED_STEP = 0.5           # [m/s]   speed change per key press

# Smoothing step - how fast the command speed approaches the target
SMOOTH_STEP = 0.2          # [m/s] per iteration

# Physical parameter of the rover (used in Ackermann conversion)
WHEELBASE = 1.0            # distance between front and rear axle [m]

# Human-readable help shown on the console
INSTRUCTIONS = (
    "---------------------------\n"
    "ROS 2 Smoothed Teleop (Twist Mux)\n"
    "---------------------------\n"
    "    w\n"
    "a   s   d\n\n"
    "w/s : Increase/Decrease Speed\n"
    "a/d : Steer Left/Right\n"
    "SPACE : Reset Speed (Stop)\n"
    "TAB   : TOGGLE PARKING BRAKE (Locks Mux)\n"
    "CTRL-C : Quit\n\n"
    "Target: {:.2f} | Current: {:.2f} | Angle: {:.2f}"
)


# -------------------------------------------------------------------------
# Helper class - ROS-2 node that publishes Twist messages
# -------------------------------------------------------------------------
class RoverTeleop(Node):
    """
    ROS 2 node responsible for publishing ``Twist`` commands.

    The node contains the boiler-plate for parameter handling, publisher
    creation and a small convenience method ``send_command``.
    """

    def __init__(self):
        """Create the node and initialise the publisher."""
        super().__init__("rover_teleop_node")

        # Respect simulation time when Gazebo is used.
        self.set_parameters(
            [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
        )

        # Publisher for the teleop velocity commands.
        self.publisher_ = self.create_publisher(Twist, TOPIC_NAME, 10)

        # Log the topic name for the user.
        self.get_logger().info(
            f"Publishing teleop commands on: {TOPIC_NAME}"
        )

    def send_command(self, speed: float, steer_angle: float) -> None:
        """
        Build and publish a ``Twist`` message using Ackermann steering geometry

        The angular velocity around the Z-axis is derived from the
        curvature of a vehicle with wheelbase ``WHEELBASE``:

            ω_z = (v / L) * tan(δ)

        where ``v`` is the linear speed, ``L`` the wheelbase and ``δ`` the
        steering angle.  For very low speeds the steering angle is forwarded
        directly to keep the calculation numerically stable.

        Parameters
        ----------
        speed : float
            Desired linear velocity (m/s).
            Positive = forward, negative = reverse.
        steer_angle : float
            Desired steering angle (rad).
            Positive = left turn, negative = right.
        """
        msg = Twist()
        msg.linear.x = float(speed)

        # Apply Ackermann conversion only when the rover is actually moving.
        if WHEELBASE > 0 and abs(speed) > 0.01:
            msg.angular.z = (speed / WHEELBASE) * math.tan(steer_angle)
        else:
            # Vehicle is essentially stationary - forward the angle unchanged.
            msg.angular.z = float(steer_angle)

        self.publisher_.publish(msg)


# -------------------------------------------------------------------------
# Utility function - non-blocking single-key reader
# -------------------------------------------------------------------------
def get_key(settings, timeout: float = 0.1) -> str:
    """
    Read a single character from stdin without blocking the program.

    The terminal is temporarily switched to *raw* mode so that each key press
    is reported immediately (instead of waiting for a newline).  After reading
    the character the original terminal settings are restored.

    Parameters
    ----------
    settings : list
        Original terminal settings saved with ``termios.tcgetattr`` - required
        for restoration.
    timeout : float, optional
    Maximum time (seconds) to wait for a key press before returning an empty
    string.  A short timeout keeps the main loop responsive.

    Returns
    -------
    str
        The character read from stdin, or ``""`` if the timeout elapsed.
    """
    tty.setraw(sys.stdin.fileno())               # enable raw mode
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ""    # read a single char
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# -------------------------------------------------------------------------
# Main entry point - orchestrates the teleoperation loop
# -------------------------------------------------------------------------
def main() -> None:
    """
    Initialise ROS, instantiate the node and run the interactive control loop.

    The loop performs the following steps on each iteration:
    1. Read a key (non-blocking).
    2. Update internal targets (speed / steering) based on the key.
    3. Apply smoothing so that ``control_speed`` gradually follows
       ``target_speed``.
    4. Decide which mode is active (parking brake, driving, stopped, idle)
       and publish the appropriate ``Twist`` command.
    5. Provide live console feedback.
    """
    rclpy.init()
    node = RoverTeleop()

    # Save the current terminal configuration so we can restore it on exit.
    settings = termios.tcgetattr(sys.stdin)

    # -----------------------------------------------------------------
    # State variables - keep track of the user’s intent and the smoothed
    # commands.
    # -----------------------------------------------------------------
    target_speed = 0.0     # speed requested by the user (via keys)
    control_speed = 0.0    # speed actually sent to the robot (smoothed)
    steer_angle = 0.0      # current steering angle request

    # Flags that influence behaviour
    was_active = True      # remembers if we were publishing non-zero commands
    parking_brake = False  # true while the parking-brake (TAB) is engaged

    try:
        # Show the short help / status line before entering the loop.
        print(INSTRUCTIONS.format(target_speed, control_speed, steer_angle))

        # -----------------------------------------------------------------
        # Main control loop - runs until ROS is shut down or Ctrl-C is pressed.
        # -----------------------------------------------------------------
        while rclpy.ok():
            key = get_key(settings)

            # -------------------------------------------------------------
            # 1️⃣ INPUT HANDLING
            # -------------------------------------------------------------
            if key == "\t":                     # TAB - toggle parking brake
                parking_brake = not parking_brake
                if parking_brake:
                    # When the brake is engaged we immediately clear all
                    # targets to avoid a sudden surge.
                    target_speed = 0.0
                    control_speed = 0.0
                    steer_angle = 0.0

            elif key == "w" and not parking_brake:
                target_speed = min(MAX_SPEED, target_speed + SPEED_STEP)
            elif key == "s" and not parking_brake:
                target_speed = max(-MAX_SPEED, target_speed - SPEED_STEP)
            elif key == "a" and not parking_brake:
                steer_angle = min(MAX_STEER_ANGLE, steer_angle + STEER_STEP)
            elif key == "d" and not parking_brake:
                steer_angle = max(-MAX_STEER_ANGLE, steer_angle - STEER_STEP)
            elif key == " ":                     # SPACE - emergency stop
                target_speed = 0.0
                control_speed = 0.0
                steer_angle = 0.0
            elif key == "\x03":                 # Ctrl-C
                break

            # -------------------------------------------------------------
            # 2️⃣ SMOOTHING - gradual approach to the target speed
            # -------------------------------------------------------------
            if not parking_brake:
                if control_speed < target_speed:
                    control_speed = min(
                        target_speed, control_speed + SMOOTH_STEP
                    )
                elif control_speed > target_speed:
                    control_speed = max(
                        target_speed, control_speed - SMOOTH_STEP
                    )

                # Snap to the exact target when we are sufficiently close.
                if abs(control_speed - target_speed) < 0.05:
                    control_speed = target_speed
            else:
                # In parking-brake mode we force the sent speed to zero.
                control_speed = 0.0

            # -------------------------------------------------------------
            # 3️⃣ PUBLISHING LOGIC - decide which message (if any) to send
            # -------------------------------------------------------------
            if parking_brake:
                # Parking brake continuously publishes zeros to keep the
                # ``twist_mux`` in the “brake” position.
                node.send_command(0.0, 0.0)
                was_active = True
                sys.stdout.write(
                    "\r*** PARKING BRAKE *** | Press TAB to release      "
                )
            elif (abs(control_speed) > 0.001) or (abs(steer_angle) > 0.001):
                # Driving mode - send the smoothed command.
                node.send_command(control_speed, steer_angle)
                was_active = True
                sys.stdout.write(
                    f"\rSpeed: {control_speed:.2f} | "
                    f"Steer: {steer_angle:.2f} | "
                    "State: DRIVING (Block Nav2) "
                )
            else:
                # No brake and no movement - either STOPPED (just after motion)
                # or IDLE (let Nav2 control the robot).
                if was_active:
                    # Transition from motion to stand-still - send a final zero
                    node.send_command(0.0, 0.0)
                    was_active = False
                    sys.stdout.write(
                        f"\rSpeed: {0.0:.2f} | "
                        f"Steer: {steer_angle:.2f} | "
                        "State: STOPPED              "
                    )
                else:
                    # Fully idle - no command is published.
                    sys.stdout.write(
                        f"\rSpeed: {0.0:.2f} | "
                        f"Steer: {steer_angle:.2f} | "
                        "State: IDLE (Nav2 Enabled)  "
                    )

            # Flush the line so that the carriage-return updates the same line.
            sys.stdout.flush()

            # Allow ROS timers, subscriptions, etc., to be processed.
            rclpy.spin_once(node, timeout_sec=0)

    finally:
        # -----------------------------------------------------------------
        # Clean shutdown - make sure the robot receives a final stop command
        # and that the terminal is restored to its original state.
        # -----------------------------------------------------------------
        node.send_command(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


# -------------------------------------------------------------------------
# Script entry point - only executed when the file is run directly.
# -------------------------------------------------------------------------
if __name__ == "__main__":
    main()
