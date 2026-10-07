"""
Calibrated Rover Teleoperation Script.

This script provides a terminal-based interface (using curses) to control
the rover's throttle and steering directly via the Adafruit ServoKit library.
It maps user inputs (-1.0 to 1.0) to specific, calibrated PWM pulse widths
required by the hardware electronic speed controller (ESC).

Hardware Calibration (Microseconds):
  - Neutral: 1530us
  - Forward: 1640us (Start) to 2030us (Max)
  - Reverse: 1430us (Start) to 1395us (Max)

Dependencies:
  - adafruit-circuitpython-servokit
  - board
  - busio
  - curses
"""
# pylint: disable=import-error
# pyright: reportMissingImports=false

import atexit
import curses
import time

import board
import busio
from adafruit_servokit import ServoKit

# -------------------------------------------------------------------------
# Configuration Constants
# -------------------------------------------------------------------------
THROTTLE_CHANNEL = 0
STEERING_CHANNEL = 1

THROTTLE_STEP = 0.05
STEERING_STEP = 0.1

# -------------------------------------------------------------------------
# Calibrated Hardware Values (Pulse Widths in Microseconds)
# -------------------------------------------------------------------------
NEUTRAL_PWM = 1530.0

# Forward Range
FWD_START = 1640.0
FWD_MAX = 2030.0

# Reverse Range ("Deadzone" / Non-linear mapping)
REV_START = 1430.0
REV_MAX = 1395.0  # Note: Max reverse PWM is often lower than start

# Library Initialization
# Try initializing I2C on specific pins first (Jetson/Pi specific),
# fall back to default
try:
    # Attempt specific I2C bus (often required for Jetson Nano)
    I2C_BUS = busio.I2C(board.SCL_1, board.SDA_1)  # type: ignore
except AttributeError:
    # Fallback to standard board definition
    I2C_BUS = busio.I2C(board.SCL, board.SDA)

KIT = ServoKit(channels=16, i2c=I2C_BUS, address=0x42)

# Set the pulse width range for the Continuous Servo
# This establishes the theoretical min/max the library uses for normalization
KIT.continuous_servo[THROTTLE_CHANNEL].set_pulse_width_range(1030, 2030)


def get_pwm_for_throttle(val: float) -> float:
    """
    Map a standardized input (-1.0 to 1.0) to
    hardware-specific PWM microseconds.

    Args:
        val (float): The virtual throttle value from -1.0 (full reverse)
                     to 1.0 (full forward).

    Returns:
        float: The target pulse width in microseconds.
    """
    if abs(val) < 0.01:
        return NEUTRAL_PWM

    if val > 0:
        # Map 0.0..1.0 -> FWD_START..FWD_MAX
        return FWD_START + (val * (FWD_MAX - FWD_START))

    # Map 0.0..-1.0 -> REV_START..REV_MAX
    # Note: We use abs(val) because the calculation adds the offset
    return REV_START + (abs(val) * (REV_MAX - REV_START))


def emergency_shutdown():
    """
    Safety callback to stop the motors on script exit.
    """
    KIT.continuous_servo[THROTTLE_CHANNEL].throttle = 0.0


# Register the shutdown function immediately
atexit.register(emergency_shutdown)


def main(stdscr):
    """
    Main control loop using curses for keyboard input.

    Args:
        stdscr: The curses standard screen object.
    """
    # Non-blocking input
    stdscr.nodelay(True)

    user_throttle = 0.0  # Virtual throttle: -1.0 to 1.0
    user_steering = 0.0  # Virtual steering: -1.0 to 1.0

    while True:
        # Fetch key press
        key = stdscr.getch()

        # Handle Controls
        if key == curses.KEY_UP:
            user_throttle += THROTTLE_STEP
        elif key == curses.KEY_DOWN:
            user_throttle -= THROTTLE_STEP
        elif key == curses.KEY_LEFT:
            user_steering += STEERING_STEP
        elif key == curses.KEY_RIGHT:
            user_steering -= STEERING_STEP
        elif key == ord(" "):
            # Spacebar: Emergency Brake / Reset
            user_throttle = 0.0
            user_steering = 0.0
        elif key == ord("q"):
            # Quit
            break

        # Clamp virtual values to safe range -1.0 to 1.0
        user_throttle = max(min(user_throttle, 1.0), -1.0)
        user_steering = max(min(user_steering, 1.0), -1.0)

        # -----------------------------------------------------------------
        # Logic: Convert Virtual Throttle -> Hardware PWM -> Library Throttle
        # -----------------------------------------------------------------
        # 1. Get exact microseconds required for the hardware ESC
        target_pwm = get_pwm_for_throttle(user_throttle)

        # 2. Convert specific PWM back to the library's normalized -1..1 scale.
        #    The library uses (pulse - center) / range_scale logic internally.
        #    Here we reverse-engineer the "kit_value" so the library
        #       outputs exactly "target_pwm".
        #    Base equation: Pulse = 1530 + (Throttle * 500)
        #       [Assuming 1030-2030 range]
        #    So: Throttle = (Pulse - 1530) / 500
        kit_value = (target_pwm - 1530) / 500.0

        # 3. Send to hardware
        KIT.continuous_servo[THROTTLE_CHANNEL].throttle = kit_value

        # -----------------------------------------------------------------
        # Visual Feedback (Curses UI)
        # -----------------------------------------------------------------
        stdscr.clear()
        stdscr.addstr(0, 0, "=== CALIBRATED ROVER CONTROL ===")
        stdscr.addstr(2, 0, f"User Throttle: {user_throttle:5.2f}")
        stdscr.addstr(3, 0, f"Sent PWM:      {target_pwm:5.0f} us")
        stdscr.addstr(4, 0, f"Lib Value:     {kit_value:5.2f}")
        stdscr.addstr(
            6,
            0,
            f"FWD Zone: {FWD_START:.0f}-{FWD_MAX:.0f} | "
            f"REV Zone: {REV_START:.0f}-{REV_MAX:.0f}",
        )
        stdscr.addstr(
            8,
            0,
            "Controls: Arrows to move, Space to Stop, 'q' to Quit"
        )
        stdscr.refresh()

        time.sleep(0.05)


if __name__ == "__main__":
    curses.wrapper(main)
