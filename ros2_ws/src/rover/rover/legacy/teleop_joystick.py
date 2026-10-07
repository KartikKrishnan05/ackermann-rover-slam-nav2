"""
Joystick Teleoperation Script for Rover.

This script reads raw input events from a connected gamepad/joystick
(e.g., /dev/input/js0) and maps them to the rover's throttle and steering
channels via the Adafruit ServoKit library.

It features:
  - Asymmetric sensitivity tuning (exponential curves).
  - Deadzone handling.
  - Specific PWM calibration for the rover's ESC.
  - Emergency stop functionality via button press or keyboard interrupt.

Dependencies:
  - adafruit-circuitpython-servokit
  - board
  - busio
"""
# pylint: disable=import-error
# pyright: reportMissingImports=false

import atexit
import struct
import sys

import board
import busio
from adafruit_servokit import ServoKit

# -------------------------------------------------------------------------
# Configuration Constants
# -------------------------------------------------------------------------
THROTTLE_CHANNEL = 0
STEERING_CHANNEL = 1
JOYSTICK_DEV = "/dev/input/js0"

# -------------------------------------------------------------------------
# Sensitivity Tuning
# -------------------------------------------------------------------------
JOY_DEADZONE = 0.02
FWD_EXPO = 1.0  # Exponential curve for forward throttle (1.0 = linear)
REV_EXPO = 1.0  # Exponential curve for reverse throttle

# -------------------------------------------------------------------------
# Calibrated Hardware Values (Pulse Widths in Microseconds)
# -------------------------------------------------------------------------
NEUTRAL_PWM = 1530.0

# Forward Range
FWD_START = 1640.0
FWD_MAX = 2030.0

# Reverse Range
REV_START = 1430.0
REV_MAX = 1395.0

# Steering Configuration
STEER_CENTER = 1900.0  # Assumed center pulse width for steering servo

# -------------------------------------------------------------------------
# Hardware Initialization
# -------------------------------------------------------------------------
try:
    # Attempt specific I2C bus (often required for Jetson Nano)
    I2C_BUS = busio.I2C(board.SCL_1, board.SDA_1)  # type: ignore
except AttributeError:
    # Fallback to standard board definition (Raspberry Pi, etc.)
    I2C_BUS = busio.I2C(board.SCL, board.SDA)

KIT = ServoKit(channels=16, i2c=I2C_BUS, address=0x42)

# Configure Pulse Width Ranges
# Throttle: 1030us to 2030us (Wide range to accommodate calibration)
KIT.continuous_servo[THROTTLE_CHANNEL].set_pulse_width_range(1030, 2030)
# Steering: 1400us to 2400us
KIT.continuous_servo[STEERING_CHANNEL].set_pulse_width_range(1400, 2400)


def apply_asymmetric_sensitivity(val: float) -> float:
    """
    Apply deadzone and exponential sensitivity curves to raw joystick input.

    Args:
        val (float): Raw input value from -1.0 to 1.0.

    Returns:
        float: Processed control value.
    """
    if abs(val) < JOY_DEADZONE:
        return 0.0

    # Normalize value outside of the deadzone (0.0 to 1.0)
    normalized = (abs(val) - JOY_DEADZONE) / (1.0 - JOY_DEADZONE)

    if val < 0:
        # Forward Logic (Negative raw input is usually 'Forward' on joysticks)
        return -1.0 * (normalized**FWD_EXPO)

    # Reverse Logic
    return 1.0 * (normalized**REV_EXPO)


def get_pwm_for_throttle(val: float) -> float:
    """
    Map processed throttle value (-1.0 to 1.0) to
    hardware-specific PWM microseconds.

    Args:
        val (float): Processed throttle value.

    Returns:
        float: Target pulse width in microseconds.
    """
    if val == 0:
        return NEUTRAL_PWM

    ratio = abs(val)

    # Joystick Axis 1 (Vertical) usually sends negative values for UP (Forward)
    if val < 0:
        return FWD_START + (ratio * (FWD_MAX - FWD_START))

    return REV_START + (ratio * (REV_MAX - REV_START))


def emergency_shutdown():
    """
    Safety callback to halt motors immediately.
    """
    print("\n[!] EMERGENCY SHUTDOWN TRIGGERED")
    try:
        KIT.continuous_servo[THROTTLE_CHANNEL].throttle = 0.0
        KIT.continuous_servo[STEERING_CHANNEL].throttle = 0.0
    except OSError:
        # Handle cases where I2C might already be disconnected
        pass


atexit.register(emergency_shutdown)


def main():
    """
    Main loop reading joystick events and updating servo outputs.
    """
    try:
        # Open joystick device in binary read mode
        # pylint: disable=consider-using-with
        jsdev = open(JOYSTICK_DEV, "rb")
    except FileNotFoundError:
        print(f"Joystick not found at {JOYSTICK_DEV}!")
        sys.exit(1)

    print("ULTRA SENSITIVE MODE ACTIVE")
    print("Press ANY BUTTON to Emergency Stop | Ctrl+C to Exit")

    user_throttle = 0.0
    user_steering = 0.0

    # Event buffer size (struct js_event is 8 bytes)
    event_size = 8

    while True:
        evbuf = jsdev.read(event_size)
        if evbuf:
            # Unpack struct: time(I), value(h), type(B), number(B)
            # h = short (2 bytes), B = unsigned char (1 byte)
            _, value, event_type, number = struct.unpack("IhBB", evbuf)

            # -----------------------------------------------------------------
            # Button Logic (Emergency Stop)
            # -----------------------------------------------------------------
            # event_type 0x01 is a button. value 1 is 'pressed'.
            if (event_type & 0x01) and value == 1:
                user_throttle = 0.0
                user_steering = 0.0
                print("\n[STOP] Button Pressed - Motors Halted!")

            # -----------------------------------------------------------------
            # Axis Logic
            # -----------------------------------------------------------------
            # event_type 0x02 is an axis.
            if event_type & 0x02:
                raw_val = value / 32767.0

                if number == 1:  # Axis 1: Throttle (Vertical)
                    user_throttle = apply_asymmetric_sensitivity(raw_val)
                elif number == 2:  # Axis 2: Steering (Horizontal)
                    user_steering = apply_asymmetric_sensitivity(raw_val)

            # -----------------------------------------------------------------
            # PWM Calculations & Hardware Output
            # -----------------------------------------------------------------
            target_t_pwm = get_pwm_for_throttle(user_throttle)

            # Steering PWM: Map -1..1 to Center +/- 500ms
            # Note: This is purely for display; we send raw steering
            target_s_pwm = STEER_CENTER + (user_steering * 500)

            # Send Throttle
            # Convert target microseconds back to the -1..1 scale expected
            # by the library based on the 1530us center (NEUTRAL_PWM)
            # and 500us scale divisor.
            kit_throttle_val = (target_t_pwm - 1530) / 500.0
            KIT.continuous_servo[THROTTLE_CHANNEL].throttle = kit_throttle_val

            # Send Steering
            # Assuming the library's range (1400-2400) maps linear input
            KIT.continuous_servo[STEERING_CHANNEL].throttle = user_steering

            # -----------------------------------------------------------------
            # Feedback Display
            # -----------------------------------------------------------------
            display = (
                f"THR: {user_throttle:5.2f} ({target_t_pwm:4.0f}ms) | "
                f"STR: {user_steering:5.2f} ({target_s_pwm:4.0f}ms)   "
            )
            print(display, end="\r")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # Exit cleanly on Ctrl+C
        pass
