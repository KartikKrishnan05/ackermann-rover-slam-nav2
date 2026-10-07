"""
PCA9685 Joystick Teleoperation Script.

This script interfaces a USB Joystick directly with a PCA9685 PWM driver
to control the rover. It handles raw input parsing, sensitivity mapping,
and hardware communication via a custom driver.

Dependencies:
    - pca9685_driver (local module)
    - struct
"""

import atexit
import struct
import sys

# Attempt to import the custom driver
try:
    from pca9685_driver import Device  # pyright: ignore[reportMissingImports]
except ImportError:
    print("Error: 'pca9685_driver.py' not found.")
    sys.exit(1)

# -------------------------------------------------------------------------
# Configuration Constants
# -------------------------------------------------------------------------
THROTTLE_CHANNEL = 0
STEERING_CHANNEL = 1
JOYSTICK_DEV = "/dev/input/js0"
I2C_ADDRESS = 0x42
I2C_BUS_NUMBER = 1

# -------------------------------------------------------------------------
# Calibrated PWM Values (Microseconds)
# -------------------------------------------------------------------------
NEUTRAL_US = 1530.0
FWD_START = 1640.0
FWD_MAX = 2030.0
REV_START = 1430.0
REV_MAX = 1395.0
STEER_CENTER = 1900.0

# -------------------------------------------------------------------------
# Hardware Initialization
# -------------------------------------------------------------------------
try:
    DEVICE = Device(I2C_ADDRESS, I2C_BUS_NUMBER)
    DEVICE.set_pwm_frequency(50)
    print(
        f"Connected to PCA9685 at {hex(I2C_ADDRESS)} "
        f"on Bus {I2C_BUS_NUMBER}"
    )
except (OSError, IOError) as err:
    print(f"Hardware Error: {err}")
    sys.exit(1)


def us_to_ticks(microseconds: float) -> int:
    """
    Convert microseconds to PCA9685 12-bit ticks (0-4096).
    Based on 50Hz frequency (20ms period).
    """
    return int((microseconds / 20000.0) * 4096)


def apply_sensitivity(val: float) -> float:
    """
    Apply deadzone and quadratic sensitivity to joystick input.

    Args:
        val (float): Raw input -1.0 to 1.0.

    Returns:
        float: Adjusted output.
    """
    deadzone = 0.02
    if abs(val) < deadzone:
        return 0.0

    normalized = (abs(val) - deadzone) / (1.0 - deadzone)
    # val * normalized results in a quadratic curve keeping the sign
    return val * normalized


def get_us_for_throttle(val: float) -> float:
    """
    Map standardized input (-1.0 to 1.0) to hardware microseconds.
    """
    if val == 0:
        return NEUTRAL_US

    # Forward Logic (Negative joystick input usually means Up/Forward)
    if val < 0:
        return FWD_START + (abs(val) * (FWD_MAX - FWD_START))

    # Reverse Logic
    return REV_START + (abs(val) * (REV_MAX - REV_START))


def emergency_shutdown():
    """
    Callback to reset motors to neutral on exit.
    """
    try:
        DEVICE.set_pwm(THROTTLE_CHANNEL, us_to_ticks(NEUTRAL_US))
        DEVICE.set_pwm(STEERING_CHANNEL, us_to_ticks(STEER_CENTER))
    except (OSError, IOError):
        pass


atexit.register(emergency_shutdown)


def main():
    """
    Main loop for joystick reading and PWM control.
    """
    try:
        # pylint: disable=consider-using-with
        jsdev = open(JOYSTICK_DEV, "rb")
    except FileNotFoundError:
        print(f"Joystick not found at {JOYSTICK_DEV}!")
        return

    # Track raw values for display
    raw_th = 0.0
    raw_st = 0.0
    user_th = 0.0
    user_st = 0.0

    print("--- ROVER SYSTEM ONLINE ---")
    print("Press any button to STOP | Ctrl+C to Exit")

    # Struct format: time(I), value(h), type(B), number(B)
    event_fmt = "IhBB"
    event_size = struct.calcsize(event_fmt)

    while True:
        evbuf = jsdev.read(event_size)
        if evbuf:
            _, value, ev_type, number = struct.unpack(event_fmt, evbuf)

            # Button Logic (Type 0x01)
            if (ev_type & 0x01) and value == 1:
                user_th = 0.0
                user_st = 0.0
                raw_th = 0.0
                raw_st = 0.0

            # Axis Logic (Type 0x02)
            if ev_type & 0x02:
                norm_val = value / 32767.0
                if number == 1:  # Throttle Axis
                    raw_th = norm_val
                    user_th = apply_sensitivity(norm_val)
                elif number == 2:  # Steering Axis
                    raw_st = norm_val
                    user_st = apply_sensitivity(norm_val)

            # PWM Calculations
            t_us = get_us_for_throttle(user_th)
            s_us = STEER_CENTER + (user_st * 500)

            # Send to PCA9685
            DEVICE.set_pwm(THROTTLE_CHANNEL, us_to_ticks(t_us))
            DEVICE.set_pwm(STEERING_CHANNEL, us_to_ticks(s_us))

            # Display Output (Formatted to fit < 80 cols)
            output = (
                f"JOY: T:{raw_th:5.2f} S:{raw_st:5.2f} | "
                f"OUT: {t_us:4.0f}us {s_us:4.0f}us"
            )
            print(output, end="\r")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting...")
