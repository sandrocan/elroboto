#!/usr/bin/env python3
"""
Move SO-101 shoulder_pan slowly left/right around the current position.

This script imports the direct kinematics functions from dk_single.py.
"""

import argparse
import time
from math import radians

import dk_single as dk


# Shoulder pan calibration from our measured values
SHOULDER_PAN_MIN_RAW = 719
SHOULDER_PAN_MAX_RAW = 3401

# STS/ST3215: 4096 steps per full rotation
TICKS_PER_REV = 4096.0
TICKS_PER_DEG = TICKS_PER_REV / 360.0


def deg_to_raw_delta(deg):
    """Convert relative servo degrees to relative raw encoder ticks."""
    return int(round(deg * TICKS_PER_DEG))


def check_limit(raw_value):
    """Stop if the target would leave the calibrated safe range."""
    if raw_value < SHOULDER_PAN_MIN_RAW or raw_value > SHOULDER_PAN_MAX_RAW:
        raise ValueError(
            f"Target raw value {raw_value} is outside "
            f"[{SHOULDER_PAN_MIN_RAW}, {SHOULDER_PAN_MAX_RAW}]"
        )


def open_servo_bus(port, baudrate):
    """Open the serial connection to the Feetech servo bus."""
    from scservo_sdk import PortHandler, sms_sts

    port_handler = PortHandler(port)
    servo = sms_sts(port_handler)

    if not port_handler.openPort():
        raise RuntimeError(f"Could not open port {port}")

    if not port_handler.setBaudRate(baudrate):
        port_handler.closePort()
        raise RuntimeError(f"Could not set baudrate {baudrate}")

    return port_handler, servo


def read_position(servo, motor_id):
    """Read current raw position of one servo."""
    position, comm_result, error = servo.ReadPos(motor_id)

    if comm_result != 0:
        raise RuntimeError(f"Could not read position from servo ID {motor_id}")

    return position


def move_to(servo, motor_id, target_raw, speed, acc):
    """Move one servo to a raw target position."""
    check_limit(target_raw)

    print(f"Moving ID {motor_id} to raw {target_raw}")
    servo.WritePosEx(motor_id, target_raw, speed, acc)


def wait_until_reached(servo, motor_id, target_raw, tolerance=10, timeout=10.0):
    """
    Wait until the servo is close to the target position.
    """
    start_time = time.time()

    while True:
        current_raw = read_position(servo, motor_id)
        error = target_raw - current_raw

        line = f"target={target_raw}, current={current_raw}, error={error}"
        print(line.ljust(80), end="\r", flush=True)

        if abs(error) <= tolerance:
            print()
            return True

        if time.time() - start_time > timeout:
            print()
            print("Warning: target was not reached before timeout.")
            return False

        time.sleep(0.05)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Slow left/right motion test for SO-101 shoulder_pan"
    )

    parser.add_argument(
        "--port",
        default="/dev/ttyACM1",
        help="Serial port of the servo controller board",
    )

    parser.add_argument(
        "--baudrate",
        type=int,
        default=1000000,
        help="Servo bus baudrate",
    )

    parser.add_argument(
        "--id",
        type=int,
        default=1,
        help="Servo ID of shoulder_pan",
    )

    parser.add_argument(
        "--angle-deg",
        type=float,
        default=10.0,
        help="Relative movement amplitude in degrees",
    )

    parser.add_argument(
        "--speed",
        type=int,
        default=120,
        help="Servo movement speed. Keep low for first tests.",
    )

    parser.add_argument(
        "--acc",
        type=int,
        default=10,
        help="Servo acceleration. Keep low for first tests.",
    )

    parser.add_argument(
        "--cycles",
        type=int,
        default=3,
        help="Number of left/right cycles",
    )

    parser.add_argument(
        "--pause",
        type=float,
        default=1.5,
        help="Pause between movements in seconds",
    )

    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip safety confirmation",
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="Maximum waiting time for each target movement in seconds",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    angle_rad = radians(args.angle_deg)
    delta_raw = deg_to_raw_delta(args.angle_deg)

    print("\nImported FK check from dk_single.py")
    print(f"FK for +{args.angle_deg:.1f} deg:")
    dk.print_matrix(dk.shoulder_pan_fk(angle_rad))

    print(f"\nFK for -{args.angle_deg:.1f} deg:")
    dk.print_matrix(dk.shoulder_pan_fk(-angle_rad))

    print("\nMovement settings:")
    print(f"Servo ID:      {args.id}")
    print(f"Angle:         +/- {args.angle_deg:.1f} deg")
    print(f"Raw delta:     +/- {delta_raw} ticks")
    print(f"Speed:         {args.speed}")
    print(f"Acceleration:  {args.acc}")
    print(f"Cycles:        {args.cycles}")

    if not args.yes:
        answer = input("\nType YES to move the real robot: ")
        if answer != "YES":
            print("Cancelled.")
            return

    port_handler = None

    try:
        port_handler, servo = open_servo_bus(args.port, args.baudrate)

        center_raw = read_position(servo, args.id)
        left_raw = center_raw - delta_raw
        right_raw = center_raw + delta_raw

        check_limit(left_raw)
        check_limit(right_raw)

        print("\nCurrent raw position used as center:")
        print(f"center = {center_raw}")
        print(f"left   = {left_raw}")
        print(f"right  = {right_raw}")

        for i in range(args.cycles):
            print(f"\nCycle {i + 1}/{args.cycles}")

            move_to(servo, args.id, left_raw, args.speed, args.acc)
            wait_until_reached(servo, args.id, left_raw, timeout=args.timeout)
            time.sleep(args.pause)

            move_to(servo, args.id, center_raw, args.speed, args.acc)
            wait_until_reached(servo, args.id, center_raw, timeout=args.timeout)
            time.sleep(args.pause)

            move_to(servo, args.id, right_raw, args.speed, args.acc)
            wait_until_reached(servo, args.id, right_raw, timeout=args.timeout)
            time.sleep(args.pause)

            move_to(servo, args.id, center_raw, args.speed, args.acc)
            wait_until_reached(servo, args.id, center_raw, timeout=args.timeout)
            time.sleep(args.pause)

        print("\nReturning to center position")
        move_to(servo, args.id, center_raw, args.speed, args.acc)

    finally:
        if port_handler is not None:
            port_handler.closePort()


if __name__ == "__main__":
    main()