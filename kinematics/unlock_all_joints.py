#!/usr/bin/env python3
"""
Disable torque on all SO-101 servos so the joints can be moved by hand.
"""

import argparse
import time


ADDR_TORQUE_ENABLE = 40  # STS/ST3215 torque enable address


def open_servo_bus(port, baudrate):
    from scservo_sdk import PortHandler, sms_sts

    port_handler = PortHandler(port)
    servo = sms_sts(port_handler)

    if not port_handler.openPort():
        raise RuntimeError(f"Could not open port {port}")

    if not port_handler.setBaudRate(baudrate):
        port_handler.closePort()
        raise RuntimeError(f"Could not set baudrate {baudrate}")

    return port_handler, servo


def set_torque(servo, motor_id, enable):
    value = 1 if enable else 0

    result = servo.write1ByteTxRx(
        motor_id,
        ADDR_TORQUE_ENABLE,
        value,
    )

    print(f"ID {motor_id}: torque {'ON' if enable else 'OFF'} -> {result}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Enable or disable torque for all SO-101 joints"
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
        "--ids",
        type=int,
        nargs="+",
        default=[1, 2, 3, 4, 5, 6],
        help="Servo IDs to unlock",
    )

    parser.add_argument(
        "--enable",
        action="store_true",
        help="Enable torque instead of disabling it",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    port_handler = None

    try:
        port_handler, servo = open_servo_bus(args.port, args.baudrate)

        print("\nSetting torque:")
        for motor_id in args.ids:
            set_torque(servo, motor_id, args.enable)
            time.sleep(0.05)

        print("\nDone.")

    finally:
        if port_handler is not None:
            port_handler.closePort()


if __name__ == "__main__":
    main()