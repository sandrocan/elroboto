#!/usr/bin/env python3
"""
Move SO-101 joints 1 to 4 slowly around their current positions.

This is the multi-joint version of move_dk_single.py.
It uses dk.py only as a forward-kinematics check/printout. The real robot
movement is still done with raw servo ticks through the Feetech servo bus.

Active test joints:
    ID 1 = shoulder_pan
    ID 2 = shoulder_lift
    ID 3 = elbow_flex
    ID 4 = wrist_flex

Joint 5 and 6 are intentionally not moved because they are treated as fixed
in the current chain.
"""

import argparse
import time
from dataclasses import dataclass
from math import radians
from typing import Dict, Iterable, List, Sequence, Tuple

import dk


# STS/ST3215: 4096 steps per full rotation.
TICKS_PER_REV = 4096.0
TICKS_PER_DEG = TICKS_PER_REV / 360.0


@dataclass(frozen=True)
class JointLimit:
    """Servo ID, readable joint name, and calibrated raw safety limits."""
    motor_id: int
    name: str
    min_raw: int
    home_raw: int
    max_raw: int


# Safety limits from the current servo joint table.
# Joints 5 and 6 are intentionally not included here.
JOINTS: Tuple[JointLimit, ...] = (
    JointLimit(1, "shoulder_pan", 729, 2047, 3391),
    JointLimit(2, "shoulder_lift", 787, 2047, 3308),
    JointLimit(3, "elbow_flex", 900, 2047, 2998),
    JointLimit(4, "wrist_flex", 933, 2047, 3228),
)


def deg_to_raw_delta(deg: float) -> int:
    """Convert relative servo degrees to relative raw encoder ticks."""
    return int(round(deg * TICKS_PER_DEG))


def check_limit(joint: JointLimit, raw_value: int) -> None:
    """Stop if a target would leave the calibrated safe range."""
    if raw_value < joint.min_raw or raw_value > joint.max_raw:
        raise ValueError(
            f"Target raw value {raw_value} for {joint.name} ID {joint.motor_id} "
            f"is outside [{joint.min_raw}, {joint.max_raw}]"
        )


def check_all_limits(targets: Dict[int, int]) -> None:
    """Validate all target raw positions against their joint limits."""
    for joint in JOINTS:
        check_limit(joint, targets[joint.motor_id])


def open_servo_bus(port: str, baudrate: int):
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


def read_position(servo, motor_id: int) -> int:
    """Read the current raw position of one servo."""
    position, comm_result, error = servo.ReadPos(motor_id)

    if comm_result != 0:
        raise RuntimeError(f"Could not read position from servo ID {motor_id}")

    if error != 0:
        raise RuntimeError(f"Servo ID {motor_id} returned error {error}")

    return position


def read_all_positions(servo) -> Dict[int, int]:
    """Read current raw positions of all active test joints."""
    return {joint.motor_id: read_position(servo, joint.motor_id) for joint in JOINTS}


def move_to(servo, joint: JointLimit, target_raw: int, speed: int, acc: int) -> None:
    """Move one servo to a raw target position."""
    check_limit(joint, target_raw)
    print(f"Moving {joint.name:13s} ID {joint.motor_id} to raw {target_raw}")
    servo.WritePosEx(joint.motor_id, target_raw, speed, acc)


def move_all_to(servo, targets: Dict[int, int], speed: int, acc: int) -> None:
    """Send target positions to all active test joints."""
    check_all_limits(targets)

    for joint in JOINTS:
        move_to(servo, joint, targets[joint.motor_id], speed, acc)


def wait_until_reached(
    servo,
    targets: Dict[int, int],
    tolerance: int = 10,
    timeout: float = 10.0,
    poll_interval: float = 0.05,
) -> bool:
    """Wait until all active joints are close to their target positions."""
    start_time = time.time()

    while True:
        currents = read_all_positions(servo)
        errors = {
            motor_id: targets[motor_id] - currents[motor_id]
            for motor_id in targets
        }

        parts = []
        all_reached = True

        for joint in JOINTS:
            motor_id = joint.motor_id
            error = errors[motor_id]
            parts.append(
                f"{joint.name}=target:{targets[motor_id]} "
                f"current:{currents[motor_id]} error:{error:+d}"
            )

            if abs(error) > tolerance:
                all_reached = False

        print(" | ".join(parts).ljust(150), end="\r", flush=True)

        if all_reached:
            print()
            return True

        if time.time() - start_time > timeout:
            print()
            print("Warning: at least one joint did not reach the target before timeout.")
            return False

        time.sleep(poll_interval)


def print_fk(label: str, relative_angles_deg: Sequence[float]) -> None:
    """Print the direct-kinematics end-effector position for relative angles."""
    angles_rad = [radians(value) for value in relative_angles_deg]
    x, y, z = dk.end_effector_position(angles_rad)

    angle_text = ", ".join(
        f"{joint.name}={angle:+.1f}deg"
        for joint, angle in zip(JOINTS, relative_angles_deg)
    )

    print(f"{label}: {angle_text}")
    print(f"  FK end effector: x={x:.9f}, y={y:.9f}, z={z:.9f}")


def target_from_relative_angles(
    center_raw: Dict[int, int],
    relative_angles_deg: Sequence[float],
) -> Dict[int, int]:
    """Create raw target positions from relative joint angles around center."""
    targets: Dict[int, int] = {}

    for joint, angle_deg in zip(JOINTS, relative_angles_deg):
        targets[joint.motor_id] = center_raw[joint.motor_id] + deg_to_raw_delta(angle_deg)

    return targets


def run_axis_by_axis_test(servo, args, center_raw: Dict[int, int]) -> None:
    """Move each joint left/center/right around the current position."""
    for cycle in range(args.cycles):
        print(f"\nCycle {cycle + 1}/{args.cycles} - axis-by-axis")

        for joint_index, joint in enumerate(JOINTS):
            print(f"\nTesting {joint.name} ID {joint.motor_id}")

            for target_name, signed_angle in (
                ("negative", -args.angle_deg),
                ("center", 0.0),
                ("positive", args.angle_deg),
                ("center", 0.0),
            ):
                relative_angles = [0.0, 0.0, 0.0, 0.0]
                relative_angles[joint_index] = signed_angle

                targets = dict(center_raw)
                targets[joint.motor_id] = (
                    center_raw[joint.motor_id] + deg_to_raw_delta(signed_angle)
                )

                print_fk(target_name, relative_angles)
                move_to(servo, joint, targets[joint.motor_id], args.speed, args.acc)
                wait_until_reached(
                    servo,
                    targets,
                    tolerance=args.tolerance,
                    timeout=args.timeout,
                    poll_interval=args.poll_interval,
                )
                time.sleep(args.pause)


def run_all_at_once_test(servo, args, center_raw: Dict[int, int]) -> None:
    """Move all active joints together around the current position."""
    for cycle in range(args.cycles):
        print(f"\nCycle {cycle + 1}/{args.cycles} - all-at-once")

        for target_name, signed_angle in (
            ("negative", -args.angle_deg),
            ("center", 0.0),
            ("positive", args.angle_deg),
            ("center", 0.0),
        ):
            relative_angles = [signed_angle, signed_angle, signed_angle, signed_angle]
            targets = target_from_relative_angles(center_raw, relative_angles)

            print_fk(target_name, relative_angles)
            move_all_to(servo, targets, args.speed, args.acc)
            wait_until_reached(
                servo,
                targets,
                tolerance=args.tolerance,
                timeout=args.timeout,
                poll_interval=args.poll_interval,
            )
            time.sleep(args.pause)


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Slow +/- angle movement test for SO-101 joints 1 to 4."
    )

    parser.add_argument(
        "--port",
        default="/dev/ttyACM1",
        help="Serial port of the servo controller board.",
    )

    parser.add_argument(
        "--baudrate",
        type=int,
        default=1000000,
        help="Servo bus baudrate.",
    )

    parser.add_argument(
        "--angle-deg",
        type=float,
        default=10.0,
        help="Relative movement amplitude in degrees.",
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
        default=1,
        help="Number of test cycles.",
    )

    parser.add_argument(
        "--pause",
        type=float,
        default=1.0,
        help="Pause between movements in seconds.",
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="Maximum waiting time for each target movement in seconds.",
    )

    parser.add_argument(
        "--tolerance",
        type=int,
        default=10,
        help="Allowed raw tick error before a joint counts as reached.",
    )

    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.05,
        help="Polling interval while waiting for a target position.",
    )

    parser.add_argument(
        "--all-at-once",
        action="store_true",
        help="Move joints 1 to 4 together instead of testing one axis at a time.",
    )

    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip safety confirmation.",
    )

    return parser.parse_args()


def main() -> None:
    """Run the multi-joint movement test."""
    args = parse_args()
    delta_raw = deg_to_raw_delta(args.angle_deg)

    print("\nSO-101 direct-kinematics movement test")
    print("Active joints:")
    for joint in JOINTS:
        print(
            f"  ID {joint.motor_id}: {joint.name:13s} "
            f"limits=[{joint.min_raw}, {joint.max_raw}] home={joint.home_raw}"
        )

    print("\nMovement settings:")
    print(f"Angle:         +/- {args.angle_deg:.1f} deg")
    print(f"Raw delta:     +/- {delta_raw} ticks")
    print(f"Speed:         {args.speed}")
    print(f"Acceleration:  {args.acc}")
    print(f"Cycles:        {args.cycles}")
    print(f"Mode:          {'all-at-once' if args.all_at_once else 'axis-by-axis'}")

    print("\nFK check around current raw position treated as q=0:")
    print_fk("zero", [0.0, 0.0, 0.0, 0.0])

    if not args.yes:
        answer = input("\nType YES to move the real robot: ")
        if answer != "YES":
            print("Cancelled.")
            return

    port_handler = None

    try:
        port_handler, servo = open_servo_bus(args.port, args.baudrate)

        center_raw = read_all_positions(servo)

        print("\nCurrent raw positions used as center:")
        for joint in JOINTS:
            print(f"  {joint.name:13s} ID {joint.motor_id}: center={center_raw[joint.motor_id]}")

        # Validate the complete +/- envelope before moving anything.
        for signed_angle in (-args.angle_deg, args.angle_deg):
            test_targets = target_from_relative_angles(
                center_raw,
                [signed_angle, signed_angle, signed_angle, signed_angle],
            )
            check_all_limits(test_targets)

        if args.all_at_once:
            run_all_at_once_test(servo, args, center_raw)
        else:
            run_axis_by_axis_test(servo, args, center_raw)

        print("\nReturning all active joints to center positions")
        move_all_to(servo, center_raw, args.speed, args.acc)
        wait_until_reached(
            servo,
            center_raw,
            tolerance=args.tolerance,
            timeout=args.timeout,
            poll_interval=args.poll_interval,
        )

    finally:
        if port_handler is not None:
            port_handler.closePort()


if __name__ == "__main__":
    main()
