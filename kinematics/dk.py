#!/usr/bin/env python3
"""
Direct kinematics for the SO-101 chain up to the end effector.

Chain:
base_link
 -> shoulder_link
 -> upper_arm_link
 -> lower_arm_link
 -> wrist_link
 -> gripper_link
 -> gripper_frame_link

URDF transform order:
T(parent -> child) = Trans(x, y, z) * RotZ(yaw) * RotY(pitch) * RotX(roll) * RotZ(q)

For the current robot setup, shoulder_pan, shoulder_lift, elbow_flex and
wrist_flex are treated as active joints. wrist_roll is included in the chain
with q = 0 by default, but an optional fifth angle can be passed.
"""

import argparse
from dataclasses import dataclass
from math import cos, sin, pi, radians
from typing import Dict, Iterable, List, Sequence, Tuple


Matrix4 = List[List[float]]
Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class JointSpec:
    """Description of one transform in the kinematic chain."""
    name: str
    parent: str
    child: str
    xyz: Vector3
    rpy: Vector3
    active: bool = True


# Values taken from the URDF/CALC chain.
CHAIN: Tuple[JointSpec, ...] = (
    JointSpec(
        name="shoulder_pan",
        parent="base_link",
        child="shoulder_link",
        xyz=(0.0388353, -8.97657e-09, 0.0624),
        rpy=(pi, 4.18253e-17, -pi),
        active=True,
    ),
    JointSpec(
        name="shoulder_lift",
        parent="shoulder_link",
        child="upper_arm_link",
        xyz=(-0.0303992, -0.0182778, -0.0542),
        rpy=(-1.5708, -1.5708, 0.0),
        active=True,
    ),
    JointSpec(
        name="elbow_flex",
        parent="upper_arm_link",
        child="lower_arm_link",
        xyz=(-0.11257, -0.028, 1.73763e-16),
        rpy=(-3.63608e-16, 8.74301e-16, 1.5708),
        active=True,
    ),
    JointSpec(
        name="wrist_flex",
        parent="lower_arm_link",
        child="wrist_link",
        xyz=(-0.1349, 0.0052, 3.62355e-17),
        rpy=(4.02456e-15, 8.67362e-16, -1.5708),
        active=True,
    ),
    JointSpec(
        name="wrist_roll",
        parent="wrist_link",
        child="gripper_link",
        xyz=(5.55112e-17, -0.0611, 0.0181),
        rpy=(1.5708, 0.0486795, pi),
        active=True,
    ),
    JointSpec(
        name="gripper_frame_joint",
        parent="gripper_link",
        child="gripper_frame_link",
        xyz=(-0.0079, -0.000218121, -0.0981274),
        rpy=(0.0, pi, 0.0),
        active=False,
    ),
)


def identity4() -> Matrix4:
    """Return a 4x4 identity matrix."""
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def mat_mul(a: Matrix4, b: Matrix4) -> Matrix4:
    """Multiply two 4x4 matrices."""
    out = [[0.0 for _ in range(4)] for _ in range(4)]

    for i in range(4):
        for j in range(4):
            out[i][j] = (
                a[i][0] * b[0][j]
                + a[i][1] * b[1][j]
                + a[i][2] * b[2][j]
                + a[i][3] * b[3][j]
            )

    return out


def trans(x: float, y: float, z: float) -> Matrix4:
    """Return a homogeneous translation matrix."""
    return [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rot_x(angle: float) -> Matrix4:
    """Return a homogeneous rotation matrix around the x-axis."""
    c = cos(angle)
    s = sin(angle)

    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, -s, 0.0],
        [0.0, s, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rot_y(angle: float) -> Matrix4:
    """Return a homogeneous rotation matrix around the y-axis."""
    c = cos(angle)
    s = sin(angle)

    return [
        [c, 0.0, s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s, 0.0, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rot_z(angle: float) -> Matrix4:
    """Return a homogeneous rotation matrix around the z-axis."""
    c = cos(angle)
    s = sin(angle)

    return [
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def fixed_transform(xyz: Vector3, rpy: Vector3) -> Matrix4:
    """Return Trans(xyz) * RotZ(yaw) * RotY(pitch) * RotX(roll)."""
    x, y, z = xyz
    roll, pitch, yaw = rpy

    return mat_mul(
        trans(x, y, z),
        mat_mul(
            rot_z(yaw),
            mat_mul(rot_y(pitch), rot_x(roll)),
        ),
    )


def joint_transform(joint: JointSpec, q: float = 0.0) -> Matrix4:
    """Return the full parent-to-child transform for one chain entry."""
    t_fixed = fixed_transform(joint.xyz, joint.rpy)

    if not joint.active:
        return t_fixed

    return mat_mul(t_fixed, rot_z(q))


def normalize_angles(angles: Sequence[float]) -> List[float]:
    """
    Return five joint angles.

    Input may contain:
    - 0 values: all active joints are set to zero
    - 4 values: shoulder_pan to wrist_flex, wrist_roll is set to zero
    - 5 values: shoulder_pan to wrist_roll
    """
    if len(angles) == 0:
        return [0.0, 0.0, 0.0, 0.0, 0.0]

    if len(angles) == 4:
        return [angles[0], angles[1], angles[2], angles[3], 0.0]

    if len(angles) == 5:
        return list(angles)

    raise ValueError("Expected either 0, 4, or 5 joint angles.")


def forward_kinematics(angles: Sequence[float]) -> Dict[str, Matrix4]:
    """
    Calculate direct kinematics for the complete chain.

    Args:
        angles: Joint angles in radians. Pass 4 angles for the active arm joints
                or 5 angles if wrist_roll should also be included.

    Returns:
        Dictionary mapping each child link name to its base_link transform.
        The final end-effector transform is stored as "gripper_frame_link".
    """
    q_values = normalize_angles(angles)
    q_index = 0
    current = identity4()
    frames: Dict[str, Matrix4] = {"base_link": current}

    for joint in CHAIN:
        q = 0.0

        if joint.active:
            q = q_values[q_index]
            q_index += 1

        current = mat_mul(current, joint_transform(joint, q))
        frames[joint.child] = current

    return frames


def end_effector_transform(angles: Sequence[float]) -> Matrix4:
    """Return the base_link to gripper_frame_link transform."""
    return forward_kinematics(angles)["gripper_frame_link"]


def position_from_transform(t: Matrix4) -> Vector3:
    """Extract xyz position from a homogeneous transform."""
    return (t[0][3], t[1][3], t[2][3])


def end_effector_position(angles: Sequence[float]) -> Vector3:
    """Return the end-effector xyz position in base_link coordinates."""
    return position_from_transform(end_effector_transform(angles))


def print_matrix(matrix: Matrix4) -> None:
    """Print a 4x4 matrix with fixed precision."""
    for row in matrix:
        print("[" + ", ".join(f"{value: .12f}" for value in row) + "]")


def print_position(label: str, matrix: Matrix4) -> None:
    """Print the xyz position of a transform."""
    x, y, z = position_from_transform(matrix)
    print(f"{label:18s} x={x: .9f}  y={y: .9f}  z={z: .9f}")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Direct kinematics for the SO-101 arm up to gripper_frame_link."
    )

    parser.add_argument(
        "angles",
        nargs="*",
        type=float,
        help=(
            "Joint angles. Use 4 values for shoulder_pan, shoulder_lift, "
            "elbow_flex, wrist_flex; optionally add a 5th value for wrist_roll."
        ),
    )

    parser.add_argument(
        "--deg",
        action="store_true",
        help="Interpret input angles as degrees instead of radians.",
    )

    parser.add_argument(
        "--frames",
        action="store_true",
        help="Print positions of all intermediate frames.",
    )

    parser.add_argument(
        "--matrix",
        action="store_true",
        help="Print the final 4x4 end-effector transform matrix.",
    )

    return parser.parse_args()


def main() -> None:
    """Run direct kinematics from command-line arguments."""
    args = parse_args()

    angles = args.angles

    if args.deg:
        angles = [radians(value) for value in angles]

    try:
        normalized_angles = normalize_angles(angles)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    frames = forward_kinematics(normalized_angles)
    end_effector = frames["gripper_frame_link"]

    print("SO-101 direct kinematics")
    print("Angles used [rad]:")
    print(
        "  shoulder_pan  = "
        f"{normalized_angles[0]: .9f}\n"
        "  shoulder_lift = "
        f"{normalized_angles[1]: .9f}\n"
        "  elbow_flex    = "
        f"{normalized_angles[2]: .9f}\n"
        "  wrist_flex    = "
        f"{normalized_angles[3]: .9f}\n"
        "  wrist_roll    = "
        f"{normalized_angles[4]: .9f}"
    )

    print()
    print_position("end_effector", end_effector)

    if args.frames:
        print("\nFrame positions:")
        for name, transform in frames.items():
            print_position(name, transform)

    if args.matrix:
        print("\nT_base_to_gripper_frame:")
        print_matrix(end_effector)


if __name__ == "__main__":
    main()
