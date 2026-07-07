#!/usr/bin/env python3
"""
Direct kinematics for the SO-101 shoulder pan joint - Only shoulder_pan (1st joint)
"""

from math import cos, sin, pi
import sys

# Translation from base_link to shoulder_link (URDF)
X = 0.0388353
Y = -8.97657e-09
Z = 0.0624

# Orientation change of the coordinate system form base_link to shoulder_link (URDF)
ROLL = pi
PITCH = 4.18253e-17
YAW = -pi

# Multiply 3x3 matrices
def mat_mul(a, b):
    out = [[0.0, 0.0, 0.0] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = (
                a[i][0] * b[0][j]
                + a[i][1] * b[1][j]
                + a[i][2] * b[2][j]
            )
    return out


def rot_x(a):
    c = cos(a)
    s = sin(a)
    return [
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c],
    ]


def rot_y(a):
    c = cos(a)
    s = sin(a)
    return [
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c],
    ]


def rot_z(a):
    c = cos(a)
    s = sin(a)
    return [
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ]


# Direct kinematics for first joint
def shoulder_pan_fk(theta):
    """Return the 4x4 transform from base_link to shoulder_link."""
    fixed = mat_mul(rot_z(YAW), mat_mul(rot_y(PITCH), rot_x(ROLL)))
    joint = rot_z(theta)
    rot = mat_mul(fixed, joint)

    return [
        [rot[0][0], rot[0][1], rot[0][2], X],
        [rot[1][0], rot[1][1], rot[1][2], Y],
        [rot[2][0], rot[2][1], rot[2][2], Z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def print_matrix(m):
    for row in m:
        print("[" + ", ".join(f"{v:.12f}" for v in row) + "]")


def main(argv):
    theta = 0.0
    if len(argv) > 1:
        theta = float(argv[1])

    t = shoulder_pan_fk(theta)
    print_matrix(t)


if __name__ == "__main__":
    main(sys.argv)
