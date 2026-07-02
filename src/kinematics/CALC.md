# SO-101 Chain Calculations

| Name | Kind | URDF type | Parent -> Child | Axis | Origin xyz | Origin rpy | Base pose at q=0 |
|---|---|---|---|---|---|---|---|
| `origin` | frame | fixed | `world` -> `base_link` | - | `0 0 0` | `0 0 0` | `(0, 0, 0)` |
| `shoulder_pan` | joint | revolute | `base_link` -> `shoulder_link` | `0 0 1` | `0.0388353 -8.97657e-09 0.0624` | `3.14159 4.18253e-17 -3.14159` | `(0.038835300, -0.000000009, 0.062400000)` |
| `shoulder_lift` | joint | revolute | `shoulder_link` -> `upper_arm_link` | `0 0 1` | `-0.0303992 -0.0182778 -0.0542` | `-1.5708 -1.5708 0` | `(0.069234500, -0.018277809, 0.116600000)` |
| `elbow_flex` | joint | revolute | `upper_arm_link` -> `lower_arm_link` | `0 0 1` | `-0.11257 -0.028 1.73763e-16` | `-3.63608e-16 8.74301e-16 1.5708` | `(0.097234500, -0.018277809, 0.229170000)` |
| `wrist_flex` | joint | revolute | `lower_arm_link` -> `wrist_link` | `0 0 1` | `-0.1349 0.0052 3.62355e-17` | `4.02456e-15 8.67362e-16 -1.5708` | `(0.232134500, -0.018277809, 0.234370000)` |
| `wrist_roll` | fixed in this chain | revolute in URDF | `wrist_link` -> `gripper_link` | `0 0 1` | `5.55112e-17 -0.0611 0.0181` | `1.5708 0.0486795 3.14159` | `(0.293234500, -0.000177809, 0.234370000)` |
| `gripper` | side branch | revolute in URDF | `gripper_link` -> `moving_jaw_so101_v1_link` | `0 0 1` | `0.0202 0.0188 -0.0234` | `1.5708 -5.24284e-08 -1.41553e-15` | not used in the requested chain |
| `gripper_frame_joint` | fixed tip frame | fixed | `gripper_link` -> `gripper_frame_link` | `0 0 0` | `-0.0079 -0.000218121 -0.0981274` | `0 3.14159 0` | `(0.391361900, -0.000011255, 0.226468745)` |

## Chain Used Here

`origin -> shoulder_pan -> shoulder_lift -> elbow_flex -> wrist_flex -> wrist_roll (fixed) -> gripper_link -> gripper_frame_joint -> gripper_frame_link`

## Calculations

URDF joint transform order used here:

`T(parent -> child) = Trans(x, y, z) * RotZ(yaw) * RotY(pitch) * RotX(roll) * RotZ(q)`

For the first motor, `shoulder_pan`:

`x = 0.0388353`, `y = -8.97657e-09`, `z = 0.0624`

`roll = pi`, `pitch = 4.18253e-17`, `yaw = -pi`

So the direct kinematics for the shoulder pan link is:

`T0_1(theta) = Trans(0.0388353, -8.97657e-09, 0.0624) * RotZ(-pi) * RotY(4.18253e-17) * RotX(pi) * RotZ(theta)`

At `theta = 0`, the shoulder link origin is:

`(0.0388353, -8.97657e-09, 0.0624)`

The later zero-pose frame positions in the table above come from chaining these same transforms with the fixed joints held at their standard configuration (`q = 0` for the last two articulated joints).
