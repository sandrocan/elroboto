# SO-101 Kinematics and Application Flow

## 0. General Introduction

### 0.1 System Scope

| Parameter | Current Implementation |
|---|---|
| Robot | SO-101 follower arm |
| Target platform | STM32U5 Nucleo |
| Servo type | Feetech serial bus servos |
| Servo bus | UART, `1,000,000 baud` |
| Debug output | `printf`, routed to COM1 UART at `115,200 baud` |
| Position representation | Unsigned encoder ticks |
| Encoder domain | `0 ... 4095` ticks |
| Encoder resolution | `4096 ticks/revolution` |
| Angular resolution | `0.087890625 deg/tick` |
| Tick resolution | `11.377777... ticks/deg` |
| Active kinematic joints | J1–J4 |
| Fixed joints | J5 `wrist_roll`, J6 `gripper` |
| TCP frame | `gripper_frame_link` |
| Kinematic model | URDF-derived homogeneous transformation chain |
| IK target | Cartesian TCP position `(x, y, z)` |
| IK orientation target | Not implemented |
| Runtime trajectory | Four corners of a square in the `y-z` plane |

### 0.2 Module Responsibilities

| Module | Responsibility |
|---|---|
| `app.c/.h` | Startup sequence, square trajectory, application state, button abort, telemetry output |
| `kinematics.c/.h` | Tick/angle conversion, FK, numerical IK, motion helpers, controlled motion |
| `operations.c/.h` | Homogeneous matrix operations, local link transforms, rounding, `3x3` inversion |
| `control.c/.h` | One PID controller per active servo joint |
| `servo.c/.h` | Joint configuration, servo packet generation, position reads/writes, torque control, homing |
| `uart.c/.h` | UART handle attachment, byte transmission/reception, RX cleanup |
| `tests.c/.h` | Standalone home, direct-kinematics, and inverse-kinematics tests |

### 0.3 Runtime Data Flow

```text
Cartesian target [m]
        |
        v
Numerical IK using URDF FK
        |
        v
Joint angles J1-J4 [deg]
        |
        v
Raw servo targets J1-J4 [ticks]
        |
        v
Per-joint PID position loop
        |
        v
Feetech UART commands
        |
        v
Measured servo positions [ticks]
```

### 0.4 Runtime Constraints

| Constraint | Enforcement |
|---|---|
| Only J1–J4 may be moved through the public position API | `Servo_WritePosition()` rejects fixed joints |
| Raw targets must remain inside calibrated tick limits | Checked before every public servo position command |
| IK joint angles must remain inside calibrated ranges | Solver clamps every iteration to tick-derived angular limits |
| IK termination is position-only | Cartesian distance threshold |
| Hardware motion may be aborted with B1 | Abort callback checked inside blocking wait/control loops |
| J5 and J6 are excluded from IK | Fixed in the FK chain or omitted from the TCP branch |

---

## 1. Tick Limits from Calibration

### 1.1 Active Follower Configuration

The active table is loaded by `Servo_Init()`.

| Joint | Servo ID | Name | Min Tick | Home / Fixed Tick | Max Tick | Firmware Usage |
|---|---:|---|---:|---:|---:|---|
| J1 | 1 | `shoulder_pan` | 729 | 2047 | 3391 | Active |
| J2 | 2 | `shoulder_lift` | 787 | 2047 | 3308 | Active |
| J3 | 3 | `elbow_flex` | 900 | 2047 | 2998 | Active |
| J4 | 4 | `wrist_flex` | 933 | 2047 | 3228 | Active |
| J5 | 5 | `wrist_roll` | 2000 | 2047 | 2200 | Fixed |
| J6 | 6 | `gripper` | 810 | 810 | 2200 | Fixed / closed |

### 1.2 Tick-to-Angle Mapping

All active joint direction multipliers are currently `+1`.

```text
angle_deg = (raw_tick - home_tick) * direction * (360 / 4096)

raw_tick = home_tick
         + round(angle_deg * (4096 / 360)) * direction
```

| Constant | Value |
|---|---:|
| `KINEMATICS_TICKS_PER_REV` | `4096.0` |
| `KINEMATICS_DEG_PER_REV` | `360.0` |
| `KINEMATICS_TICKS_PER_DEG` | `11.377777...` |
| `KINEMATICS_DEG_PER_TICK` | `0.087890625` |
| J1 direction | `+1` |
| J2 direction | `+1` |
| J3 direction | `+1` |
| J4 direction | `+1` |

### 1.3 Tick-Derived Active Joint Limits

These are the limits actually used by the numerical IK solver.

| Joint | Minimum Angle [deg] | Home Angle [deg] | Maximum Angle [deg] |
|---|---:|---:|---:|
| J1 `shoulder_pan` | `-115.839844` | `0.0` | `118.125000` |
| J2 `shoulder_lift` | `-110.742188` | `0.0` | `110.830078` |
| J3 `elbow_flex` | `-100.810547` | `0.0` | `83.583984` |
| J4 `wrist_flex` | `-97.910156` | `0.0` | `103.798828` |

### 1.4 Fixed Joint Values

| Joint | Fixed Tick | Kinematic Treatment |
|---|---:|---|
| J5 `wrist_roll` | `2047` | Included in the TCP chain with `q = 0` |
| J6 `gripper` | `810` | Not part of the TCP chain; jaw is a side branch |

`Servo_WritePosition()` rejects both joints because `is_fixed = 1`.
`Servo_DriveHome()` bypasses this public restriction internally and commands all six configured joints to their home values.

### 1.5 Application Start Position

The application uses a dedicated start pose rather than the all-joint home pose.

| Joint | Start Tick | Start Angle [deg] | Inside Calibrated Range |
|---|---:|---:|---|
| J1 | 2047 | `0.000000` | Yes |
| J2 | 1208 | `-73.740234` | Yes |
| J3 | 2548 | `44.033203` | Yes |
| J4 | 2372 | `28.564453` | Yes |

### 1.6 Servo-Side Validation

| Function | Validation Behavior |
|---|---|
| `Servo_GetJointConfigById()` | Rejects unknown/uninitialized joint configuration |
| `Servo_WritePosition()` | Rejects fixed joints and out-of-range ticks |
| `Kinematics_AngleDegToRaw()` | Rejects fixed joints and angles mapping outside tick limits |
| `Kinematics_CalculateRelativeTargetRaw()` | Reads current position, adds relative tick delta, rejects out-of-range result |
| `Kinematics_GetJointLimitsDeg()` | Converts calibrated min/home/max ticks into solver angle limits |

---

## 2. Values from the URDF

### 2.1 Transform Convention

For every chain element:

```text
T(parent -> child)
    = Trans(x, y, z)
    * RotZ(yaw)
    * RotY(pitch)
    * RotX(roll)
    * RotZ(q)
```

URDF origins are parent-relative transforms. They are not global joint positions.

### 2.2 URDF Joint Parameters

| Joint | Type | Parent Link | Child Link | Origin XYZ [m] | Origin RPY [rad] | Axis | URDF Limit [rad] |
|---|---|---|---|---|---|---|---|
| `shoulder_pan` | revolute | `base_link` | `shoulder_link` | `0.0388353 -8.97657e-09 0.0624` | `3.14159 4.18253e-17 -3.14159` | `0 0 1` | `[-1.91986, 1.91986]` |
| `shoulder_lift` | revolute | `shoulder_link` | `upper_arm_link` | `-0.0303992 -0.0182778 -0.0542` | `-1.5708 -1.5708 0` | `0 0 1` | `[-1.74533, 1.74533]` |
| `elbow_flex` | revolute | `upper_arm_link` | `lower_arm_link` | `-0.11257 -0.028 1.73763e-16` | `-3.63608e-16 8.74301e-16 1.5708` | `0 0 1` | `[-1.69, 1.69]` |
| `wrist_flex` | revolute | `lower_arm_link` | `wrist_link` | `-0.1349 0.0052 3.62355e-17` | `4.02456e-15 8.67362e-16 -1.5708` | `0 0 1` | `[-1.65806, 1.65806]` |
| `wrist_roll` | revolute | `wrist_link` | `gripper_link` | `5.55112e-17 -0.0611 0.0181` | `1.5708 0.0486795 3.14159` | `0 0 1` | `[-2.74385, 2.84121]` |
| `gripper_frame_joint` | fixed | `gripper_link` | `gripper_frame_link` | `-0.0079 -0.000218121 -0.0981274` | `0 3.14159 0` | `0 0 0` | Fixed |
| `gripper` | revolute | `gripper_link` | `moving_jaw_so101_v1_link` | `0.0202 0.0188 -0.0234` | `1.5708 -5.24284e-08 -1.41553e-15` | `0 0 1` | `[-0.174533, 1.74533]` |

### 2.3 URDF Limits in Degrees

| Joint | URDF Minimum [deg] | URDF Maximum [deg] |
|---|---:|---:|
| J1 `shoulder_pan` | `-109.999875` | `109.999875` |
| J2 `shoulder_lift` | `-100.000043` | `100.000043` |
| J3 `elbow_flex` | `-96.829867` | `96.829867` |
| J4 `wrist_flex` | `-94.999840` | `94.999840` |
| J5 `wrist_roll` | `-157.211025` | `162.789342` |
| J6 `gripper` | `-10.000004` | `100.000043` |

### 2.4 Firmware Chain Entries

The following entries are compiled directly into `kinematics_chain[]`.

| Index | Link / Joint | Active | Joint ID | Runtime Joint Angle |
|---:|---|---:|---:|---|
| 0 | `shoulder_pan` | 1 | 1 | `q1` |
| 1 | `shoulder_lift` | 1 | 2 | `q2` |
| 2 | `elbow_flex` | 1 | 3 | `q3` |
| 3 | `wrist_flex` | 1 | 4 | `q4` |
| 4 | `wrist_roll` | 0 | 5 | `0` |
| 5 | `gripper_frame_joint` | 0 | 0 | `0` |

The gripper revolute joint is not in the TCP chain because it only moves the jaw side branch.

### 2.5 Zero-Pose Frame Positions

All active angles and fixed articulated angles are set to `q = 0`.

| Frame / Joint Origin | Base Position at `q = 0` [m] |
|---|---|
| `origin` | `(0, 0, 0)` |
| `shoulder_pan` | `(0.038835300, -0.000000009, 0.062400000)` |
| `shoulder_lift` | `(0.069234500, -0.018277809, 0.116600000)` |
| `elbow_flex` | `(0.097234500, -0.018277809, 0.229170000)` |
| `wrist_flex` | `(0.232134500, -0.018277809, 0.234370000)` |
| `wrist_roll` / `gripper_link` | `(0.293234500, -0.000177809, 0.234370000)` |
| `gripper_frame_link` TCP | `(0.391361900, -0.000011255, 0.226468745)` |

### 2.6 Limit Source Used at Runtime

| Limit Source | Used for Geometry | Used for IK Clamping | Used before Servo Write |
|---|---:|---:|---:|
| URDF joint limits | Yes, documentation/reference | No | No |
| Calibrated tick limits | No | Yes | Yes |

The current solver does **not** intersect calibrated limits with URDF limits. Runtime angle limits are derived exclusively from the servo calibration table.

---

## 3. Kinematic Chain

### 3.1 TCP Chain

```text
world
  -> base_link
  -> shoulder_pan      q1
  -> shoulder_link
  -> shoulder_lift     q2
  -> upper_arm_link
  -> elbow_flex        q3
  -> lower_arm_link
  -> wrist_flex        q4
  -> wrist_link
  -> wrist_roll        fixed q5 = 0
  -> gripper_link
  -> gripper_frame_joint
  -> gripper_frame_link / TCP
```

### 3.2 Homogeneous Transform Representation

```text
T = [ R00 R01 R02 px ]
    [ R10 R11 R12 py ]
    [ R20 R21 R22 pz ]
    [  0   0   0   1 ]
```

| Type | C Representation |
|---|---|
| Transform | `Kinematics_Transform_t` |
| Matrix storage | `float m[4][4]` |
| Fixed link pose | `Operations_LinkPose_t {x,y,z,roll,pitch,yaw}` |
| Cartesian position | `Kinematics_Position_t {x,y,z}` |

### 3.3 Local Link Transform

The fixed orientation is calculated first:

```text
R_fixed = RotZ(yaw) * RotY(pitch) * RotX(roll)
```

The active joint rotation is post-multiplied:

```text
R_local = R_fixed * RotZ(q)
```

The full local transform is:

```text
T_local = [ R_local  p_origin ]
          [    0        1     ]
```

### 3.4 Chain Multiplication

For two homogeneous transforms:

```text
A = [ Ra  pa ]
    [  0   1 ]

B = [ Rb  pb ]
    [  0   1 ]

A * B = [ Ra*Rb   Ra*pb + pa ]
        [   0          1     ]
```

`Operations_Multiply()` uses this specialized homogeneous multiplication rather than a generic `4x4` matrix loop.

### 3.5 Forward-Kinematics Flow

```text
Kinematics_ForwardDeg(q_deg)
        |
        +-- convert q1...q4 from degrees to radians
        |
        v
Kinematics_ForwardRad(q_rad)
        |
        +-- total = identity
        |
        +-- for each compiled chain entry:
        |      active entry -> use next q value
        |      fixed entry  -> use q = 0
        |      build local transform
        |      total = total * local
        |
        v
4x4 TCP transform
        |
        v
Kinematics_GetPosition()
        |
        v
TCP position = [T03, T13, T23]
```

### 3.6 Current-Hardware FK Flow

```text
Servo_ReadPosition(J1...J4)
        |
        v
Kinematics_RawToAngleDeg()
        |
        v
Current joint vector [q1 q2 q3 q4]
        |
        v
Kinematics_ForwardDeg()
        |
        v
Current TCP transform
```

Public entry point:

```c
Kinematics_ReadCurrentEndEffector(&transform);
```

### 3.7 Application Start Pose

The start TCP is calculated from the hard-coded start ticks, not from a new hardware measurement.

| Quantity | Value |
|---|---|
| Joint vector [deg] | `[0.000000, -73.740234, 44.033203, 28.564453]` |
| TCP `x` [m] | `0.242953` |
| TCP `y` [m] | `-0.000010` |
| TCP `z` [m] | `0.241643` |

Approximate start transform:

```text
[  0.019920  -0.000978   0.999801   0.242953 ]
[  0.048660   0.998815   0.000007  -0.000010 ]
[ -0.998617   0.048651   0.019944   0.241643 ]
[  0.000000   0.000000   0.000000   1.000000 ]
```

### 3.8 Compute Path Selection

| Setting | Current Value | Effect |
|---|---:|---|
| `KINEMATICS_FASTEST_COMPUTE` | `0` | Standard `sinf()` / `cosf()` path |
| `OPERATIONS_USE_CMSIS_DSP` | Build-dependent | Enables `arm_sin_f32()` / `arm_cos_f32()` for the fast path |

With the current compile-time value, `Operations_LinkTransform()` is used.

---

## 4. IK Chain

### 4.1 IK Problem Definition

```text
Input:
    target_position = [x_target, y_target, z_target]
    seed_joint_deg  = [q1, q2, q3, q4]

Output:
    result_joint_deg = [q1*, q2*, q3*, q4*]
```

| Property | Current Implementation |
|---|---|
| Task-space dimensions | `3` (`x`, `y`, `z`) |
| Joint-space dimensions | `4` (`q1`–`q4`) |
| Redundancy | One extra joint-space degree of freedom |
| Orientation objective | None |
| Solver type | Numerical damped least-squares Jacobian method |
| Jacobian generation | Forward finite differences |
| Seed | Current measured joint angles for runtime moves |
| Limit handling | Clamp each joint after every solver update |

### 4.2 Default IK Configuration

| Parameter | Default Value | Meaning |
|---|---:|---|
| `max_iterations` | `80` | Maximum numerical solver iterations |
| `position_tolerance_m` | `0.005` | Cartesian convergence radius |
| `finite_difference_step_deg` | `1.0` | Trial joint step used for numerical Jacobian |
| `damping` | `0.0015` | Damped least-squares regularization factor |
| `max_step_deg` | `5.0` | Per-joint update clamp per iteration |

Invalid or non-positive configuration values are replaced by their defaults.

### 4.3 Application IK Configuration

`App_Process()` overrides the defaults before every square movement.

| Parameter | App Value |
|---|---:|
| `max_iterations` | `200` |
| `position_tolerance_m` | `0.005` |
| `finite_difference_step_deg` | `0.5` |
| `damping` | `0.02` |
| `max_step_deg` | `5.0` |

### 4.4 Numerical Jacobian

For each active joint `j`, the solver creates a trial vector:

```text
q_trial[j] = q[j] + finite_difference_step
```

Near the upper joint limit, the trial direction is reversed. The final trial value is clamped to the calibrated angle range.

The derivative is calculated per radian:

```text
J[:,j] = (p(q_trial) - p(q)) / delta_q_rad
```

Matrix dimensions:

```text
J : 3 x 4
```

| Row | Cartesian Component |
|---:|---|
| `0` | `x` |
| `1` | `y` |
| `2` | `z` |

| Column | Joint |
|---:|---|
| `0` | J1 |
| `1` | J2 |
| `2` | J3 |
| `3` | J4 |

### 4.5 Damped Least-Squares Update

The implemented matrix sequence is:

```text
e = p_target - p_current

A = J * J^T + damping^2 * I        // 3 x 3

v = inverse(A) * e                 // 3 x 1

delta_q = J^T * v                 // 4 x 1
```

Equivalent pseudoinverse form:

```text
delta_q = J^T * (J * J^T + damping^2 * I)^-1 * e
```

Each `delta_q` is converted from radians to degrees and clamped:

```text
-max_step_deg <= delta_q_deg <= max_step_deg
```

Then:

```text
q_next = clamp(q_current + delta_q, q_min, q_max)
```

### 4.6 IK Iteration Flow

```text
Seed current joint angles
        |
        v
Clamp seed to calibrated joint limits
        |
        v
Forward kinematics -> current TCP
        |
        v
Compute Cartesian error
        |
        +-- error <= tolerance -> success
        |
        v
Build numerical 3x4 Jacobian
        |
        v
A = J*J^T + damping^2*I
        |
        v
Invert A
        |
        +-- singular / determinant too small -> target not reached
        |
        v
delta_q = J^T * A^-1 * error
        |
        v
Clamp update and joint angles
        |
        v
Next iteration
        |
        +-- max iterations reached -> target not reached
```

`Operations_Invert3x3()` rejects matrices with:

```text
|det(A)| < 1.0e-12
```

### 4.7 Angle-to-Raw Conversion

After successful IK:

```text
q1...q4 [deg]
        |
        v
Kinematics_AngleDegToRaw()
        |
        +-- add home offset
        +-- apply direction
        +-- round to nearest tick
        +-- validate calibrated range
        |
        v
target_raw[4]
```

### 4.8 Motion API Variants

| Function | IK Computation | Servo Command Strategy | Completion Check |
|---|---|---|---|
| `Kinematics_MoveEndEffectorToPosition()` | Once | Send final raw target once to J1–J4 | None |
| `Kinematics_MoveEndEffectorToPositionAndWait()` | Once | Send final raw target once to J1–J4 | Sequential tick polling |
| `Kinematics_MoveEndEffectorToPositionControlled()` | Once | Repeated incremental PID commands | All joints within tolerance for 3 cycles |

The application uses the controlled variant.

### 4.9 Controlled Motion Loop

The IK target is solved once before entering the control loop. The IK is **not** recomputed after every servo update.

```text
Read current angles
        |
        v
Solve IK once -> fixed target_raw[4]
        |
        v
Reset four PID controllers
        |
        v
Every 50 ms:
    read all four current raw positions
    calculate each tick error
    run PID for joints outside tolerance
    clamp command to calibrated limits
    send incremental raw command
    report telemetry
        |
        v
Success after all four joints remain inside tolerance for 3 cycles
```

### 4.10 Joint PID Configuration

| Parameter | Value |
|---|---:|
| `Kp` | `1.0` |
| `Ki` | `0.5` |
| `Kd` | `0.05` |
| Maximum update per cycle | `100 ticks` |
| Derivative filter time constant | `0.05 s` |
| Control period | `50 ms` |
| Required settled cycles | `3` |

Controller equation:

```text
error = setpoint_tick - actual_tick

integral_candidate = integral + error * dt

derivative_raw = (error - previous_error) / dt
alpha = dt / (tau + dt)
derivative_filtered += alpha * (derivative_raw - derivative_filtered)

output = Kp*error + Ki*integral_candidate + Kd*derivative_filtered
```

The output is clamped to `[-100, +100]` ticks. Conditional integration prevents windup while saturation acts in the same direction as the error.

### 4.11 Controlled-Motion Termination

| Condition | Result |
|---|---|
| All joints inside tolerance for three consecutive cycles | `SERVO_RESULT_OK` |
| Abort callback returns non-zero | `SERVO_RESULT_ABORTED` |
| Timeout expires | `SERVO_RESULT_TARGET_NOT_REACHED` |
| Position read/write fails | Corresponding servo/UART error |
| IK fails | `SERVO_RESULT_TARGET_NOT_REACHED` or propagated error |

### 4.12 Telemetry Fields

| Field | Meaning |
|---|---|
| `cycle_index` | Control loop iteration |
| `dt_s` | Measured control interval |
| `joint_id` | Active joint ID |
| `current_position_ticks` | Latest encoder reading |
| `target_position_ticks` | Fixed IK result |
| `error_ticks` | Signed target error |
| `controller_output_ticks` | Floating-point PID result |
| `applied_correction_ticks` | Actual integer command increment |
| `commanded_position_ticks` | New raw target sent to servo |
| `within_tolerance` | Joint currently accepted as reached |
| `command_sent` | Position command transmitted this cycle |
| `joint_limit_clamped` | Incremental command hit a calibrated limit |

---

## 5. App Implementation

### 5.1 Application Constants

| Constant | Value | Purpose |
|---|---:|---|
| `BUTTON_DEBOUNCE_MS` | `50 ms` | B1 debounce interval |
| `APP_MOVEMENT_SPEED` | `300` | Servo speed field |
| `APP_MOVEMENT_ACCELERATION` | `50` | Servo acceleration field |
| `APP_CONTROL_TOLERANCE_TICKS` | `5 ticks` | PID target tolerance |
| `APP_SQUARE_RADIUS_CM` | `12.0 cm` | Absolute offset applied independently to `y` and `z` |
| Controlled move timeout | `20,000 ms` | Maximum time per square corner |
| Startup wait tolerance | `50 ticks` | Startup pose acceptance |
| Startup timeout | `15,000 ms` per joint | Startup wait limit |

`APP_SQUARE_RADIUS_CM` acts as a half-side offset. The resulting square side length is `24 cm`.

### 5.2 Application States

```c
typedef enum
{
    APP_STATE_INIT = 0,
    APP_STATE_CHECKING_HOME,
    APP_STATE_IDLE,
    APP_STATE_UNLOCKING,
    APP_STATE_HOMING,
    APP_STATE_FAULT
} App_State;
```

| State | Current Runtime Use |
|---|---|
| `INIT` | Initial value during `App_Init()` |
| `CHECKING_HOME` | Declared, not entered by current app flow |
| `IDLE` | Normal square-trajectory execution |
| `UNLOCKING` | Declared, not entered by current app flow |
| `HOMING` | Declared, not entered by current app flow |
| `FAULT` | Entered after startup, IK, control, or unlock failure |

Actual state transitions:

```text
INIT
  |
  +-- startup move succeeds --> IDLE
  |
  +-- startup move fails ----> FAULT

IDLE
  |
  +-- square move succeeds --> IDLE
  |
  +-- square move fails ----> FAULT

Any state
  |
  +-- B1 unlock fails ------> FAULT
```

### 5.3 `App_Init()` Sequence

```text
1. Reset application variables
2. Attach CubeMX servo UART handle
3. Initialize servo joint table
4. Print boot message
5. Load hard-coded start ticks for J1-J4
6. Send one position command to each active joint
7. Delay 20 ms between joint commands
8. Delay 300 ms after all commands
9. Wait for each active joint sequentially
10. Accept each joint within 50 ticks
11. Abort on B1, timeout, or servo communication failure
12. Enter IDLE
```

Startup target:

```text
J1 = 2047
J2 = 1208
J3 = 2548
J4 = 2372
```

Startup command parameters:

```text
speed        = 300
acceleration = 50
tolerance    = 50 ticks
timeout      = 15000 ms per joint
```

### 5.4 Startup Error Handling

| Failure Point | Application Result |
|---|---|
| Servo write fails | Log joint/result, enter `FAULT`, return |
| Joint wait fails | Log joint/result, enter `FAULT`, return |
| B1 pressed during wait | Abort callback unlocks joints; wait returns aborted; app enters `FAULT` |

### 5.5 One-Time TCP Start Calculation

On the first valid `App_Process()` call:

```text
Hard-coded start ticks
        |
        v
RawToAngleDeg for J1-J4
        |
        v
ForwardDeg
        |
        v
Extract TCP XYZ
        |
        v
Store static tcp_start_position
```

The calculation uses the commanded startup constants. It does not read the actual encoder positions again for this reference.

Stored reference:

```text
x_start ~= 0.242953 m
y_start ~= -0.000010 m
z_start ~= 0.241643 m
```

### 5.6 Square Trajectory

The trajectory remains at constant `x` and moves around the stored start TCP in the `y-z` plane.

```text
offset = 12 cm = 0.12 m
```

| Step | Label in Code | Target X | Target Y | Target Z |
|---:|---|---|---|---|
| 0 | up-left | `x0` | `y0 - 0.12` | `z0 + 0.12` |
| 1 | up-right | `x0` | `y0 + 0.12` | `z0 + 0.12` |
| 2 | down-right | `x0` | `y0 + 0.12` | `z0 - 0.12` |
| 3 | down-left | `x0` | `y0 - 0.12` | `z0 - 0.12` |

Approximate absolute targets:

| Step | X [m] | Y [m] | Z [m] |
|---:|---:|---:|---:|
| 0 | `0.242953` | `-0.120010` | `0.361643` |
| 1 | `0.242953` | `0.119990` | `0.361643` |
| 2 | `0.242953` | `0.119990` | `0.121643` |
| 3 | `0.242953` | `-0.120010` | `0.121643` |

Sequence:

```text
step 0 -> step 1 -> step 2 -> step 3 -> step 0 -> ...
```

The center pose is used only as the target reference. The robot does not return to the center between corners.

### 5.7 `App_Process()` Motion Sequence

```text
Process pending button event
        |
        +-- not IDLE or motion disabled -> return
        |
        v
Load app IK configuration
        |
        v
Calculate start TCP once
        |
        v
Build target for current square step
        |
        v
Kinematics_MoveEndEffectorToPositionControlled()
        |
        +-- failure -> FAULT
        |
        v
Process button again
        |
        v
Increment step modulo 4
        |
        v
Delay 50 ms
        |
        v
Process button again
```

Although `App_Process()` is declared as a non-blocking application cycle in `app.h`, the current implementation blocks while the controlled motion is active, up to `20 s` per corner.

### 5.8 Controlled Move Call

```c
Kinematics_MoveEndEffectorToPositionControlled(
    &target_position,
    300U,
    50U,
    5U,
    20000U,
    &ik_config,
    app_motion_abort_requested,
    app_log_control_telemetry
);
```

| Argument | Value |
|---|---:|
| Speed | `300` |
| Acceleration | `50` |
| Tick tolerance | `5` |
| Timeout | `20000 ms` |
| Abort source | B1 motion-disable flag |
| Telemetry | One UART line per joint and control cycle |

### 5.9 B1 Abort and Unlock Flow

Interrupt handler:

```text
App_OnButtonInterrupt()
    -> button_event_pending = 1
```

Deferred processing:

```text
app_process_button(now_ms)
    |
    +-- ignore if no event
    +-- clear event flag
    +-- reject if inside 50 ms debounce window
    +-- app_motion_enabled = 0
    +-- unlock every configured joint J1-J6
    +-- enter FAULT only if unlock fails
```

Abort callback:

```text
app_motion_abort_requested()
    -> process button event
    -> return 1 when app_motion_enabled == 0
```

The callback is checked inside:

- startup position waits,
- controlled motion cycles,
- controlled-loop timing waits,
- other blocking kinematic wait helpers.

### 5.10 Post-Abort Behavior

| Property | Current Behavior |
|---|---|
| All six joints | Torque disable command is sent |
| Ongoing controlled motion | Returns `SERVO_RESULT_ABORTED` |
| Future square movements | Disabled because `app_motion_enabled = 0` |
| Resume function | Not implemented |
| Re-enable path | Requires reset or additional application logic |
| State after B1 between motions | Remains `IDLE`, but motion flag blocks execution |
| State after B1 during a controlled move | Controlled move returns `ABORTED`; `App_Process()` treats it as failure and enters `FAULT` |

### 5.11 Telemetry Output Format

```text
CTRL cycle=<n> dt_ms=<dt> joint=<id> current=<tick> target=<tick>
error=<signed_tick_error> pid_step=<float> applied=<tick_delta>
command=<tick> reached=<0|1> sent=<0|1> joint_limit=<0|1>
```

Example field mapping:

| Log Key | Telemetry Field |
|---|---|
| `cycle` | `cycle_index` |
| `dt_ms` | `dt_s * 1000` |
| `current` | `current_position_ticks` |
| `target` | `target_position_ticks` |
| `error` | `error_ticks` |
| `pid_step` | `controller_output_ticks` |
| `applied` | `applied_correction_ticks` |
| `command` | `commanded_position_ticks` |
| `reached` | `within_tolerance` |
| `sent` | `command_sent` |
| `joint_limit` | `joint_limit_clamped` |

### 5.12 Servo Communication Behavior

| Operation | Current Implementation |
|---|---|
| Position write | 14-byte Feetech write packet |
| Position write acknowledgement | Not validated; RX is drained after transmission |
| Position read | 8-byte read request and 8-byte response |
| Position read retries | Up to 5 attempts |
| Delay between read attempts | `20 ms` |
| TX timeout | `100 ms` |
| RX timeout | `100 ms` |
| Response checks | Header, servo ID, checksum, servo error byte |

Position command packet payload:

```text
FF FF ID LENGTH WRITE ACC_REGISTER
ACC POSITION_L POSITION_H TIME_L TIME_H SPEED_L SPEED_H CHECKSUM
```

### 5.13 Result Codes Propagated to the App

| Category | Result Codes |
|---|---|
| Success | `SERVO_RESULT_OK` |
| UART | `TX_ERROR`, `RX_TIMEOUT`, `RX_ERROR` |
| Packet validation | `INVALID_HEADER`, `INVALID_ID`, `INVALID_CHECKSUM`, `SERVO_ERROR` |
| Configuration | `NOT_INITIALIZED`, `UNKNOWN_JOINT_ID`, `JOINT_IS_FIXED` |
| Motion validation | `POSITION_OUT_OF_RANGE`, `TARGET_NOT_REACHED` |
| API validation | `NULL_POINTER` |
| User abort | `ABORTED` |

### 5.14 Current Implementation Notes

| Item | Current State |
|---|---|
| IK orientation control | Not implemented |
| IK update during PID motion | Not performed; IK target is fixed before control starts |
| Cartesian feedback during PID motion | Not used; control feedback is raw joint ticks |
| Square reachability pre-check | Not implemented; solver failure moves app to `FAULT` |
| Start reference source | Hard-coded target pose, not measured reached pose |
| Start tick definitions | Duplicated in `App_Init()` and first `App_Process()` calculation |
| Automatic homing in current app flow | Not used |
| Torque enable before startup command | Not explicitly called by `App_Init()` |
| Motion resume after B1 | Not implemented |
| `CHECKING_HOME`, `UNLOCKING`, `HOMING` states | Declared but unused |
| `last_status_log_ms` | Declared but unused |
| `HOME_CHECK_TOLERANCE_TICKS` | Defined but unused |
| `max_abs_delta` inside IK | Calculated but not used as a convergence criterion |
| Test functions | Available separately; not called by the active app path |

### 5.15 Complete Active Runtime Flow

```text
System startup
    |
    v
App_Init()
    |
    +-- attach UART
    +-- load follower calibration
    +-- command J1-J4 start ticks
    +-- wait for each active joint
    |
    +-- failure ------------------------------> FAULT
    |
    v
IDLE
    |
    v
App_Process()
    |
    +-- handle B1
    +-- calculate start TCP once
    +-- select square corner
    +-- read current J1-J4 angles
    +-- solve position-only IK once
    +-- convert IK angles to target ticks
    +-- run four tick-space PID controllers
    +-- require three settled cycles
    |
    +-- success -> next corner -> IDLE
    |
    +-- failure ------------------------------> FAULT
    |
    +-- B1 -> disable motion -> unlock J1-J6 -> remain stopped
```