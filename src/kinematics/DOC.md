# Inverse Kinematics Progress Notes

## 1. Goal of the Task

* My task is to implement the inverse kinematics for the SO101 follower arm.
* The final goal is to map Cartesian target positions of the end-effector to servo commands.
* The robot will later be controlled from an STM microcontroller over the UART servo daisy chain.
* For the first proof-of-concept, the gripper is kept closed and the wrist roll joint is fixed.
* Therefore, the first IK model focuses on Cartesian position control, not full 6D pose control.

---

## 2. Planned Workflow

### 2.1 Hardware and Calibration

* Connect the SO101 follower arm to the servo controller board.
* Use external 12 V motor power and USB-C for communication.
* Identify the serial port.
* Calibrate the robot and record the usable raw servo ranges.
* Record fixed raw values for:

  * `wrist_roll`
  * `gripper`

### 2.2 Robot Description

* Collect the joint description of the SO101 arm.
* Use the available URDF model as the nominal robot description.
* Extract:

  * joint names
  * parent and child links
  * joint origins
  * joint axes
  * joint limits
* Use this information to build the kinematic model.

### 2.3 Python Prototype

* Implement the first model in Python.
* Use Python because it is easier for:

  * testing the math
  * plotting the arm
  * debugging transformations
  * visualizing joint frames and TCP positions

### 2.4 Forward Kinematics

* First implement forward kinematics.
* Start with a single joint transformation.
* Then extend the transformation chain to all active joints.
* Use homogeneous transformation matrices.
* Check if the calculated TCP position behaves plausibly when changing joint angles.

### 2.5 Inverse Kinematics

* After forward kinematics is working, derive the inverse kinematics.
* Start with a simplified position-only IK.
* The first IK target is:

```python
target_position = [x, y, z]
```

* The end-effector orientation is fixed in the first version.
* The fixed joints are added later to the final servo command vector.

### 2.6 C/C++ Conversion

* After the Python prototype works, convert the mathematical core to C/C++.
* Keep the implementation lightweight.
* Only include the functions needed for the microcontroller:

  * joint mapping
  * forward kinematics
  * inverse kinematics
  * limit checks
  * trajectory update
  * fixed joint insertion

### 2.7 STM Integration

* Integrate the C/C++ code on the STM microcontroller.
* Send commands over the UART servo bus.
* Use the IK output to generate raw servo target values.
* Later connect this with the demo trajectory and LiDAR safety stop.

---

## 3. Calibration

* The SO101 follower arm was calibrated using the available calibration tool.
* During calibration, the main arm joints were moved through their safe motion range.
* The calibration produced raw minimum and maximum values for the active joints.
* The STS/ST3215 servo position scale is based on 4096 encoder steps over one full rotation.
* Therefore, raw values are expected to be in the range around:

```text
0 ... 4095
```

* The middle position is around:

```text
2047
```

* This explains why the recorded values are in the range of hundreds to a few thousand and not arbitrary large numbers.
* The wrist roll joint and gripper were not used as active IK joints.
* Instead, both were manually moved to the desired fixed configuration and their raw values were read out.

---

## 4. Joint Calibration and Fixed Joint Values

| Joint | Motor Name      | Min Raw Value | Current / Fixed Raw Value | Max Raw Value | Usage in IK    |
| ----- | --------------- | ------------: | ------------------------: | ------------: | -------------- |
| J1    | `shoulder_pan`  |           719 |                      2026 |          3401 | Active         |
| J2    | `shoulder_lift` |           777 |                       777 |          3318 | Active         |
| J3    | `elbow_flex`    |           890 |                      2541 |          3008 | Active         |
| J4    | `wrist_flex`    |           923 |                      1526 |          3238 | Active         |
| J5    | `wrist_roll`    |             — |                      1139 |             — | Fixed          |
| J6    | `gripper`       |           808 |                       826 |          2294 | Fixed / Closed |

---

## 5. Fixed Joint Values

```python
FIXED_WRIST_ROLL_RAW = 1139
FIXED_GRIPPER_RAW = 826
```

* These values are kept constant during the first IK implementation.
* The first IK version only computes target values for:

```python
q = [
    shoulder_pan,
    shoulder_lift,
    elbow_flex,
    wrist_flex
]
```

* The final servo command will later contain all six motor values:

```python
servo_command = [
    shoulder_pan_target,
    shoulder_lift_target,
    elbow_flex_target,
    wrist_flex_target,
    FIXED_WRIST_ROLL_RAW,
    FIXED_GRIPPER_RAW
]
```

---

## 6. Need for a Joint Description

* The calibration only gives the raw motion limits of the servos.
* For inverse kinematics, this is not enough.
* A kinematic model also needs the geometric relationship between the joints.
* Therefore, the next required step is to describe the robot structure.
* This includes:

  * joint frames
  * joint axes
  * link offsets
  * parent-child relations
  * TCP definition

---

## 7. URDF-Based Joint Description

* The SO101 arm already has an available URDF model.
* The URDF describes the robot with consistent joint frames and link transformations.
* The model can be visualized and appears consistent with the physical robot.
* Therefore, the URDF is used as the nominal source for the kinematic description.
* Manual measurements are only used as plausibility checks.

| Joint                 | Type     | Parent Link      | Child Link                 |                    Origin XYZ [m] |                   Origin RPY [rad] |    Axis |            Limit [rad] |
| --------------------- | -------- | ---------------- | -------------------------- | --------------------------------: | ---------------------------------: | ------: | ---------------------: |
| `shoulder_pan`        | revolute | `base_link`      | `shoulder_link`            |   `0.0388353 -8.97657e-09 0.0624` |     `3.14159 4.18253e-17 -3.14159` | `0 0 1` |  `-1.91986 to 1.91986` |
| `shoulder_lift`       | revolute | `shoulder_link`  | `upper_arm_link`           |   `-0.0303992 -0.0182778 -0.0542` |                `-1.5708 -1.5708 0` | `0 0 1` |  `-1.74533 to 1.74533` |
| `elbow_flex`          | revolute | `upper_arm_link` | `lower_arm_link`           |     `-0.11257 -0.028 1.73763e-16` |  `-3.63608e-16 8.74301e-16 1.5708` | `0 0 1` |        `-1.69 to 1.69` |
| `wrist_flex`          | revolute | `lower_arm_link` | `wrist_link`               |      `-0.1349 0.0052 3.62355e-17` |  `4.02456e-15 8.67362e-16 -1.5708` | `0 0 1` |  `-1.65806 to 1.65806` |
| `wrist_roll`          | revolute | `wrist_link`     | `gripper_link`             |      `5.55112e-17 -0.0611 0.0181` |         `1.5708 0.0486795 3.14159` | `0 0 1` |  `-2.74385 to 2.84121` |
| `gripper_frame_joint` | fixed    | `gripper_link`   | `gripper_frame_link`       | `-0.0079 -0.000218121 -0.0981274` |                      `0 3.14159 0` | `0 0 0` |                  fixed |
| `gripper`             | revolute | `gripper_link`   | `moving_jaw_so101_v1_link` |           `0.0202 0.0188 -0.0234` | `1.5708 -5.24284e-08 -1.41553e-15` | `0 0 1` | `-0.174533 to 1.74533` |

---

## 8. How the URDF Will Be Used

* The URDF origins are not interpreted as simple global coordinates.
* Each origin describes the transformation from a parent link frame to a child joint/link frame.
* To obtain joint positions, the transformations have to be multiplied along the kinematic chain.
* A new IK base frame can be defined at the `shoulder_pan` axis.
* This makes the model easier to understand:

```text
IK base origin = shoulder_pan rotation axis
IK z-axis      = shoulder_pan rotation axis
IK x-axis      = forward direction of the arm
IK y-axis      = right-hand rule
```

* The full URDF-based model will be used for forward kinematics and validation.
* A simplified geometric model can then be derived from it for the first IK implementation.

---

## 9. Current Unclear Point: URDF vs. Manual DH Model

### Option A: URDF-Based Model

Advantages:

* The URDF already contains a consistent robot model.
* Joint origins, axes and parent-child relations are already defined.
* It avoids inaccurate hand measurements.
* It includes 3D offsets and rotations that are hard to measure manually.
* It is a good basis for forward kinematics and visualization.

Disadvantages:

* The frame definitions are not always intuitive.
* It is not immediately obvious where every origin is located on the physical robot.
* The model has to be trusted as the nominal CAD-based description.
* Some offsets look strange when viewed only as numbers.

### Option B: Manual DH Parameterization

Advantages:

* The coordinate frames can be defined manually.
* The model may be easier to explain step by step.
* DH parameters are a standard method for robot kinematics.
* The origin of every measurement is more directly controlled.

Disadvantages:

* Exact joint axes are difficult to measure on the physical robot.
* It is easy to measure from servo housings instead of true rotation axes.
* Small measurement errors can accumulate over the kinematic chain.
* Side offsets and non-intuitive geometry make DH setup more difficult.
* The result may fit the real robot worse than the URDF-based model.

### Current Decision

* Use the URDF as the primary source for the nominal kinematic model.
* Use manual measurements only as plausibility checks.
* Implement the first forward kinematics from the URDF transformations.
* Derive a simplified planar/geometric IK model from the URDF-based joint distances.
* Validate the simplified IK by comparing its result with the URDF-based forward kinematics.

---

## 10. Next Steps

* Visualize the SO101 URDF.
* Extract the joint positions from the URDF transformation chain.
* Define the IK base frame at the `shoulder_pan` axis.
* Compute the approximate link distances.
* Implement a first Python forward kinematics model.
* Plot the arm and TCP position.
* Start with a simple position-only IK for the rectangle demo.
