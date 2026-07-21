# elroboto

### Standalone embedded motion control for the SO-101 robot arm

`elroboto` is a bare-metal robotics firmware project that runs the complete
motion stack of a six-axis SO-101 arm directly on an STM32 microcontroller. It
connects a NUCLEO-U545RE-Q to six Feetech STS3215 serial-bus servos through a
Waveshare Bus Servo Adapter and executes Cartesian motion without a host PC,
ROS, or an external motion controller.

The repository covers the full path from board configuration and UART
communication to servo calibration, forward and inverse kinematics, closed-loop
joint control, diagnostics, and application-level trajectory execution. The
current demonstration moves the arm's tool center point (TCP) continuously
around a square in the YZ plane.

<!--
Portfolio media placeholder. Suggested asset:
docs/images/elroboto-hero.jpg

When available, place a wide photograph or demo GIF here, for example:
<p align="center"><img src="docs/images/elroboto-hero.jpg" alt="SO-101 arm controlled by the STM32 Nucleo" width="900"></p>
-->

## Project highlights

- Entire control pipeline executes on an STM32U545RE Cortex-M33 MCU.
- Direct 1 Mbit/s UART communication with the STS3215 servo bus.
- Position-only forward and numerical inverse kinematics derived from the
  SO-101 kinematic chain.
- Joint calibration and mechanical limits enforced before position commands
  reach the servo bus.
- Four independent PID controllers close the loop using live encoder feedback.
- Cartesian square trajectory generated and executed autonomously on the MCU.
- Structured error propagation for UART failures, malformed responses,
  unreachable targets, joint-limit violations, timeouts, and user aborts.
- Reproducible CMake/Ninja cross-build using GCC for Arm Embedded.

## System overview

```text
                         ST-LINK Virtual COM Port
                         115200 baud diagnostics
                                  ^
                                  |
+----------------+       +--------+---------+       +-----------------------+
| Application    |------>| Kinematics and   |------>| Servo protocol and    |
| square path    |       | PID motion loop  |       | checked joint targets |
+----------------+       +------------------+       +-----------+-----------+
                                                               |
                                                  LPUART1, 1 Mbit/s, 8N1
                                                               |
                                                    +----------v----------+
                                                    | Waveshare Bus Servo |
                                                    | Adapter (A)         |
                                                    +----------+----------+
                                                               |
                                                       serial servo bus
                                                               |
                                                    +----------v----------+
                                                    | 6 x Feetech         |
                                                    | STS3215 / SO-101    |
                                                    +---------------------+
```

The software is intentionally layered. Hardware initialization remains in the
STM32CubeMX-generated `Core/` code, while the handwritten application,
kinematics, control, UART adapter, and servo protocol live under `App/`.

## Hardware platform

| Component | Role |
| --- | --- |
| **SO-101 robot arm** | Six-degree-of-freedom mechanical platform |
| **6 x Feetech STS3215** | Smart serial-bus actuators with position feedback |
| **Waveshare Bus Servo Adapter (A), WSH-SBS-01** | Interface between the MCU UART and the single-wire servo bus |
| **ST NUCLEO-U545RE-Q** | Main controller and integrated ST-LINK debugger |
| **STM32U545RET6Q** | Arm Cortex-M33 MCU with hardware FPU, 512 KiB Flash, and 274 KiB SRAM |
| **External 12 V supply** | Separate power source for adapter and servos |

### Servo-bus wiring

The Waveshare adapter is operated in jumper position **A** for the external MCU
interface.

| Signal | Nucleo connection | STM32 function | Waveshare label |
| --- | --- | --- | --- |
| Servo TX | Arduino D1, CN9 pin 2 | PA2 / LPUART1_TX / AF8 | TX |
| Servo RX | Arduino D0, CN9 pin 1 | PA3 / LPUART1_RX / AF8 | RX |
| Reference | GND | Ground | GND |

The TX-to-TX and RX-to-RX mapping is deliberate and follows the labeling and
interface behavior of this Waveshare adapter. It must not be rewired according
to conventional crossed UART assumptions without checking the adapter
documentation and the physical board revision.

```text
NUCLEO-U545RE-Q                         Waveshare adapter (jumper A)

PA2 / D1 / LPUART1_TX  ----------------  TX
PA3 / D0 / LPUART1_RX  ----------------  RX
GND                    ----------------  GND

                                               12 V external supply
                                                        |
                                             adapter + servo power rail
```

> [!CAUTION]
> The 12 V servo supply must never be connected to a Nucleo supply pin. The
> Nucleo and the adapter require a common ground. Connect servo power only after
> verifying the signal wiring, jumper position, servo IDs, and supply polarity.

<!-- Suggested future asset: docs/images/hardware-wiring.jpg -->

## STM32 configuration

[`elroboto.ioc`](elroboto.ioc) is the source of truth for the MCU, pins,
clocks, interrupts, power configuration, and generated HAL initialization. The
current project was generated with STM32CubeMX 6.17.0 and STM32CubeU5 firmware
package 1.8.0.

| Area | Configuration |
| --- | --- |
| Target | NUCLEO-U545RE-Q / STM32U545RET6Q / LQFP64 |
| Execution model | Bare metal, cooperative foreground application |
| TrustZone | Disabled |
| System clock | HSI16 at 16 MHz, no PLL |
| AHB / APB1 / APB2 / APB3 | 16 MHz |
| Power supply | Internal SMPS regulator |
| Instruction cache | Enabled, direct-mapped (one-way) |
| Time base | SysTick, 1 ms HAL tick |
| User button | B1 on PC13, EXTI13 interrupt |
| Servo interface | LPUART1, PA2 TX and PA3 RX, AF8 |
| Servo UART format | 1,000,000 baud, 8 data bits, no parity, 1 stop bit, no flow control |
| LPUART1 clock | HSI16 at 16 MHz |
| Debug interface | USART1 through the ST-LINK Virtual COM Port |
| Debug pins | PA9 TX and PA10 RX, AF7 |
| Debug UART format | 115,200 baud, 8N1, no flow control |

The 16 MHz clock is an important integration decision. During bring-up, a 4 MHz
system clock combined with byte-by-byte receive calls could not reliably capture
the complete six-byte response at 1 Mbit/s. Receiving each expected packet as a
single block at 16 MHz resolved the observed data loss. The bring-up history is
documented in [`docs/servo_bus_bringup.md`](docs/servo_bus_bringup.md).

## Firmware architecture

```text
Core/Src/main.c
  |
  +-- CubeMX/HAL: clock, power, ICACHE, GPIO, EXTI, LPUART1
  +-- Nucleo BSP: USART1 Virtual COM Port and B1 user button
  |
  `-- App_Init() / App_Process()
        |
        +-- app.c          demo sequencing, state, abort handling, telemetry
        +-- kinematics.c   calibration conversion, FK, IK, Cartesian movement
        +-- control.c      per-joint PID controller
        +-- operations.c   transforms, optimized matrix math, 3x3 inversion
        +-- servo.c        STS3215 packets, feedback, limits, torque, homing
        `-- uart.c         HAL UART transport adapter and receive recovery
```

### Servo communication and joint safety

The servo module implements the packet-level operations required by the
application:

- ping and status-packet validation;
- torque enable and disable;
- present-position reads from register `0x38`;
- position, speed, and acceleration writes beginning at register `0x29`;
- header, response ID, checksum, and servo-status validation;
- bounded TX/RX timeouts, stale-RX draining, and retry handling;
- centralized ID-to-joint mapping, home values, fixed-joint flags, and raw
  position limits.

Commands are addressed to individual servo IDs. The default motion path does
not use broadcast writes. `Servo_WritePosition()` rejects unknown or fixed
joints and refuses targets outside the configured raw range before generating a
bus packet.

### Joint configuration

The currently selected table is the SO-101 follower-arm configuration in
[`App/Src/servo.c`](App/Src/servo.c).

| Joint | Servo ID | Raw minimum | Raw home | Raw maximum | Motion role |
| --- | ---: | ---: | ---: | ---: | --- |
| `shoulder_pan` | 1 | 729 | 2047 | 3391 | Active |
| `shoulder_lift` | 2 | 787 | 2047 | 3308 | Active |
| `elbow_flex` | 3 | 900 | 2047 | 2998 | Active |
| `wrist_flex` | 4 | 933 | 2047 | 3228 | Active |
| `wrist_roll` | 5 | 2000 | 2047 | 2200 | Fixed for the current kinematic model |
| `gripper` | 6 | 810 | 810 | 2200 | Fixed and closed for the current demo |

The STS3215 position domain is represented as 4096 ticks per revolution.
Active-joint angles are defined relative to each joint's calibrated home tick:

```text
angle_deg = (raw_ticks - home_ticks) * 360 / 4096
raw_ticks = home_ticks + round(angle_deg * 4096 / 360)
```

All four active joints currently use a positive mathematical-angle-to-raw-tick
direction. The conversion layer checks the calibrated range in both directions.

## Direct and inverse kinematics

The project uses **forward kinematics (FK)**—also referred to as direct
kinematics (DK)—to calculate the TCP pose from measured joint angles. It uses
**inverse kinematics (IK)** to calculate joint targets for a requested Cartesian
TCP position.

### Kinematic model

The model is based on the SO-101 link geometry and uses homogeneous 4 x 4
transforms. URDF joint origins are interpreted as parent-to-child transforms:

```text
T(parent -> child) = Trans(x, y, z)
                   * RotZ(yaw) * RotY(pitch) * RotX(roll)
                   * RotZ(q)
```

The evaluated chain is:

```text
base_link
  -> shoulder_link       q1: shoulder_pan
  -> upper_arm_link      q2: shoulder_lift
  -> lower_arm_link      q3: elbow_flex
  -> wrist_link          q4: wrist_flex
  -> gripper_link        wrist_roll fixed
  -> gripper_frame_link  TCP
```

Joints 1 through 4 are active in the solver. Wrist roll and gripper remain
fixed, so the current IK solves TCP **position** `(x, y, z)` but does not solve
end-effector orientation.

### Forward kinematics

`Kinematics_ForwardRad()` composes the six link transforms in chain order. The
operations layer uses a specialized homogeneous-transform multiplication that
avoids unnecessary work on the constant final matrix row. Standard `sinf()` and
`cosf()` provide the production trigonometric path; a CMSIS-DSP FastMath path is
available behind a compile-time option and has been benchmarked separately.

### Numerical inverse kinematics

`Kinematics_InversePositionDeg()` implements a damped least-squares numerical
solver:

1. Read the current joint angles and use them as the IK seed.
2. Evaluate the current TCP position with FK.
3. Estimate the 3 x 4 positional Jacobian using finite differences.
4. Solve the damped system `J^T (J J^T + lambda^2 I)^-1 e`.
5. Limit the update per joint, clamp every candidate to calibrated joint limits,
   and iterate until the Cartesian tolerance is reached.
6. Convert the solution back to validated STS3215 raw targets.

Default IK parameters are configurable through `Kinematics_IkConfig_t`. The
square application currently selects:

| Parameter | Demo value |
| --- | ---: |
| Maximum iterations | 200 |
| Cartesian position tolerance | 0.005 m |
| Finite-difference step | 0.5 deg |
| Damping factor | 0.02 |
| Maximum joint update per iteration | 5 deg |

The solver reports `TARGET_NOT_REACHED` for an unreachable or numerically
unsolved target rather than issuing a partially validated Cartesian move.

## Closed-loop joint controller

After IK produces the four raw joint targets, the controlled Cartesian move
runs one PID instance per active joint. Every 50 ms, the firmware:

1. reads the current encoder position of all four active joints;
2. computes each signed raw-tick error;
3. applies PID feedback with derivative low-pass filtering and conditional
   integration for anti-windup;
4. limits each correction to 100 ticks per control cycle;
5. clamps the command to the joint's calibrated raw limits;
6. sends a checked position command to every joint still outside tolerance;
7. publishes detailed telemetry through the debug UART.

The current controller constants are `Kp = 1.0`, `Ki = 0.5`, `Kd = 0.05`, with
a derivative-filter time constant of 50 ms. A move is accepted only after all
four joints remain within the configured tolerance for three consecutive
cycles. Communication errors, user aborts, and the 20 s motion timeout are
returned to the application as explicit result codes.

Example telemetry fields include cycle time, joint ID, current and target raw
positions, signed error, PID output, applied correction, final command,
settled-state indication, and joint-limit clamping.

## Square TCP application

The default application in [`App/Src/app.c`](App/Src/app.c) is an autonomous
Cartesian demonstration:

1. Initialize the servo transport and calibrated joint table.
2. Command joints 1-4 to the known start configuration
   `[2047, 1208, 2548, 2372]` ticks.
3. Wait for every joint to reach that configuration.
4. Convert the start configuration to angles and calculate the start TCP with
   forward kinematics.
5. Generate four corner targets around that TCP while keeping X constant.
6. Solve IK and execute each target with the closed-loop joint controller.
7. Repeat the four Cartesian segments continuously.

The square lies in the YZ plane. Each corner is offset by 0.12 m in Y and
0.12 m in Z from the calculated center, producing a nominal side length of
0.24 m:

```text
                         +Z
                          ^
                          |
        step 0 (-Y,+Z) +--+--+ step 1 (+Y,+Z)
                       |     |
                       | TCP |    X remains constant
                       |start|
        step 3 (-Y,-Z) +-----+ step 2 (+Y,-Z)
                          |
                          +--------------------> +Y
```

The demo uses servo speed `300`, acceleration `50`, a five-tick controller
tolerance, and a 20 s timeout per corner. These speed and acceleration values
are STS3215 command units, not SI units.

Pressing the Nucleo B1 user button sets an abort flag, stops the square sequence,
and disables torque on all six joints. The interrupt handler only records the
event; debouncing and servo communication occur in application context.

> [!WARNING]
> B1 is a software convenience stop. It is not a certified emergency stop and
> does not make the arm electrically or mechanically safe. Keep a physical means
> of disconnecting servo power available during every hardware test.

<!-- Suggested future asset: docs/images/square-path-demo.gif -->

## Error handling and operational boundaries

The firmware exposes failures instead of silently discarding them. Result codes
cover transport errors, receive timeouts, invalid headers, unexpected IDs,
checksum failures, servo-reported errors, invalid positions, fixed or unknown
joints, null arguments, unreachable targets, and motion aborts.

The application defines `INIT`, `CHECKING_HOME`, `IDLE`, `UNLOCKING`, `HOMING`,
and `FAULT` states. The current square-demo path actively transitions from
`INIT` to `IDLE`, or to `FAULT` when startup or motion fails; the additional
states support retained bring-up and homing workflows.

Current engineering boundaries are intentionally visible:

- servo transport uses bounded, blocking HAL UART calls rather than DMA;
- the controlled move is synchronous and occupies the foreground application
  until the target settles, aborts, or times out;
- IK controls position only and uses four active joints;
- wrist roll and gripper are fixed for the current demonstration;
- this firmware is a development and demonstration controller, not a
  functionally safe motion system.

## Repository structure

| Path | Purpose |
| --- | --- |
| `App/Inc`, `App/Src` | Handwritten application, control, kinematics, operations, servo, and UART modules |
| `Core/` | STM32CubeMX-generated startup, peripheral initialization, interrupts, and runtime integration |
| `Drivers/` | STM32U5 HAL, CMSIS, CMSIS-DSP sources, and Nucleo BSP |
| `cmake/` | Arm GCC toolchain and generated STM32 target integration |
| `docs/` | Hardware wiring, kinematic assumptions, and servo-bus bring-up record |
| `kinematics/` | Python reference scripts, retained test utilities, and MCU benchmark documentation |
| `elroboto.ioc` | Authoritative STM32CubeMX project configuration |
| `CMakeLists.txt` | Top-level C11 firmware build definition |
| `STM32U545xx_FLASH.ld` | Flash execution linker script |
| `startup_stm32u545xx.s` | Cortex-M33 reset handler and vector table |

Additional details are available in:

- [`docs/hardware.md`](docs/hardware.md) — power, wiring, and hardware status;
- [`docs/kinematics.md`](docs/kinematics.md) — calibration and kinematic model notes;
- [`docs/servo_bus_bringup.md`](docs/servo_bus_bringup.md) — UART integration and
  real-hardware communication bring-up;
- [`kinematics/KIN_BENCHMARKS.md`](kinematics/KIN_BENCHMARKS.md) — operations
  and kinematics performance experiments;
- [`Core/README.md`](Core/README.md) — generated core-file responsibilities.

## Build

### Prerequisites

- CMake 3.22 or newer;
- Ninja;
- GNU Arm Embedded toolchain available as `arm-none-eabi-*` on `PATH`;
- STM32CubeMX or STM32CubeIDE only when changing and regenerating the `.ioc`
  configuration;
- ST-LINK tooling or an IDE debug integration for programming the board.

Configure and build the Debug firmware:

```bash
cmake --preset Debug
cmake --build --preset Debug
```

For an optimized size-oriented build:

```bash
cmake --preset Release
cmake --build --preset Release
```

The resulting images are written to:

```text
build/Debug/elroboto.elf
build/Release/elroboto.elf
```

The toolchain targets Cortex-M33 with the single-precision hardware FPU and
hard-float ABI. Debug uses `-O0 -g3`; Release uses `-Os -g0`.

### Serial diagnostics

After programming the board, connect to the ST-LINK Virtual COM Port at
115200 baud, 8N1. On macOS, first identify the assigned device and then open it:

```bash
ls /dev/cu.usbmodem*
screen /dev/cu.usbmodemXXXX 115200
```

The firmware logs startup progress, application state changes, Cartesian
targets, motion failures, and per-joint controller telemetry.

## Validation status

- The current Debug target cross-compiles and links successfully with no build
  warnings emitted by the project sources.
- The latest local Debug build occupies approximately 60 KiB of Flash and
  2.8 KiB of RAM.
- LPUART1 loopback at 1 Mbit/s and the complete
  `STM32 -> Waveshare adapter -> STS3215` communication chain have been validated
  on real hardware.
- A single STS3215 responded successfully to a read-only ping as servo ID 6 at
  1 Mbit/s during the documented initial bring-up.
- Dedicated home, direct-kinematics, inverse-kinematics, and benchmark routines
  are retained in `App/Src/tests.c` and `kinematics/`; they are not part of the
  default square-demo build.

Hardware validation always depends on the actual arm calibration, servo-ID
assignment, wiring, power supply, and mechanical setup. Before reproducing a
movement test, support the arm, clear its workspace, use conservative motion
parameters, verify each servo individually, and keep a physical power disconnect
within reach.

## Development workflow

Hardware changes belong in [`elroboto.ioc`](elroboto.ioc). After regenerating
with STM32CubeMX, handwritten changes inside generated files must remain within
`USER CODE` sections. Application modules under `App/` are maintained separately
and registered in the top-level CMake target.

When extending the project, keep physical values unit-qualified, preserve
finite communication timeouts, validate every target against centralized joint
limits, and document changes to wiring, calibration, coordinate conventions, or
motion behavior together with the code.
