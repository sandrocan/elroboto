# elroboto

### Standalone embedded motion control for the SO-101 robot arm

`elroboto` is a bare-metal robotics firmware project that runs the complete
motion stack of a six-axis SO-101 arm directly on an STM32 microcontroller. It
connects a NUCLEO-U545RE-Q to six Feetech STS3215 serial-bus servos through a
Waveshare Bus Servo Adapter and executes Cartesian motion without a host PC,
ROS, or an external motion controller. A large-area event-driven E-Skin provides
proximity feedback through an ESP32-C6 and a second MCU UART.

The repository covers the full path from board configuration and UART
communication to servo calibration, forward and inverse kinematics, closed-loop
Cartesian control, E-Skin monitoring, diagnostics, and application-level
trajectory execution. The current demonstration moves the arm's tool center
point (TCP) continuously around a square in the YZ plane and pauses the motion
when the E-Skin reports a nearby object.

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
- Cartesian resolved-rate P control using live servo position feedback.
- Event-driven E-Skin integration over UART4 with startup validation, runtime
  timeout supervision, pause/hold behavior, and hysteretic resume handling.
- Cartesian square trajectory generated and executed autonomously on the MCU.
- Structured error propagation for UART failures, malformed responses,
  malformed E-Skin frames, stale sensor data, unreachable targets, joint-limit
  violations, timeouts, and user aborts.
- Reproducible CMake/Ninja cross-build using GCC for Arm Embedded.

## System overview

```text
Large-area E-Skin --> ESP32-C6 --> UART4 RX, 115200 baud
                                          |
                                          v
                         +----------------+----------------+
                         | Application state and E-Skin    |
                         | pause/fault supervision         |
                         +---------------+-----------------+
                                         |
                                         v
                         +---------------+-----------------+
                         | Resolved-rate control and       |
                         | damped least-squares kinematics |
                         +---------------+-----------------+
                                         |
                              checked joint targets
                                         |
                                         v
                         +---------------+-----------------+
                         | STS3215 protocol on LPUART1     |
                         | 1 Mbit/s --> Waveshare adapter  |
                         +---------------+-----------------+
                                         |
                                         v
                              6 x STS3215 / SO-101

STM32 diagnostics --> USART1 --> ST-LINK Virtual COM Port, 115200 baud
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
| **Large-area event-driven E-Skin** | Distributed tactile/proximity sensing surface on the robot |
| **ESP32-C6** | Aggregates the E-Skin data and transmits the maximum proximity value to the STM32 |
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

### E-Skin wiring and UART protocol

The project uses the large-area event-driven E-Skin described by Bergner,
Dean-Leon, and Cheng. An ESP32-C6 aggregates its sensor data and sends the
maximum proximity value as an ASCII line to the STM32.

| Signal | ESP32-C6 connection | STM32 connection |
| --- | --- | --- |
| E-Skin data | TX / GPIO16 | PC11 / UART4_RX / CN7 pin 2 |
| Reference | GND | GND |

UART4 operates at 115,200 baud with 8N1 framing and no flow control. The current
protocol is unidirectional, so PC10 / UART4_TX is configured in CubeMX but is not
required by the physical connection. Each accepted measurement contains exactly
five ASCII characters followed by LF; CR before LF is ignored. Valid examples
are `0.123\n` and `0.123\r\n`.

UART4 receives one byte at a time using interrupts. The receive callback only
assembles and publishes a complete frame; `UartCell_Process()` performs the
floating-point conversion in foreground context. Sequence counters protect the
frame handover from concurrent ISR updates. Byte, frame, UART-error, receive-
restart, and last-valid-sample diagnostics remain available to the application.

The E-Skin design and event-driven sensing principle are described in:

> F. Bergner, E. Dean-Leon, and G. Cheng, “Design and Realization of an
> Efficient Large-Area Event-Driven E-Skin,” *Sensors*, vol. 20, no. 7, 1965,
> 2020. [doi:10.3390/s20071965](https://doi.org/10.3390/s20071965).

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
| Servo RX buffering | LPUART1 RX FIFO enabled |
| E-Skin interface | UART4, PC10 TX and PC11 RX, AF8; only RX is physically required |
| E-Skin UART format | 115,200 baud, 8N1, no flow control; interrupt-driven receive |
| UART4 clock | APB1 peripheral clock at 16 MHz |
| Debug interface | USART1 through the ST-LINK Virtual COM Port |
| Debug pins | PA9 TX and PA10 RX, AF7 |
| Debug UART format | 115,200 baud, 8N1, no flow control |

The 16 MHz clock is an important integration decision. During bring-up, a 4 MHz
system clock combined with byte-by-byte receive calls could not reliably capture
the complete six-byte response at 1 Mbit/s. Receiving each expected packet as a
single block at 16 MHz resolved the observed data loss. The LPUART1 RX FIFO also
buffers short servo-response bursts while UART4 interrupts are serviced. The
bring-up history is documented in
[`docs/servo_bus_bringup.md`](docs/servo_bus_bringup.md).

## Firmware architecture

```text
Core/Src/main.c
  |
  +-- CubeMX/HAL: clock, power, ICACHE, GPIO, EXTI, LPUART1, UART4
  +-- Nucleo BSP: USART1 Virtual COM Port and B1 user button
  |
  `-- App_Init(servo UART, E-Skin UART) / App_Process()
        |
        +-- app.c          demo sequencing, E-Skin pause/fault logic, telemetry
        +-- kinematics.c   calibration conversion, FK, IK, Cartesian movement
        +-- control.c      resolved-rate Cartesian P step and optional joint-tick PID
        +-- operations.c   transforms, optimized matrix math, 3x3 inversion
        +-- servo.c        STS3215 packets, feedback, limits, torque, homing
        `-- uart.c         servo transport and interrupt-driven E-Skin parser
```

### Servo communication and joint safety

The servo module implements the packet-level operations required by the
application:

- ping and status-packet validation;
- torque enable and disable;
- present-position reads from register `0x38`;
- position, speed, and acceleration writes beginning at register `0x29`;
- synchronous position reads and writes for the four-joint control cycle;
- header, response ID, checksum, and servo-status validation;
- bounded TX/RX timeouts, stale-RX draining, and retry handling;
- centralized ID-to-joint mapping, home values, fixed-joint flags, and raw
  position limits.

Single-servo operations are addressed to individual IDs. The resolved-rate
motion path uses the STS3215 broadcast ID only for protocol-defined synchronous
read and write commands after validating every participating ID and target.
`Servo_WritePosition()` rejects unknown or fixed joints and refuses targets
outside the configured raw range before generating a bus packet.

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
| Cartesian position tolerance | 0.001 m |
| Finite-difference step | 0.5 deg |
| Damping factor | 0.02 |
| Maximum joint update per iteration | 5 deg |

The solver reports `TARGET_NOT_REACHED` for an unreachable or numerically
unsolved target rather than issuing a partially validated Cartesian move.

## Closed-loop controllers

The final application uses Cartesian resolved-rate P control. Every 20 ms, the
firmware synchronously reads the four active joints, reconstructs the model-based
TCP position, and calls `Control_ResolvedRate_CalculateStep()`. This HAL-free
function calculates the Cartesian error and a speed-limited P step using
`Kp = 2.0 s^-1` and a maximum TCP speed of `0.1 m/s`. The kinematics module maps
the resulting Cartesian increment to joint increments through one damped
least-squares step, applies joint limits, and sends all four targets in one
synchronous packet.

An additional joint-tick PID implementation remains available through
`Kinematics_MoveEndEffectorToPositionJointTickPid()`. It regulates the raw-tick
error of each active motor independently, applies derivative filtering and
conditional integration, and limits each update to 100 ticks. This optional
path is included in the source but is not selected by the final application or
used in the reported baseline-versus-resolved-rate comparison.

## E-Skin integration and motion response

The servo bus, E-Skin input, and debug output use three independent UART
peripherals. Servo requests and responses run on LPUART1, E-Skin bytes arrive
asynchronously on UART4, and `printf()` diagnostics are routed through USART1 to
the ST-LINK Virtual COM Port. UART4 interrupts can therefore collect sensor data
while the foreground controller is communicating with the servos or logging.

The application treats E-Skin availability as a prerequisite for motion:

| Parameter | Value | Behavior |
| --- | ---: | --- |
| Startup sample timeout | 2,000 ms | Enter latched `FAULT` before the first motion command if no valid sample arrives |
| Runtime sample timeout | 1,000 ms | Stop motion and enter latched `FAULT` if valid data becomes stale |
| Stop threshold | 0.050 | Pause an active square segment and command the measured joint positions as hold targets |
| Clear threshold | 0.020 | Begin the resume qualification below this value |
| Clear stability time | 500 ms | Resume the interrupted segment only after the value remains clear for the full interval |

Separate stop and clear thresholds provide hysteresis, preventing rapid
pause/resume switching near one threshold. While paused, the servos remain
powered and hold their measured positions. A sensor timeout or a failed hold
command latches `FAULT`; it does not automatically resume. Detection and the
resulting stop are software-based and depend on UART scheduling, the current
servo transaction, and servo response time.

## Square TCP application

The default application in [`App/Src/app.c`](App/Src/app.c) is an autonomous
Cartesian demonstration:

1. Initialize the servo transport, calibrated joint table, and interrupt-driven
   E-Skin reception.
2. Require a valid E-Skin sample within 2 s and verify that the stop threshold
   is not active.
3. Command joints 1-4 to the known start configuration
   `[2047, 1208, 2548, 2372]` ticks.
4. Wait for every joint to reach that configuration.
5. Convert the start configuration to angles and calculate the start TCP with
   forward kinematics.
6. Generate four corner targets around that TCP while keeping X constant.
7. Execute each target with the Cartesian resolved-rate P controller while
   supervising the E-Skin and B1 abort input.
8. After a proximity pause clears, recalculate and continue the interrupted
   target; otherwise repeat the four Cartesian segments continuously.

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

The demo uses servo speed `300`, acceleration `50`, a 1 mm Cartesian target
tolerance, a 20 ms resolved-rate step period, and a 20 s timeout per corner.
The speed and acceleration values are STS3215 command units, not SI units.

Pressing the Nucleo B1 user button sets an abort flag, stops the square sequence,
and disables torque on all six joints. The interrupt handler only records the
event; debouncing and servo communication occur in application context.

> [!WARNING]
> B1 is a software convenience stop. It is not a certified emergency stop and
> does not make the arm electrically or mechanically safe. Keep a physical means
> of disconnecting servo power available during every hardware test.

<!-- Suggested future asset: docs/images/square-path-demo.gif -->

## Error handling and operational boundaries

The firmware exposes failures through result codes, application states, and
diagnostic counters instead of silently discarding them. These mechanisms cover
transport errors, receive timeouts, invalid headers, unexpected IDs, checksum
failures, servo-reported errors, invalid positions, fixed or unknown joints,
invalid arguments, unreachable targets, motion aborts, malformed E-Skin frames,
sensor timeouts, and UART4 receive-restart failures.

The application defines `INIT`, `CHECKING_HOME`, `IDLE`, `PAUSED_SKIN`,
`UNLOCKING`, `HOMING`, and `FAULT` states. The current square-demo path actively
uses `INIT`, `IDLE`, `PAUSED_SKIN`, and `FAULT`; the additional states support
retained bring-up and homing workflows.

Current engineering boundaries are intentionally visible:

- servo transport uses bounded, blocking HAL UART calls rather than DMA;
- the controlled move is synchronous and occupies the foreground application
  until the target settles, aborts, or times out;
- E-Skin reception is interrupt-driven, but parsing and motion response run in
  the cooperative foreground control path;
- IK controls position only and uses four active joints;
- wrist roll and gripper are fixed for the current demonstration;
- this firmware is a development and demonstration controller, not a
  functionally safe motion system.

## Repository structure

| Path | Purpose |
| --- | --- |
| `App/Inc`, `App/Src` | Handwritten application, E-Skin supervision, control, kinematics, operations, servo, and UART modules |
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

- [`docs/hardware.md`](docs/hardware.md) — power, servo/E-Skin wiring, UART
  settings, and hardware status;
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
targets, resolved-rate telemetry, E-Skin values and UART diagnostics, pause and
resume events, and motion failures. Logging uses USART1 independently of the
LPUART1 servo bus and UART4 E-Skin input.

## Validation status

- The current Debug target cross-compiles and links successfully with no build
  warnings emitted by the project sources.
- The latest local Debug build occupies approximately 76 KiB of Flash and
  3.0 KiB of RAM.
- LPUART1 loopback at 1 Mbit/s and the complete
  `STM32 -> Waveshare adapter -> STS3215` communication chain have been validated
  on real hardware.
- The `ESP32-C6 GPIO16 -> STM32 PC11/UART4_RX` E-Skin path was validated on real
  hardware on 2026-07-21. Unloaded values around `0.011` and proximity values
  above `0.8` were observed during that bring-up.
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
