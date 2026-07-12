# Operations and Kinematics Benchmark Report

This document summarizes the operation variants used in `operations.c` / `operations.h` and documents the benchmark results from the latest STM32 run.

The benchmark run was executed in benchmark-only app mode. No servo motion was started. The logged values are measured in CPU cycles and compare each variant against the standard baseline implementation.

---

## 1. Purpose of the Operations Layer

The `operations` module collects the math functions that are used by the kinematics layer. The goal is to keep the kinematics code clean while allowing different implementation variants to be benchmarked against each other.

The main optimization targets are:

1. trigonometric functions used for rotation matrices,
2. 4x4 matrix multiplication used for homogeneous transforms,
3. helper functions used by forward and inverse kinematics.

---

## 2. Data Types and Mode Enums

### `Kinematics_Transform_t`

Stores one 4x4 homogeneous transformation matrix.

```c
typedef struct
{
    float m[4][4];
} Kinematics_Transform_t;
```

It is used to represent link transforms and composed robot poses.

### `Operations_LinkPose_t`

Stores a pose in a more human-readable form.

```c
typedef struct
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
} Operations_LinkPose_t;
```

### `op_trig_mode_t`

Selects the trigonometric backend.

```c
typedef enum
{
    OP_TRIG_STANDARD = 0,
    OP_TRIG_ARM_FAST,
    OP_TRIG_LOOKUP
} op_trig_mode_t;
```

### `op_matmul_mode_t`

Selects the matrix multiplication backend.

```c
typedef enum
{
    OP_MATMUL_BASELINE = 0,
    OP_MATMUL_OPTIMIZED,
    OP_MATMUL_HOMOGENEOUS
} op_matmul_mode_t;
```

---

## 3. Trigonometric Function Variants

### `op_sin_standard()` / `op_cos_standard()`

These functions call the standard C math implementation:

```c
sinf(x)
cosf(x)
```

They are used as the numerical reference. In the benchmark, all errors are measured relative to this implementation.

### `op_sin_arm_fast()` / `op_cos_arm_fast()`

These functions use CMSIS-DSP fast math when `OPERATIONS_USE_CMSIS_DSP` is enabled:

```c
arm_sin_f32(x)
arm_cos_f32(x)
```

If CMSIS-DSP is not enabled, the implementation falls back to the standard `sinf()` / `cosf()` path.

In the latest run, CMSIS-DSP was active. This can be seen because the isolated `arm_fast` trigonometry functions became faster than the standard implementation. The tradeoff is a small approximation error.

### `op_sin_lookup()` / `op_cos_lookup()`

These functions use a lookup table and interpolation. The idea is to avoid calling the standard math library repeatedly.

The lookup approach is approximate and introduces a small numerical error. In the measured run, it was also slower than the standard implementation, most likely because the table access, angle wrapping, and interpolation overhead are larger than the saved computation time on this target.

### `op_sin()` / `op_cos()`

These are dispatcher functions. They select the actual backend based on `op_trig_mode_t`.

Example:

```c
float s = op_sin(angle, OP_TRIG_ARM_FAST);
float c = op_cos(angle, OP_TRIG_ARM_FAST);
```

This allows FK and IK benchmarks to switch between trigonometric backends without changing the kinematics code itself.

---

## 4. Matrix Multiplication Variants

### `op_mat4_mul_baseline()`

This is the reference implementation for 4x4 matrix multiplication. It uses the straightforward nested-loop approach.

Characteristics:

- simple,
- easy to verify,
- numerically exact compared to itself,
- slowest implementation.

### `op_mat4_mul_optimized()`

This implementation reduces loop overhead and uses a more direct calculation of the 4x4 result entries.

Characteristics:

- same numerical result as baseline,
- significantly faster than the generic baseline,
- still treats the matrix as a general 4x4 matrix.

### `op_mat4_mul_homogeneous()`

This implementation uses the structure of homogeneous transformation matrices.

A typical homogeneous transform has this form:

```text
[ R00 R01 R02 tx ]
[ R10 R11 R12 ty ]
[ R20 R21 R22 tz ]
[  0   0   0  1 ]
```

Because the last row is known, not all 16 result entries need the full generic 4x4 multiplication. This saves operations.

Characteristics:

- fastest matrix backend in the benchmark,
- exact result compared to baseline in the measured cases,
- best fit for robot kinematics because FK mainly multiplies homogeneous transforms.

### `op_mat4_mul()`

Dispatcher function for matrix multiplication. It calls one of the matrix backends depending on `op_matmul_mode_t`.

---

## 5. Higher-Level Operation Functions

### `Operations_SetIdentity()`

Writes an identity matrix into a `Kinematics_Transform_t`.

Used whenever a fresh transform chain starts.

### `Operations_Multiply()`

Compatibility wrapper for matrix multiplication. It multiplies two transforms and stores the result.

### `Operations_LinkTransform()`

Creates a homogeneous transform for one robot link using the default operation backend.

### `Operations_LinkTransformMode()`

Creates a homogeneous transform for one link, but allows selecting the trigonometric backend explicitly.

This is important for benchmarking because the same FK structure can be evaluated with:

- standard trigonometry,
- CMSIS-DSP fast trigonometry,
- lookup-table trigonometry.

### `Operations_RoundToI32()`

Rounds a floating-point value to a signed 32-bit integer.

This is useful when float-based kinematic results need to be mapped to integer domains, for example encoder ticks or raw servo units.

### `Operations_Invert3x3()`

Computes the inverse of a 3x3 matrix.

This is useful for small matrix operations inside kinematics or numerical algorithms where a full general-purpose matrix library would be unnecessary.

### `op_trig_mode_name()` / `op_matmul_mode_name()`

Convert enum values into readable strings for UART output and benchmark logs.

---

## 6. Benchmark Configuration Structure

The benchmark configuration combines the important selectable variants into one struct.

```c
typedef struct
{
    const char *name;
    op_trig_mode_t trig_mode;
    op_matmul_mode_t matmul_mode;
    uint8_t use_direct_formula;
    uint32_t iterations;
} kin_benchmark_config_t;
```

### Fields

| Field | Meaning |
| --- | --- |
| `name` | Human-readable name printed in the benchmark output |
| `trig_mode` | Selects standard, CMSIS-DSP fast, or lookup trigonometry |
| `matmul_mode` | Selects baseline, optimized, or homogeneous matrix multiplication |
| `use_direct_formula` | Flag reserved for direct-formula variants |
| `iterations` | Number of benchmark repetitions |

In the current benchmark results, `direct_formula = 1` does not create a measurable difference compared to `direct_formula = 0`. This means that the measured execution path is effectively identical for both direct and non-direct config variants.

---

## 7. Benchmark Results

### 7.1 Single Trigonometry Benchmark

| function | method | iterations | avg_cycles | min_cycles | max_cycles | baseline_avg_cycles | speedup | max_error |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| sin | standard | 10000 | 214 | 100 | 352 | 214 | 1.000 | 0.000000000 |
| sin | lookup | 10000 | 235 | 230 | 319 | 214 | 0.911 | 0.000057876 |
| sin | arm_fast | 10000 | 172 | 170 | 223 | 214 | 1.244 | 0.000014544 |
| cos | standard | 10000 | 215 | 102 | 337 | 215 | 1.000 | 0.000000000 |
| cos | lookup | 10000 | 259 | 252 | 349 | 215 | 0.830 | 0.000057995 |
| cos | arm_fast | 10000 | 173 | 172 | 222 | 215 | 1.243 | 0.000014603 |

### Interpretation

The isolated CMSIS-DSP `arm_fast` functions are faster than the standard implementation:

- `arm_fast` sine: 172 cycles vs. standard 214 cycles,
- `arm_fast` cosine: 173 cycles vs. standard 215 cycles.

However, this isolated advantage does not fully transfer to the complete FK and IK benchmarks.

The lookup implementation is slower than standard and introduces a larger approximation error than CMSIS-DSP fast math.

---

### 7.2 Single Matrix Multiplication Benchmark

| method | iterations | avg_cycles | min_cycles | max_cycles | baseline_avg_cycles | speedup | max_error |
| --- | --- | --- | --- | --- | --- | --- | --- |
| baseline | 10000 | 4608 | 4582 | 4644 | 4608 | 1.000 | 0.000000000 |
| optimized | 10000 | 1422 | 1405 | 1475 | 4608 | 3.241 | 0.000000000 |
| homogeneous | 10000 | 1146 | 1143 | 1192 | 4608 | 4.021 | 0.000000000 |

### Interpretation

The homogeneous matrix multiplication is the strongest isolated optimization:

- baseline: 4608 cycles,
- optimized: 1422 cycles,
- homogeneous: 1146 cycles.

The homogeneous implementation reaches a speedup of 4.021x with zero measured numerical error.

---

### 7.3 Forward Kinematics Benchmark

| trig_mode | matmul_mode | iterations | avg_cycles | min_cycles | max_cycles | baseline_avg_cycles | speedup | max_pos_error_m | max_rot_matrix_error |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| standard | baseline | 1000 | 39142 | 38712 | 39598 | 39142 | 1.000 | 0.000000000 | 0.000000000 |
| standard | optimized | 1000 | 20328 | 19903 | 20800 | 39142 | 1.926 | 0.000000000 | 0.000000000 |
| standard | homogeneous | 1000 | 18889 | 18473 | 19355 | 39141 | 2.072 | 0.000000000 | 0.000000000 |
| lookup | baseline | 1000 | 43033 | 42990 | 43058 | 39142 | 0.910 | 0.000071013 | 0.000311417 |
| lookup | optimized | 1000 | 24224 | 24178 | 24261 | 39142 | 1.616 | 0.000071013 | 0.000311417 |
| lookup | homogeneous | 1000 | 22783 | 22746 | 22817 | 39142 | 1.718 | 0.000071013 | 0.000311417 |
| arm_fast | baseline | 1000 | 39744 | 39687 | 39784 | 39141 | 0.985 | 0.000019309 | 0.000091989 |
| arm_fast | optimized | 1000 | 20934 | 20882 | 20989 | 39142 | 1.870 | 0.000019309 | 0.000091989 |
| arm_fast | homogeneous | 1000 | 19495 | 19450 | 19544 | 39142 | 2.008 | 0.000019309 | 0.000091989 |

### Interpretation

The best FK result is:

| trig mode | matrix mode | avg cycles | speedup | position error | rotation error |
| --- | --- | ---: | ---: | ---: | ---: |
| `standard` | `homogeneous` | 18889 | 2.072x | 0.000000000 | 0.000000000 |

This means the best FK configuration is standard trigonometry with homogeneous matrix multiplication.

---

### 7.4 Inverse Kinematics Benchmark

| trig_mode | matmul_mode | iterations | avg_cycles | min_cycles | max_cycles | baseline_avg_cycles | speedup | max_target_error_m | fail_count | avg_ik_iterations |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| standard | baseline | 100 | 884852 | 39590 | 1650054 | 884856 | 1.000 | 0.004488843 | 0 | 5.20 |
| standard | optimized | 100 | 471044 | 20788 | 878837 | 884855 | 1.878 | 0.004488843 | 0 | 5.20 |
| standard | homogeneous | 100 | 438841 | 19344 | 818837 | 884855 | 2.016 | 0.004488843 | 0 | 5.20 |
| lookup | baseline | 100 | 974634 | 43884 | 1816845 | 884855 | 0.908 | 0.003470416 | 0 | 5.20 |
| lookup | optimized | 100 | 561007 | 25083 | 1046016 | 884854 | 1.577 | 0.003470416 | 0 | 5.20 |
| lookup | homogeneous | 100 | 528683 | 23635 | 985729 | 884853 | 1.674 | 0.003470416 | 0 | 5.20 |
| arm_fast | baseline | 100 | 901703 | 40573 | 1680884 | 884853 | 0.981 | 0.004518745 | 0 | 5.20 |
| arm_fast | optimized | 100 | 487991 | 21762 | 909868 | 884851 | 1.813 | 0.004518745 | 0 | 5.20 |
| arm_fast | homogeneous | 100 | 455745 | 20323 | 849773 | 884855 | 1.942 | 0.004518745 | 0 | 5.20 |

### Interpretation

The best IK result is:

| trig mode | matrix mode | avg cycles | speedup | target error | fail count | avg IK iterations |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `standard` | `homogeneous` | 438841 | 2.016x | 0.004488843 | 0 | 5.20 |

All IK variants completed with `fail_count = 0`, so every tested configuration was valid for the selected dummy targets.

---

### 7.5 Full Config Benchmark Summary

| config | direct | trig | matmul | fk_avg | fk_speedup | fk_error_m | fk_rot_error | ik_avg | ik_speedup | ik_error_m | ik_fail_count | avg_ik_iter |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| baseline_standard_baseline_matmul | 0 | standard | baseline | 39142 | 1.000 | 0.000000000 | 0.000000000 | 884854 | 1.000 | 0.004488843 | 0 | 5.20 |
| standard_optimized_matmul | 0 | standard | optimized | 20328 | 1.925 | 0.000000000 | 0.000000000 | 471044 | 1.878 | 0.004488843 | 0 | 5.20 |
| standard_homogeneous_matmul | 0 | standard | homogeneous | 18889 | 2.072 | 0.000000000 | 0.000000000 | 438841 | 2.016 | 0.004488843 | 0 | 5.20 |
| lookup_baseline_matmul | 0 | lookup | baseline | 43033 | 0.910 | 0.000071013 | 0.000311417 | 974635 | 0.908 | 0.003470416 | 0 | 5.20 |
| lookup_optimized_matmul | 0 | lookup | optimized | 24224 | 1.616 | 0.000071013 | 0.000311417 | 561007 | 1.577 | 0.003470416 | 0 | 5.20 |
| lookup_homogeneous_matmul | 0 | lookup | homogeneous | 22783 | 1.718 | 0.000071013 | 0.000311417 | 528685 | 1.674 | 0.003470416 | 0 | 5.20 |
| arm_fast_baseline_matmul | 0 | arm_fast | baseline | 39744 | 0.985 | 0.000019309 | 0.000091989 | 901701 | 0.981 | 0.004518745 | 0 | 5.20 |
| arm_fast_optimized_matmul | 0 | arm_fast | optimized | 20934 | 1.870 | 0.000019309 | 0.000091989 | 487990 | 1.813 | 0.004518745 | 0 | 5.20 |
| arm_fast_homogeneous_matmul | 0 | arm_fast | homogeneous | 19494 | 2.008 | 0.000019309 | 0.000091989 | 455744 | 1.942 | 0.004518745 | 0 | 5.20 |
| direct_standard_baseline_matmul | 1 | standard | baseline | 39141 | 1.000 | 0.000000000 | 0.000000000 | 884854 | 1.000 | 0.004488843 | 0 | 5.20 |
| direct_standard_optimized_matmul | 1 | standard | optimized | 20328 | 1.925 | 0.000000000 | 0.000000000 | 471044 | 1.878 | 0.004488843 | 0 | 5.20 |
| direct_standard_homogeneous_matmul | 1 | standard | homogeneous | 18889 | 2.072 | 0.000000000 | 0.000000000 | 438841 | 2.016 | 0.004488843 | 0 | 5.20 |
| direct_lookup_baseline_matmul | 1 | lookup | baseline | 43033 | 0.910 | 0.000071013 | 0.000311417 | 974634 | 0.908 | 0.003470416 | 0 | 5.20 |
| direct_lookup_optimized_matmul | 1 | lookup | optimized | 24224 | 1.616 | 0.000071013 | 0.000311417 | 561008 | 1.577 | 0.003470416 | 0 | 5.20 |
| direct_lookup_homogeneous_matmul | 1 | lookup | homogeneous | 22783 | 1.718 | 0.000071013 | 0.000311417 | 528685 | 1.674 | 0.003470416 | 0 | 5.20 |
| direct_arm_fast_baseline_matmul | 1 | arm_fast | baseline | 39744 | 0.985 | 0.000019309 | 0.000091989 | 901701 | 0.981 | 0.004518745 | 0 | 5.20 |
| direct_arm_fast_optimized_matmul | 1 | arm_fast | optimized | 20934 | 1.870 | 0.000019309 | 0.000091989 | 487990 | 1.813 | 0.004518745 | 0 | 5.20 |
| direct_arm_fast_homogeneous_matmul | 1 | arm_fast | homogeneous | 19495 | 2.008 | 0.000019309 | 0.000091989 | 455744 | 1.942 | 0.004518745 | 0 | 5.20 |

---

## 8. Final Configuration Decision

The benchmark results show that the matrix multiplication backend has the largest effect on complete kinematics performance.

The isolated matrix benchmark shows:

| Backend | Average cycles | Speedup |
| --- | ---: | ---: |
| baseline | 4608 | 1.000x |
| optimized | 1422 | 3.241x |
| homogeneous | 1146 | 4.021x |

The homogeneous matrix implementation is therefore the best matrix backend.

For trigonometry, the isolated CMSIS-DSP fast math functions are faster than standard math. However, in the complete FK and IK benchmarks, the standard trigonometric backend with homogeneous matrix multiplication gives the best overall result:

| Test | Best configuration | Average cycles | Speedup | Error |
| --- | --- | ---: | ---: | ---: |
| FK | `standard + homogeneous` | 18889 | 2.072x | 0.000000000 m |
| IK | `standard + homogeneous` | 438841 | 2.016x | 0.004488843 m |

Therefore, the recommended application configuration is:

```c
kin_benchmark_config_t final_config =
{
    .name = "final_standard_homogeneous",
    .trig_mode = OP_TRIG_STANDARD,
    .matmul_mode = OP_MATMUL_HOMOGENEOUS,
    .use_direct_formula = 0U,
    .iterations = 1000U
};
```

### Reasoning

This configuration is selected because it provides:

1. the best FK runtime,
2. the best IK runtime,
3. zero measured FK position and rotation error relative to the standard baseline,
4. stable IK behavior with `fail_count = 0`,
5. no dependency on approximation-based trigonometry for the final application path.

CMSIS-DSP fast trigonometry is useful as an isolated optimization and was successfully validated. However, the complete kinematics benchmark shows that it is not the best final application choice in the current implementation.

The lookup-table trigonometry is not recommended because it is slower than the standard implementation on this target and introduces a larger numerical error.

---

## 9. Conclusion

The most important optimization is not the trigonometric backend, but the matrix multiplication backend.

The final recommendation is:

```c
OP_TRIG_STANDARD
OP_MATMUL_HOMOGENEOUS
```

This gives about 2x speedup for both forward and inverse kinematics while keeping the FK result numerically identical to the baseline.
