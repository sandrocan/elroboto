#include "kin_benchmarks.h"

#include "kinematics.h"
#include "servo.h"
#include "uart.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* -------------------------------------------------------------------------- */
/* Private defines                                                            */
/* -------------------------------------------------------------------------- */

#define BENCHMARK_PI                         3.14159265358979323846f
#define BENCHMARK_DEG_TO_RAD                 (BENCHMARK_PI / 180.0f)
#define BENCHMARK_RAD_TO_DEG                 (180.0f / BENCHMARK_PI)

#define BENCHMARK_FK_SAMPLE_COUNT            8U
#define BENCHMARK_IK_TARGET_COUNT            5U
#define BENCHMARK_TRIG_SAMPLE_COUNT          16U

#define BENCHMARK_IK_MAX_ITERATIONS          80U
#define BENCHMARK_IK_TOLERANCE_M             0.005f
#define BENCHMARK_IK_FD_STEP_DEG             1.0f
#define BENCHMARK_IK_DAMPING                 0.0015f
#define BENCHMARK_IK_MAX_STEP_DEG            5.0f

#define BENCHMARK_UART_BUFFER_SIZE           192U

/* -------------------------------------------------------------------------- */
/* Private types                                                              */
/* -------------------------------------------------------------------------- */

typedef struct
{
    Operations_LinkPose_t pose;
    uint8_t active;
} Benchmark_Link_t;

typedef struct
{
    uint32_t iterations;
    uint32_t avg_cycles;
    uint32_t min_cycles;
    uint32_t max_cycles;
} Benchmark_Timing_t;

typedef struct
{
    Servo_Result_t result;
    uint16_t used_iterations;
} Benchmark_IkStatus_t;

/* -------------------------------------------------------------------------- */
/* Private variables                                                          */
/* -------------------------------------------------------------------------- */

/*
 * This chain mirrors the pure geometry from kinematics.c.
 * No servo communication is done here. If the robot geometry changes in
 * kinematics.c, keep this benchmark copy in sync.
 */
static const Benchmark_Link_t benchmark_chain[] =
{
    { { 0.0388353f,  -8.97657e-09f, 0.0624f,       BENCHMARK_PI,    4.18253e-17f, -BENCHMARK_PI }, 1U },
    { {-0.0303992f,  -0.0182778f,   -0.0542f,     -1.5708f,        -1.5708f,       0.0f          }, 1U },
    { {-0.11257f,    -0.028f,        1.73763e-16f,-3.63608e-16f,    8.74301e-16f,  1.5708f       }, 1U },
    { {-0.1349f,      0.0052f,       3.62355e-17f, 4.02456e-15f,    8.67362e-16f, -1.5708f       }, 1U },
    { { 5.55112e-17f,-0.0611f,       0.0181f,      1.5708f,         0.0486795f,    BENCHMARK_PI }, 0U },
    { {-0.0079f,     -0.000218121f, -0.0981274f,   0.0f,            BENCHMARK_PI,  0.0f          }, 0U }
};

static const float benchmark_trig_angles[BENCHMARK_TRIG_SAMPLE_COUNT] =
{
    -3.14159265f,
    -2.35619449f,
    -1.57079633f,
    -0.78539816f,
    -0.10000000f,
     0.0f,
     0.10000000f,
     0.52359878f,
     0.78539816f,
     1.04719755f,
     1.57079633f,
     2.35619449f,
     3.14159265f,
     4.71238898f,
     6.28318531f,
     7.85398163f
};

static const float benchmark_fk_joint_deg[BENCHMARK_FK_SAMPLE_COUNT][KINEMATICS_ACTIVE_JOINT_COUNT] =
{
    {   0.0f,   0.0f,   0.0f,   0.0f },
    {  10.0f, -15.0f,  20.0f, -10.0f },
    { -20.0f,  25.0f, -15.0f,  30.0f },
    {  45.0f, -35.0f,  40.0f, -25.0f },
    { -60.0f,  20.0f,  35.0f,  15.0f },
    {   5.0f,   2.0f,  -3.0f,   1.0f },
    {  85.0f, -65.0f,  55.0f, -45.0f },
    { -90.0f,  60.0f, -50.0f,  40.0f }
};

static const float benchmark_ik_seed_deg[KINEMATICS_ACTIVE_JOINT_COUNT] =
{
    0.0f,
    0.0f,
    0.0f,
    0.0f
};

static const float benchmark_ik_target_joint_deg[BENCHMARK_IK_TARGET_COUNT][KINEMATICS_ACTIVE_JOINT_COUNT] =
{
    {   0.0f,   0.0f,   0.0f,   0.0f },
    {  10.0f, -15.0f,  20.0f, -10.0f },
    { -20.0f,  20.0f, -15.0f,  15.0f },
    {  30.0f, -25.0f,  30.0f, -20.0f },
    { -35.0f,  30.0f, -25.0f,  20.0f }
};

/* Soft math limits only for the isolated benchmark IK. No servo config read. */
static const float benchmark_joint_min_deg[KINEMATICS_ACTIVE_JOINT_COUNT] =
{
    -115.0f,
    -110.0f,
    -100.0f,
    -95.0f
};

static const float benchmark_joint_max_deg[KINEMATICS_ACTIVE_JOINT_COUNT] =
{
    115.0f,
    110.0f,
    95.0f,
    100.0f
};

static volatile float benchmark_sink;

/* -------------------------------------------------------------------------- */
/* Private function prototypes                                                */
/* -------------------------------------------------------------------------- */

static void benchmark_print(const char *text);
static void benchmark_print_header(const char *title);
static void benchmark_cycle_counter_init(void);
static uint32_t benchmark_cycle_counter_read(void);
static uint8_t benchmark_cycle_counter_available(void);

static void benchmark_timing_reset(Benchmark_Timing_t *timing, uint32_t iterations);
static void benchmark_timing_add(Benchmark_Timing_t *timing, uint32_t cycles, uint64_t *sum_cycles);
static void benchmark_timing_finish(Benchmark_Timing_t *timing, uint64_t sum_cycles);

static float benchmark_absf(float value);
static float benchmark_maxf(float a, float b);
static void benchmark_clampf(float *value, float min_value, float max_value);
static float benchmark_deg_to_rad(float deg);
static float benchmark_rad_to_deg(float rad);
static float benchmark_position_distance(const Kinematics_Position_t *a, const Kinematics_Position_t *b);
static float benchmark_position_distance_squared(const Kinematics_Position_t *a, const Kinematics_Position_t *b);
static float benchmark_transform_position_error(const Kinematics_Transform_t *a, const Kinematics_Transform_t *b);
static float benchmark_transform_rotation_error(const Kinematics_Transform_t *a, const Kinematics_Transform_t *b);
static float benchmark_matrix_max_abs_error(
    const float A[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    const float B[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE]
);

static Servo_Result_t benchmark_forward_rad(
    const float joint_rad[KINEMATICS_ACTIVE_JOINT_COUNT],
    Kinematics_Transform_t *transform,
    op_trig_mode_t trig_mode,
    op_matmul_mode_t matmul_mode
);

static Servo_Result_t benchmark_forward_deg(
    const float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT],
    Kinematics_Transform_t *transform,
    op_trig_mode_t trig_mode,
    op_matmul_mode_t matmul_mode
);

static Servo_Result_t benchmark_forward_position_deg(
    const float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT],
    Kinematics_Position_t *position,
    op_trig_mode_t trig_mode,
    op_matmul_mode_t matmul_mode
);

static Benchmark_IkStatus_t benchmark_inverse_position_deg(
    const Kinematics_Position_t *target_position,
    const float seed_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT],
    float result_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT],
    op_trig_mode_t trig_mode,
    op_matmul_mode_t matmul_mode
);

static void benchmark_measure_sin(
    op_trig_mode_t trig_mode,
    uint32_t iterations,
    Benchmark_Timing_t *timing,
    float *max_error
);

static void benchmark_measure_cos(
    op_trig_mode_t trig_mode,
    uint32_t iterations,
    Benchmark_Timing_t *timing,
    float *max_error
);

static void benchmark_measure_matmul(
    op_matmul_mode_t matmul_mode,
    uint32_t iterations,
    Benchmark_Timing_t *timing,
    float *max_error
);

static void benchmark_measure_fk(
    op_trig_mode_t trig_mode,
    op_matmul_mode_t matmul_mode,
    uint32_t iterations,
    Benchmark_Timing_t *timing,
    float *max_pos_error,
    float *max_rot_error
);

static void benchmark_measure_ik(
    op_trig_mode_t trig_mode,
    op_matmul_mode_t matmul_mode,
    uint32_t iterations,
    Benchmark_Timing_t *timing,
    float *max_target_error,
    uint32_t *fail_count,
    float *avg_ik_iterations
);

/* -------------------------------------------------------------------------- */
/* Public functions                                                           */
/* -------------------------------------------------------------------------- */

void Tests_Benchmarks(void)
{
	const kin_benchmark_config_t benchmark_configs[] =
	{
		{
			.name = "baseline_standard_baseline_matmul",
			.trig_mode = OP_TRIG_STANDARD,
			.matmul_mode = OP_MATMUL_BASELINE,
			.use_direct_formula = 0U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "standard_optimized_matmul",
			.trig_mode = OP_TRIG_STANDARD,
			.matmul_mode = OP_MATMUL_OPTIMIZED,
			.use_direct_formula = 0U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "standard_homogeneous_matmul",
			.trig_mode = OP_TRIG_STANDARD,
			.matmul_mode = OP_MATMUL_HOMOGENEOUS,
			.use_direct_formula = 0U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "lookup_baseline_matmul",
			.trig_mode = OP_TRIG_LOOKUP,
			.matmul_mode = OP_MATMUL_BASELINE,
			.use_direct_formula = 0U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "lookup_optimized_matmul",
			.trig_mode = OP_TRIG_LOOKUP,
			.matmul_mode = OP_MATMUL_OPTIMIZED,
			.use_direct_formula = 0U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "lookup_homogeneous_matmul",
			.trig_mode = OP_TRIG_LOOKUP,
			.matmul_mode = OP_MATMUL_HOMOGENEOUS,
			.use_direct_formula = 0U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "arm_fast_baseline_matmul",
			.trig_mode = OP_TRIG_ARM_FAST,
			.matmul_mode = OP_MATMUL_BASELINE,
			.use_direct_formula = 0U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "arm_fast_optimized_matmul",
			.trig_mode = OP_TRIG_ARM_FAST,
			.matmul_mode = OP_MATMUL_OPTIMIZED,
			.use_direct_formula = 0U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "arm_fast_homogeneous_matmul",
			.trig_mode = OP_TRIG_ARM_FAST,
			.matmul_mode = OP_MATMUL_HOMOGENEOUS,
			.use_direct_formula = 0U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},

		/*
		 * Now the same setups again, but with direct formula enabled.
		 * This is useful to see if skipping the generic transform chain helps.
		 */
		{
			.name = "direct_standard_baseline_matmul",
			.trig_mode = OP_TRIG_STANDARD,
			.matmul_mode = OP_MATMUL_BASELINE,
			.use_direct_formula = 1U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "direct_standard_optimized_matmul",
			.trig_mode = OP_TRIG_STANDARD,
			.matmul_mode = OP_MATMUL_OPTIMIZED,
			.use_direct_formula = 1U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "direct_standard_homogeneous_matmul",
			.trig_mode = OP_TRIG_STANDARD,
			.matmul_mode = OP_MATMUL_HOMOGENEOUS,
			.use_direct_formula = 1U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "direct_lookup_baseline_matmul",
			.trig_mode = OP_TRIG_LOOKUP,
			.matmul_mode = OP_MATMUL_BASELINE,
			.use_direct_formula = 1U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "direct_lookup_optimized_matmul",
			.trig_mode = OP_TRIG_LOOKUP,
			.matmul_mode = OP_MATMUL_OPTIMIZED,
			.use_direct_formula = 1U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "direct_lookup_homogeneous_matmul",
			.trig_mode = OP_TRIG_LOOKUP,
			.matmul_mode = OP_MATMUL_HOMOGENEOUS,
			.use_direct_formula = 1U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "direct_arm_fast_baseline_matmul",
			.trig_mode = OP_TRIG_ARM_FAST,
			.matmul_mode = OP_MATMUL_BASELINE,
			.use_direct_formula = 1U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "direct_arm_fast_optimized_matmul",
			.trig_mode = OP_TRIG_ARM_FAST,
			.matmul_mode = OP_MATMUL_OPTIMIZED,
			.use_direct_formula = 1U,
			.iterations = APP_BENCHMARK_ITERATIONS
		},
		{
			.name = "direct_arm_fast_homogeneous_matmul",
			.trig_mode = OP_TRIG_ARM_FAST,
			.matmul_mode = OP_MATMUL_HOMOGENEOUS,
			.use_direct_formula = 1U,
			.iterations = APP_BENCHMARK_ITERATIONS
		}
	};

	UartDebug_SendString("\r\n");
	UartDebug_SendString("==================================================\r\n");
	UartDebug_SendString("elroboto full benchmark app booted\r\n");
	UartDebug_SendString("no servo movement will be started here\r\n");
	UartDebug_SendString("==================================================\r\n");
	UartDebug_SendString("\r\n");

	/*
	 * First test all trigonometry backends alone.
	 * This tells us how expensive sin/cos are without FK or IK around it.
	 */
	UartDebug_SendString("\r\n--- single trig benchmarks ---\r\n");
	KinematicBenchmark_RunSinTest(OP_TRIG_STANDARD);
	KinematicBenchmark_RunSinTest(OP_TRIG_LOOKUP);
	KinematicBenchmark_RunSinTest(OP_TRIG_ARM_FAST);

	KinematicBenchmark_RunCosTest(OP_TRIG_STANDARD);
	KinematicBenchmark_RunCosTest(OP_TRIG_LOOKUP);
	KinematicBenchmark_RunCosTest(OP_TRIG_ARM_FAST);

	/*
	 * Now test all matrix multiplication backends alone.
	 * This isolates the matrix part from the kinematics code.
	 */
	UartDebug_SendString("\r\n--- single matrix benchmarks ---\r\n");
	KinematicBenchmark_RunMatMulTest(OP_MATMUL_BASELINE);
	KinematicBenchmark_RunMatMulTest(OP_MATMUL_OPTIMIZED);
	KinematicBenchmark_RunMatMulTest(OP_MATMUL_HOMOGENEOUS);

	/*
	 * Now run FK for every trig and matrix combination.
	 * These tests still use only dummy joint angles from the benchmark code.
	 */
	UartDebug_SendString("\r\n--- single FK benchmarks ---\r\n");

	KinematicBenchmark_RunFKTest(OP_TRIG_STANDARD, OP_MATMUL_BASELINE);
	KinematicBenchmark_RunFKTest(OP_TRIG_STANDARD, OP_MATMUL_OPTIMIZED);
	KinematicBenchmark_RunFKTest(OP_TRIG_STANDARD, OP_MATMUL_HOMOGENEOUS);

	KinematicBenchmark_RunFKTest(OP_TRIG_LOOKUP, OP_MATMUL_BASELINE);
	KinematicBenchmark_RunFKTest(OP_TRIG_LOOKUP, OP_MATMUL_OPTIMIZED);
	KinematicBenchmark_RunFKTest(OP_TRIG_LOOKUP, OP_MATMUL_HOMOGENEOUS);

	KinematicBenchmark_RunFKTest(OP_TRIG_ARM_FAST, OP_MATMUL_BASELINE);
	KinematicBenchmark_RunFKTest(OP_TRIG_ARM_FAST, OP_MATMUL_OPTIMIZED);
	KinematicBenchmark_RunFKTest(OP_TRIG_ARM_FAST, OP_MATMUL_HOMOGENEOUS);

	/*
	 * Now run IK for every trig and matrix combination.
	 * This uses dummy targets and does not write servo positions.
	 */
	UartDebug_SendString("\r\n--- single IK benchmarks ---\r\n");

	KinematicBenchmark_RunIKTest(OP_TRIG_STANDARD, OP_MATMUL_BASELINE);
	KinematicBenchmark_RunIKTest(OP_TRIG_STANDARD, OP_MATMUL_OPTIMIZED);
	KinematicBenchmark_RunIKTest(OP_TRIG_STANDARD, OP_MATMUL_HOMOGENEOUS);

	KinematicBenchmark_RunIKTest(OP_TRIG_LOOKUP, OP_MATMUL_BASELINE);
	KinematicBenchmark_RunIKTest(OP_TRIG_LOOKUP, OP_MATMUL_OPTIMIZED);
	KinematicBenchmark_RunIKTest(OP_TRIG_LOOKUP, OP_MATMUL_HOMOGENEOUS);

	KinematicBenchmark_RunIKTest(OP_TRIG_ARM_FAST, OP_MATMUL_BASELINE);
	KinematicBenchmark_RunIKTest(OP_TRIG_ARM_FAST, OP_MATMUL_OPTIMIZED);
	KinematicBenchmark_RunIKTest(OP_TRIG_ARM_FAST, OP_MATMUL_HOMOGENEOUS);

	/*
	 * Now run the config-style benchmark table.
	 * This gives one clean named result per complete setup.
	 */
	UartDebug_SendString("\r\n--- full config benchmarks ---\r\n");

	for (uint8_t i = 0U; i < (uint8_t)(sizeof(benchmark_configs) / sizeof(benchmark_configs[0])); i++)
	{
		KinematicBenchmark_RunConfig(&benchmark_configs[i]);
	}

	UartDebug_SendString("\r\n");
	UartDebug_SendString("==================================================\r\n");
	UartDebug_SendString("all kinematics benchmarks finished\r\n");
	UartDebug_SendString("App_Process can now stay idle\r\n");
	UartDebug_SendString("==================================================\r\n");
	UartDebug_SendString("\r\n");

}


void KinematicBenchmark_RunSinTest(op_trig_mode_t trig_mode)
{
    Benchmark_Timing_t baseline_timing;
    Benchmark_Timing_t test_timing;
    float baseline_error = 0.0f;
    float test_error = 0.0f;
    float speedup = 0.0f;
    char text[BENCHMARK_UART_BUFFER_SIZE];

    benchmark_cycle_counter_init();

    benchmark_measure_sin(OP_TRIG_STANDARD, KIN_BENCHMARK_DEFAULT_MICRO_ITERATIONS, &baseline_timing, &baseline_error);
    benchmark_measure_sin(trig_mode, KIN_BENCHMARK_DEFAULT_MICRO_ITERATIONS, &test_timing, &test_error);

    if (test_timing.avg_cycles > 0U)
    {
        speedup = (float)baseline_timing.avg_cycles / (float)test_timing.avg_cycles;
    }

    benchmark_print_header("SIN BENCHMARK");
    benchmark_print("method, iterations, avg_cycles, min_cycles, max_cycles, baseline_avg_cycles, speedup, max_error\r\n");

    snprintf(
        text,
        sizeof(text),
        "%s, %lu, %lu, %lu, %lu, %lu, %.3f, %.9f\r\n",
        op_trig_mode_name(trig_mode),
        (unsigned long)test_timing.iterations,
        (unsigned long)test_timing.avg_cycles,
        (unsigned long)test_timing.min_cycles,
        (unsigned long)test_timing.max_cycles,
        (unsigned long)baseline_timing.avg_cycles,
        speedup,
        test_error
    );

    benchmark_print(text);
}

void KinematicBenchmark_RunCosTest(op_trig_mode_t trig_mode)
{
    Benchmark_Timing_t baseline_timing;
    Benchmark_Timing_t test_timing;
    float baseline_error = 0.0f;
    float test_error = 0.0f;
    float speedup = 0.0f;
    char text[BENCHMARK_UART_BUFFER_SIZE];

    benchmark_cycle_counter_init();

    benchmark_measure_cos(OP_TRIG_STANDARD, KIN_BENCHMARK_DEFAULT_MICRO_ITERATIONS, &baseline_timing, &baseline_error);
    benchmark_measure_cos(trig_mode, KIN_BENCHMARK_DEFAULT_MICRO_ITERATIONS, &test_timing, &test_error);

    if (test_timing.avg_cycles > 0U)
    {
        speedup = (float)baseline_timing.avg_cycles / (float)test_timing.avg_cycles;
    }

    benchmark_print_header("COS BENCHMARK");
    benchmark_print("method, iterations, avg_cycles, min_cycles, max_cycles, baseline_avg_cycles, speedup, max_error\r\n");

    snprintf(
        text,
        sizeof(text),
        "%s, %lu, %lu, %lu, %lu, %lu, %.3f, %.9f\r\n",
        op_trig_mode_name(trig_mode),
        (unsigned long)test_timing.iterations,
        (unsigned long)test_timing.avg_cycles,
        (unsigned long)test_timing.min_cycles,
        (unsigned long)test_timing.max_cycles,
        (unsigned long)baseline_timing.avg_cycles,
        speedup,
        test_error
    );

    benchmark_print(text);
}

void KinematicBenchmark_RunMatMulTest(op_matmul_mode_t matmul_mode)
{
    Benchmark_Timing_t baseline_timing;
    Benchmark_Timing_t test_timing;
    float baseline_error = 0.0f;
    float test_error = 0.0f;
    float speedup = 0.0f;
    char text[BENCHMARK_UART_BUFFER_SIZE];

    benchmark_cycle_counter_init();

    benchmark_measure_matmul(OP_MATMUL_BASELINE, KIN_BENCHMARK_DEFAULT_MICRO_ITERATIONS, &baseline_timing, &baseline_error);
    benchmark_measure_matmul(matmul_mode, KIN_BENCHMARK_DEFAULT_MICRO_ITERATIONS, &test_timing, &test_error);

    if (test_timing.avg_cycles > 0U)
    {
        speedup = (float)baseline_timing.avg_cycles / (float)test_timing.avg_cycles;
    }

    benchmark_print_header("MATMUL BENCHMARK");
    benchmark_print("method, iterations, avg_cycles, min_cycles, max_cycles, baseline_avg_cycles, speedup, max_error\r\n");

    snprintf(
        text,
        sizeof(text),
        "%s, %lu, %lu, %lu, %lu, %lu, %.3f, %.9f\r\n",
        op_matmul_mode_name(matmul_mode),
        (unsigned long)test_timing.iterations,
        (unsigned long)test_timing.avg_cycles,
        (unsigned long)test_timing.min_cycles,
        (unsigned long)test_timing.max_cycles,
        (unsigned long)baseline_timing.avg_cycles,
        speedup,
        test_error
    );

    benchmark_print(text);
}

void KinematicBenchmark_RunFKTest(op_trig_mode_t trig_mode, op_matmul_mode_t matmul_mode)
{
    Benchmark_Timing_t baseline_timing;
    Benchmark_Timing_t test_timing;
    float baseline_pos_error = 0.0f;
    float baseline_rot_error = 0.0f;
    float test_pos_error = 0.0f;
    float test_rot_error = 0.0f;
    float speedup = 0.0f;
    char text[BENCHMARK_UART_BUFFER_SIZE];

    benchmark_cycle_counter_init();

    benchmark_measure_fk(OP_TRIG_STANDARD, OP_MATMUL_BASELINE, KIN_BENCHMARK_DEFAULT_FK_ITERATIONS, &baseline_timing, &baseline_pos_error, &baseline_rot_error);
    benchmark_measure_fk(trig_mode, matmul_mode, KIN_BENCHMARK_DEFAULT_FK_ITERATIONS, &test_timing, &test_pos_error, &test_rot_error);

    if (test_timing.avg_cycles > 0U)
    {
        speedup = (float)baseline_timing.avg_cycles / (float)test_timing.avg_cycles;
    }

    benchmark_print_header("FK BENCHMARK");
    benchmark_print("trig_mode, matmul_mode, iterations, avg_cycles, min_cycles, max_cycles, baseline_avg_cycles, speedup, max_pos_error_m, max_rot_matrix_error\r\n");

    snprintf(
        text,
        sizeof(text),
        "%s, %s, %lu, %lu, %lu, %lu, %lu, %.3f, %.9f, %.9f\r\n",
        op_trig_mode_name(trig_mode),
        op_matmul_mode_name(matmul_mode),
        (unsigned long)test_timing.iterations,
        (unsigned long)test_timing.avg_cycles,
        (unsigned long)test_timing.min_cycles,
        (unsigned long)test_timing.max_cycles,
        (unsigned long)baseline_timing.avg_cycles,
        speedup,
        test_pos_error,
        test_rot_error
    );

    benchmark_print(text);
}

void KinematicBenchmark_RunIKTest(op_trig_mode_t trig_mode, op_matmul_mode_t matmul_mode)
{
    Benchmark_Timing_t baseline_timing;
    Benchmark_Timing_t test_timing;
    float baseline_target_error = 0.0f;
    float test_target_error = 0.0f;
    float baseline_avg_iter = 0.0f;
    float test_avg_iter = 0.0f;
    uint32_t baseline_fail_count = 0U;
    uint32_t test_fail_count = 0U;
    float speedup = 0.0f;
    char text[BENCHMARK_UART_BUFFER_SIZE];

    benchmark_cycle_counter_init();

    benchmark_measure_ik(OP_TRIG_STANDARD, OP_MATMUL_BASELINE, KIN_BENCHMARK_DEFAULT_IK_ITERATIONS, &baseline_timing, &baseline_target_error, &baseline_fail_count, &baseline_avg_iter);
    benchmark_measure_ik(trig_mode, matmul_mode, KIN_BENCHMARK_DEFAULT_IK_ITERATIONS, &test_timing, &test_target_error, &test_fail_count, &test_avg_iter);

    if (test_timing.avg_cycles > 0U)
    {
        speedup = (float)baseline_timing.avg_cycles / (float)test_timing.avg_cycles;
    }

    benchmark_print_header("IK BENCHMARK");
    benchmark_print("trig_mode, matmul_mode, iterations, avg_cycles, min_cycles, max_cycles, baseline_avg_cycles, speedup, max_target_error_m, fail_count, avg_ik_iterations\r\n");

    snprintf(
        text,
        sizeof(text),
        "%s, %s, %lu, %lu, %lu, %lu, %lu, %.3f, %.9f, %lu, %.2f\r\n",
        op_trig_mode_name(trig_mode),
        op_matmul_mode_name(matmul_mode),
        (unsigned long)test_timing.iterations,
        (unsigned long)test_timing.avg_cycles,
        (unsigned long)test_timing.min_cycles,
        (unsigned long)test_timing.max_cycles,
        (unsigned long)baseline_timing.avg_cycles,
        speedup,
        test_target_error,
        (unsigned long)test_fail_count,
        test_avg_iter
    );

    benchmark_print(text);
}

void KinematicBenchmark_RunConfig(const kin_benchmark_config_t *config)
{
    Benchmark_Timing_t baseline_fk_timing;
    Benchmark_Timing_t test_fk_timing;
    Benchmark_Timing_t baseline_ik_timing;
    Benchmark_Timing_t test_ik_timing;
    float baseline_fk_pos_error = 0.0f;
    float baseline_fk_rot_error = 0.0f;
    float test_fk_pos_error = 0.0f;
    float test_fk_rot_error = 0.0f;
    float baseline_ik_target_error = 0.0f;
    float test_ik_target_error = 0.0f;
    float baseline_ik_avg_iter = 0.0f;
    float test_ik_avg_iter = 0.0f;
    uint32_t baseline_ik_fail_count = 0U;
    uint32_t test_ik_fail_count = 0U;
    uint32_t iterations;
    float fk_speedup = 0.0f;
    float ik_speedup = 0.0f;
    char text[BENCHMARK_UART_BUFFER_SIZE];

    if (config == NULL)
    {
        benchmark_print("KinematicBenchmark_RunConfig failed: config is NULL\r\n");
        return;
    }

    iterations = config->iterations;
    if (iterations == 0U)
    {
        iterations = KIN_BENCHMARK_DEFAULT_FK_ITERATIONS;
    }

    benchmark_cycle_counter_init();

    benchmark_measure_fk(OP_TRIG_STANDARD, OP_MATMUL_BASELINE, iterations, &baseline_fk_timing, &baseline_fk_pos_error, &baseline_fk_rot_error);
    benchmark_measure_fk(config->trig_mode, config->matmul_mode, iterations, &test_fk_timing, &test_fk_pos_error, &test_fk_rot_error);

    benchmark_measure_ik(OP_TRIG_STANDARD, OP_MATMUL_BASELINE, iterations, &baseline_ik_timing, &baseline_ik_target_error, &baseline_ik_fail_count, &baseline_ik_avg_iter);
    benchmark_measure_ik(config->trig_mode, config->matmul_mode, iterations, &test_ik_timing, &test_ik_target_error, &test_ik_fail_count, &test_ik_avg_iter);

    if (test_fk_timing.avg_cycles > 0U)
    {
        fk_speedup = (float)baseline_fk_timing.avg_cycles / (float)test_fk_timing.avg_cycles;
    }

    if (test_ik_timing.avg_cycles > 0U)
    {
        ik_speedup = (float)baseline_ik_timing.avg_cycles / (float)test_ik_timing.avg_cycles;
    }

    benchmark_print_header("KINEMATIC CONFIG BENCHMARK");

    snprintf(
        text,
        sizeof(text),
        "config=%s trig=%s matmul=%s direct_formula=%u iterations=%lu\r\n",
        (config->name != NULL) ? config->name : "unnamed",
        op_trig_mode_name(config->trig_mode),
        op_matmul_mode_name(config->matmul_mode),
        (unsigned int)config->use_direct_formula,
        (unsigned long)iterations
    );
    benchmark_print(text);

    benchmark_print("test, avg_cycles, min_cycles, max_cycles, baseline_avg_cycles, speedup, max_error, fail_count_or_rot_error, avg_ik_iterations\r\n");

    snprintf(
        text,
        sizeof(text),
        "fk, %lu, %lu, %lu, %lu, %.3f, %.9f, %.9f, 0.00\r\n",
        (unsigned long)test_fk_timing.avg_cycles,
        (unsigned long)test_fk_timing.min_cycles,
        (unsigned long)test_fk_timing.max_cycles,
        (unsigned long)baseline_fk_timing.avg_cycles,
        fk_speedup,
        test_fk_pos_error,
        test_fk_rot_error
    );
    benchmark_print(text);

    snprintf(
        text,
        sizeof(text),
        "ik, %lu, %lu, %lu, %lu, %.3f, %.9f, %lu, %.2f\r\n",
        (unsigned long)test_ik_timing.avg_cycles,
        (unsigned long)test_ik_timing.min_cycles,
        (unsigned long)test_ik_timing.max_cycles,
        (unsigned long)baseline_ik_timing.avg_cycles,
        ik_speedup,
        test_ik_target_error,
        (unsigned long)test_ik_fail_count,
        test_ik_avg_iter
    );
    benchmark_print(text);
}

/* -------------------------------------------------------------------------- */
/* Private benchmark measure functions                                        */
/* -------------------------------------------------------------------------- */

static void benchmark_measure_sin(
    op_trig_mode_t trig_mode,
    uint32_t iterations,
    Benchmark_Timing_t *timing,
    float *max_error)
{
    uint64_t sum_cycles = 0U;

    benchmark_timing_reset(timing, iterations);
    *max_error = 0.0f;

    for (uint8_t i = 0U; i < BENCHMARK_TRIG_SAMPLE_COUNT; i++)
    {
        benchmark_sink += op_sin(benchmark_trig_angles[i], trig_mode);
    }

    for (uint32_t i = 0U; i < iterations; i++)
    {
        const float angle = benchmark_trig_angles[i % BENCHMARK_TRIG_SAMPLE_COUNT];
        const float reference = op_sin_standard(angle);
        uint32_t start_cycles;
        uint32_t stop_cycles;
        float value;
        float error;

        start_cycles = benchmark_cycle_counter_read();
        value = op_sin(angle, trig_mode);
        stop_cycles = benchmark_cycle_counter_read();

        benchmark_sink += value;
        benchmark_timing_add(timing, stop_cycles - start_cycles, &sum_cycles);

        error = benchmark_absf(value - reference);
        *max_error = benchmark_maxf(*max_error, error);
    }

    benchmark_timing_finish(timing, sum_cycles);
}

static void benchmark_measure_cos(
    op_trig_mode_t trig_mode,
    uint32_t iterations,
    Benchmark_Timing_t *timing,
    float *max_error)
{
    uint64_t sum_cycles = 0U;

    benchmark_timing_reset(timing, iterations);
    *max_error = 0.0f;

    for (uint8_t i = 0U; i < BENCHMARK_TRIG_SAMPLE_COUNT; i++)
    {
        benchmark_sink += op_cos(benchmark_trig_angles[i], trig_mode);
    }

    for (uint32_t i = 0U; i < iterations; i++)
    {
        const float angle = benchmark_trig_angles[i % BENCHMARK_TRIG_SAMPLE_COUNT];
        const float reference = op_cos_standard(angle);
        uint32_t start_cycles;
        uint32_t stop_cycles;
        float value;
        float error;

        start_cycles = benchmark_cycle_counter_read();
        value = op_cos(angle, trig_mode);
        stop_cycles = benchmark_cycle_counter_read();

        benchmark_sink += value;
        benchmark_timing_add(timing, stop_cycles - start_cycles, &sum_cycles);

        error = benchmark_absf(value - reference);
        *max_error = benchmark_maxf(*max_error, error);
    }

    benchmark_timing_finish(timing, sum_cycles);
}

static void benchmark_measure_matmul(
    op_matmul_mode_t matmul_mode,
    uint32_t iterations,
    Benchmark_Timing_t *timing,
    float *max_error)
{
    static const float A[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE] =
    {
        {  0.8660254f, -0.5000000f,  0.0000000f,  0.1200000f },
        {  0.3535534f,  0.6123724f, -0.7071068f, -0.0400000f },
        {  0.3535534f,  0.6123724f,  0.7071068f,  0.0800000f },
        {  0.0000000f,  0.0000000f,  0.0000000f,  1.0000000f }
    };

    static const float B[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE] =
    {
        {  0.7071068f,  0.0000000f,  0.7071068f, -0.0300000f },
        {  0.0000000f,  1.0000000f,  0.0000000f,  0.0600000f },
        { -0.7071068f,  0.0000000f,  0.7071068f,  0.1100000f },
        {  0.0000000f,  0.0000000f,  0.0000000f,  1.0000000f }
    };

    uint64_t sum_cycles = 0U;
    float reference[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE];
    float result[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE];

    benchmark_timing_reset(timing, iterations);

    op_mat4_mul_baseline(A, B, reference);
    op_mat4_mul(A, B, result, matmul_mode);
    *max_error = benchmark_matrix_max_abs_error(reference, result);

    for (uint8_t i = 0U; i < 8U; i++)
    {
        op_mat4_mul(A, B, result, matmul_mode);
        benchmark_sink += result[0][3];
    }

    for (uint32_t i = 0U; i < iterations; i++)
    {
        uint32_t start_cycles;
        uint32_t stop_cycles;

        start_cycles = benchmark_cycle_counter_read();
        op_mat4_mul(A, B, result, matmul_mode);
        stop_cycles = benchmark_cycle_counter_read();

        benchmark_sink += result[i % KINEMATICS_MATRIX_SIZE][3];
        benchmark_timing_add(timing, stop_cycles - start_cycles, &sum_cycles);
    }

    benchmark_timing_finish(timing, sum_cycles);
}

static void benchmark_measure_fk(
    op_trig_mode_t trig_mode,
    op_matmul_mode_t matmul_mode,
    uint32_t iterations,
    Benchmark_Timing_t *timing,
    float *max_pos_error,
    float *max_rot_error)
{
    uint64_t sum_cycles = 0U;
    Kinematics_Transform_t transform;

    benchmark_timing_reset(timing, iterations);
    *max_pos_error = 0.0f;
    *max_rot_error = 0.0f;

    for (uint8_t i = 0U; i < BENCHMARK_FK_SAMPLE_COUNT; i++)
    {
        Kinematics_Transform_t reference;
        Kinematics_Transform_t tested;
        float pos_error;
        float rot_error;

        (void)benchmark_forward_deg(benchmark_fk_joint_deg[i], &reference, OP_TRIG_STANDARD, OP_MATMUL_BASELINE);
        (void)benchmark_forward_deg(benchmark_fk_joint_deg[i], &tested, trig_mode, matmul_mode);

        pos_error = benchmark_transform_position_error(&reference, &tested);
        rot_error = benchmark_transform_rotation_error(&reference, &tested);

        *max_pos_error = benchmark_maxf(*max_pos_error, pos_error);
        *max_rot_error = benchmark_maxf(*max_rot_error, rot_error);
    }

    for (uint8_t i = 0U; i < BENCHMARK_FK_SAMPLE_COUNT; i++)
    {
        (void)benchmark_forward_deg(benchmark_fk_joint_deg[i], &transform, trig_mode, matmul_mode);
        benchmark_sink += transform.m[0][3];
    }

    for (uint32_t i = 0U; i < iterations; i++)
    {
        const uint32_t sample_index = i % BENCHMARK_FK_SAMPLE_COUNT;
        uint32_t start_cycles;
        uint32_t stop_cycles;

        start_cycles = benchmark_cycle_counter_read();
        (void)benchmark_forward_deg(benchmark_fk_joint_deg[sample_index], &transform, trig_mode, matmul_mode);
        stop_cycles = benchmark_cycle_counter_read();

        benchmark_sink += transform.m[0][3] + transform.m[1][3] + transform.m[2][3];
        benchmark_timing_add(timing, stop_cycles - start_cycles, &sum_cycles);
    }

    benchmark_timing_finish(timing, sum_cycles);
}

static void benchmark_measure_ik(
    op_trig_mode_t trig_mode,
    op_matmul_mode_t matmul_mode,
    uint32_t iterations,
    Benchmark_Timing_t *timing,
    float *max_target_error,
    uint32_t *fail_count,
    float *avg_ik_iterations)
{
    uint64_t sum_cycles = 0U;
    uint32_t total_ik_iterations = 0U;
    uint32_t successful_count = 0U;
    Kinematics_Position_t targets[BENCHMARK_IK_TARGET_COUNT];

    benchmark_timing_reset(timing, iterations);
    *max_target_error = 0.0f;
    *fail_count = 0U;
    *avg_ik_iterations = 0.0f;

    for (uint8_t i = 0U; i < BENCHMARK_IK_TARGET_COUNT; i++)
    {
        (void)benchmark_forward_position_deg(benchmark_ik_target_joint_deg[i], &targets[i], OP_TRIG_STANDARD, OP_MATMUL_BASELINE);
    }

    for (uint8_t i = 0U; i < BENCHMARK_IK_TARGET_COUNT; i++)
    {
        float solved_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
        Benchmark_IkStatus_t status;

        status = benchmark_inverse_position_deg(&targets[i], benchmark_ik_seed_deg, solved_deg, trig_mode, matmul_mode);
        benchmark_sink += solved_deg[0];
        (void)status;
    }

    for (uint32_t i = 0U; i < iterations; i++)
    {
        const uint32_t target_index = i % BENCHMARK_IK_TARGET_COUNT;
        float solved_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
        Benchmark_IkStatus_t status;
        uint32_t start_cycles;
        uint32_t stop_cycles;

        start_cycles = benchmark_cycle_counter_read();
        status = benchmark_inverse_position_deg(&targets[target_index], benchmark_ik_seed_deg, solved_deg, trig_mode, matmul_mode);
        stop_cycles = benchmark_cycle_counter_read();

        benchmark_sink += solved_deg[0] + solved_deg[1] + solved_deg[2] + solved_deg[3];
        benchmark_timing_add(timing, stop_cycles - start_cycles, &sum_cycles);

        if (status.result != SERVO_RESULT_OK)
        {
            (*fail_count)++;
        }
        else
        {
            Kinematics_Position_t reconstructed;
            float error;

            (void)benchmark_forward_position_deg(solved_deg, &reconstructed, OP_TRIG_STANDARD, OP_MATMUL_BASELINE);
            error = benchmark_position_distance(&targets[target_index], &reconstructed);
            *max_target_error = benchmark_maxf(*max_target_error, error);
            total_ik_iterations += status.used_iterations;
            successful_count++;
        }
    }

    benchmark_timing_finish(timing, sum_cycles);

    if (successful_count > 0U)
    {
        *avg_ik_iterations = (float)total_ik_iterations / (float)successful_count;
    }
}

/* -------------------------------------------------------------------------- */
/* Private kinematics functions                                               */
/* -------------------------------------------------------------------------- */

static Servo_Result_t benchmark_forward_rad(
    const float joint_rad[KINEMATICS_ACTIVE_JOINT_COUNT],
    Kinematics_Transform_t *transform,
    op_trig_mode_t trig_mode,
    op_matmul_mode_t matmul_mode)
{
    Kinematics_Transform_t total;
    uint8_t active_index = 0U;

    if ((joint_rad == NULL) || (transform == NULL))
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    op_mat4_identity(total.m);

    for (uint8_t i = 0U; i < (uint8_t)(sizeof(benchmark_chain) / sizeof(benchmark_chain[0])); i++)
    {
        Kinematics_Transform_t link_transform;
        Kinematics_Transform_t next_total;
        float q = 0.0f;

        if (benchmark_chain[i].active != 0U)
        {
            if (active_index >= KINEMATICS_ACTIVE_JOINT_COUNT)
            {
                return SERVO_RESULT_UNKNOWN_JOINT_ID;
            }

            q = joint_rad[active_index];
            active_index++;
        }

        Operations_LinkTransformMode(&benchmark_chain[i].pose, q, &link_transform, trig_mode);
        op_mat4_mul(total.m, link_transform.m, next_total.m, matmul_mode);
        total = next_total;
    }

    *transform = total;
    return SERVO_RESULT_OK;
}

static Servo_Result_t benchmark_forward_deg(
    const float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT],
    Kinematics_Transform_t *transform,
    op_trig_mode_t trig_mode,
    op_matmul_mode_t matmul_mode)
{
    float joint_rad[KINEMATICS_ACTIVE_JOINT_COUNT];

    if ((joint_deg == NULL) || (transform == NULL))
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
    {
        joint_rad[i] = benchmark_deg_to_rad(joint_deg[i]);
    }

    return benchmark_forward_rad(joint_rad, transform, trig_mode, matmul_mode);
}

static Servo_Result_t benchmark_forward_position_deg(
    const float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT],
    Kinematics_Position_t *position,
    op_trig_mode_t trig_mode,
    op_matmul_mode_t matmul_mode)
{
    Servo_Result_t result;
    Kinematics_Transform_t transform;

    if ((joint_deg == NULL) || (position == NULL))
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    result = benchmark_forward_deg(joint_deg, &transform, trig_mode, matmul_mode);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    position->x = transform.m[0][3];
    position->y = transform.m[1][3];
    position->z = transform.m[2][3];

    return SERVO_RESULT_OK;
}

static Benchmark_IkStatus_t benchmark_inverse_position_deg(
    const Kinematics_Position_t *target_position,
    const float seed_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT],
    float result_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT],
    op_trig_mode_t trig_mode,
    op_matmul_mode_t matmul_mode)
{
    Benchmark_IkStatus_t status;
    float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
    const float tolerance_squared = BENCHMARK_IK_TOLERANCE_M * BENCHMARK_IK_TOLERANCE_M;

    status.result = SERVO_RESULT_TARGET_NOT_REACHED;
    status.used_iterations = 0U;

    if ((target_position == NULL) || (seed_joint_deg == NULL) || (result_joint_deg == NULL))
    {
        status.result = SERVO_RESULT_NULL_POINTER;
        return status;
    }

    for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
    {
        joint_deg[i] = seed_joint_deg[i];
        benchmark_clampf(&joint_deg[i], benchmark_joint_min_deg[i], benchmark_joint_max_deg[i]);
        result_joint_deg[i] = joint_deg[i];
    }

    for (uint16_t iteration = 0U; iteration < BENCHMARK_IK_MAX_ITERATIONS; iteration++)
    {
        Servo_Result_t result;
        Kinematics_Position_t current_position;
        float error[3];
        float error_squared;
        float jacobian[3][KINEMATICS_ACTIVE_JOINT_COUNT];
        float a[3][3];
        float a_inv[3][3];
        float v[3];

        status.used_iterations = (uint16_t)(iteration + 1U);

        result = benchmark_forward_position_deg(joint_deg, &current_position, trig_mode, matmul_mode);
        if (result != SERVO_RESULT_OK)
        {
            status.result = result;
            return status;
        }

        error[0] = target_position->x - current_position.x;
        error[1] = target_position->y - current_position.y;
        error[2] = target_position->z - current_position.z;

        error_squared = benchmark_position_distance_squared(target_position, &current_position);

        if (error_squared <= tolerance_squared)
        {
            for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
            {
                result_joint_deg[i] = joint_deg[i];
            }

            status.result = SERVO_RESULT_OK;
            return status;
        }

        for (uint8_t joint_index = 0U; joint_index < KINEMATICS_ACTIVE_JOINT_COUNT; joint_index++)
        {
            float trial_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
            float actual_step_deg;
            float actual_step_rad;
            Kinematics_Position_t trial_position;

            for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
            {
                trial_joint_deg[i] = joint_deg[i];
            }

            trial_joint_deg[joint_index] = joint_deg[joint_index] + BENCHMARK_IK_FD_STEP_DEG;

            if (trial_joint_deg[joint_index] > benchmark_joint_max_deg[joint_index])
            {
                trial_joint_deg[joint_index] = joint_deg[joint_index] - BENCHMARK_IK_FD_STEP_DEG;
            }

            benchmark_clampf(&trial_joint_deg[joint_index], benchmark_joint_min_deg[joint_index], benchmark_joint_max_deg[joint_index]);

            actual_step_deg = trial_joint_deg[joint_index] - joint_deg[joint_index];
            actual_step_rad = benchmark_deg_to_rad(actual_step_deg);

            if ((actual_step_rad > -0.000001f) && (actual_step_rad < 0.000001f))
            {
                jacobian[0][joint_index] = 0.0f;
                jacobian[1][joint_index] = 0.0f;
                jacobian[2][joint_index] = 0.0f;
            }
            else
            {
                result = benchmark_forward_position_deg(trial_joint_deg, &trial_position, trig_mode, matmul_mode);
                if (result != SERVO_RESULT_OK)
                {
                    status.result = result;
                    return status;
                }

                jacobian[0][joint_index] = (trial_position.x - current_position.x) / actual_step_rad;
                jacobian[1][joint_index] = (trial_position.y - current_position.y) / actual_step_rad;
                jacobian[2][joint_index] = (trial_position.z - current_position.z) / actual_step_rad;
            }
        }

        for (uint8_t row = 0U; row < 3U; row++)
        {
            for (uint8_t col = 0U; col < 3U; col++)
            {
                a[row][col] = 0.0f;

                for (uint8_t joint_index = 0U; joint_index < KINEMATICS_ACTIVE_JOINT_COUNT; joint_index++)
                {
                    a[row][col] += jacobian[row][joint_index] * jacobian[col][joint_index];
                }

                if (row == col)
                {
                    a[row][col] += BENCHMARK_IK_DAMPING * BENCHMARK_IK_DAMPING;
                }
            }
        }

        if (Operations_Invert3x3(a, a_inv) == 0U)
        {
            status.result = SERVO_RESULT_TARGET_NOT_REACHED;
            return status;
        }

        for (uint8_t row = 0U; row < 3U; row++)
        {
            v[row] = 0.0f;

            for (uint8_t col = 0U; col < 3U; col++)
            {
                v[row] += a_inv[row][col] * error[col];
            }
        }

        for (uint8_t joint_index = 0U; joint_index < KINEMATICS_ACTIVE_JOINT_COUNT; joint_index++)
        {
            float delta_rad = 0.0f;
            float delta_deg;

            for (uint8_t row = 0U; row < 3U; row++)
            {
                delta_rad += jacobian[row][joint_index] * v[row];
            }

            delta_deg = benchmark_rad_to_deg(delta_rad);
            benchmark_clampf(&delta_deg, -BENCHMARK_IK_MAX_STEP_DEG, BENCHMARK_IK_MAX_STEP_DEG);

            joint_deg[joint_index] += delta_deg;
            benchmark_clampf(&joint_deg[joint_index], benchmark_joint_min_deg[joint_index], benchmark_joint_max_deg[joint_index]);
        }
    }

    for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
    {
        result_joint_deg[i] = joint_deg[i];
    }

    status.result = SERVO_RESULT_TARGET_NOT_REACHED;
    return status;
}

/* -------------------------------------------------------------------------- */
/* Private helpers                                                            */
/* -------------------------------------------------------------------------- */

static void benchmark_print(const char *text)
{
    UartDebug_SendString(text);
}

static void benchmark_print_header(const char *title)
{
    char text[BENCHMARK_UART_BUFFER_SIZE];

    snprintf(text, sizeof(text), "\r\n=== %s ===\r\n", title);
    benchmark_print(text);

    if (benchmark_cycle_counter_available() == 0U)
    {
        benchmark_print("warning: DWT cycle counter not available, timing may be 0 or HAL tick based\r\n");
    }
}

static void benchmark_cycle_counter_init(void)
{
#if defined(CoreDebug) && defined(DWT) && defined(CoreDebug_DEMCR_TRCENA_Msk) && defined(DWT_CTRL_CYCCNTENA_Msk)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

static uint32_t benchmark_cycle_counter_read(void)
{
#if defined(CoreDebug) && defined(DWT) && defined(DWT_CTRL_CYCCNTENA_Msk)
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U)
    {
        return DWT->CYCCNT;
    }
#endif

    return HAL_GetTick();
}

static uint8_t benchmark_cycle_counter_available(void)
{
#if defined(CoreDebug) && defined(DWT) && defined(DWT_CTRL_CYCCNTENA_Msk)
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U)
    {
        return 1U;
    }
#endif

    return 0U;
}

static void benchmark_timing_reset(Benchmark_Timing_t *timing, uint32_t iterations)
{
    if (timing == NULL)
    {
        return;
    }

    timing->iterations = iterations;
    timing->avg_cycles = 0U;
    timing->min_cycles = 0xFFFFFFFFU;
    timing->max_cycles = 0U;
}

static void benchmark_timing_add(Benchmark_Timing_t *timing, uint32_t cycles, uint64_t *sum_cycles)
{
    if ((timing == NULL) || (sum_cycles == NULL))
    {
        return;
    }

    if (cycles < timing->min_cycles)
    {
        timing->min_cycles = cycles;
    }

    if (cycles > timing->max_cycles)
    {
        timing->max_cycles = cycles;
    }

    *sum_cycles += cycles;
}

static void benchmark_timing_finish(Benchmark_Timing_t *timing, uint64_t sum_cycles)
{
    if (timing == NULL)
    {
        return;
    }

    if (timing->iterations > 0U)
    {
        timing->avg_cycles = (uint32_t)(sum_cycles / timing->iterations);
    }

    if (timing->min_cycles == 0xFFFFFFFFU)
    {
        timing->min_cycles = 0U;
    }
}

static float benchmark_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float benchmark_maxf(float a, float b)
{
    return (a > b) ? a : b;
}

static void benchmark_clampf(float *value, float min_value, float max_value)
{
    if (value == NULL)
    {
        return;
    }

    if (*value < min_value)
    {
        *value = min_value;
    }
    else if (*value > max_value)
    {
        *value = max_value;
    }
}

static float benchmark_deg_to_rad(float deg)
{
    return deg * BENCHMARK_DEG_TO_RAD;
}

static float benchmark_rad_to_deg(float rad)
{
    return rad * BENCHMARK_RAD_TO_DEG;
}

static float benchmark_position_distance(const Kinematics_Position_t *a, const Kinematics_Position_t *b)
{
    return sqrtf(benchmark_position_distance_squared(a, b));
}

static float benchmark_position_distance_squared(const Kinematics_Position_t *a, const Kinematics_Position_t *b)
{
    float dx;
    float dy;
    float dz;

    if ((a == NULL) || (b == NULL))
    {
        return 0.0f;
    }

    dx = a->x - b->x;
    dy = a->y - b->y;
    dz = a->z - b->z;

    return (dx * dx) + (dy * dy) + (dz * dz);
}

static float benchmark_transform_position_error(const Kinematics_Transform_t *a, const Kinematics_Transform_t *b)
{
    Kinematics_Position_t pa;
    Kinematics_Position_t pb;

    if ((a == NULL) || (b == NULL))
    {
        return 0.0f;
    }

    pa.x = a->m[0][3];
    pa.y = a->m[1][3];
    pa.z = a->m[2][3];

    pb.x = b->m[0][3];
    pb.y = b->m[1][3];
    pb.z = b->m[2][3];

    return benchmark_position_distance(&pa, &pb);
}

static float benchmark_transform_rotation_error(const Kinematics_Transform_t *a, const Kinematics_Transform_t *b)
{
    float sum_squared = 0.0f;

    if ((a == NULL) || (b == NULL))
    {
        return 0.0f;
    }

    for (uint8_t row = 0U; row < 3U; row++)
    {
        for (uint8_t col = 0U; col < 3U; col++)
        {
            const float diff = a->m[row][col] - b->m[row][col];
            sum_squared += diff * diff;
        }
    }

    return sqrtf(sum_squared);
}

static float benchmark_matrix_max_abs_error(
    const float A[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    const float B[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE])
{
    float max_error = 0.0f;

    if ((A == NULL) || (B == NULL))
    {
        return 0.0f;
    }

    for (uint8_t row = 0U; row < KINEMATICS_MATRIX_SIZE; row++)
    {
        for (uint8_t col = 0U; col < KINEMATICS_MATRIX_SIZE; col++)
        {
            const float error = benchmark_absf(A[row][col] - B[row][col]);
            max_error = benchmark_maxf(max_error, error);
        }
    }

    return max_error;
}
