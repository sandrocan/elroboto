/**
 ******************************************************************************
 * @file           : kin_benchmarks.h
 * @author         : Niklas Peter
 * @brief          : Manual performance benchmarks for the pure kinematics math
 *                      without any servo movement or hardware communication
 ******************************************************************************
 */

#ifndef KIN_BENCHMARKS_H_
#define KIN_BENCHMARKS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "op_benchmarks.h"
#include <stdint.h>

#define KIN_BENCHMARK_DEFAULT_MICRO_ITERATIONS  10000U
#define KIN_BENCHMARK_DEFAULT_FK_ITERATIONS      1000U
#define KIN_BENCHMARK_DEFAULT_IK_ITERATIONS       100U

//Complete setup for one named FK and IK benchmark configuration
typedef struct
{
    const char *name;
    op_trig_mode_t trig_mode;
    op_matmul_mode_t matmul_mode;
    uint8_t use_direct_formula;
    uint32_t iterations;
} kin_benchmark_config_t;

/* -------------------------------------------------------------------------- */
/* Public benchmark functions                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Runs all benchmark configurations and prints results in UART
 */
void Tests_Benchmarks(void);

/**
 * @brief Benchmarks one sine implementation against the standard implementation.
 * @param trig_mode Sine implementation that will be tested.
 */
void KinematicBenchmark_RunSinTest(op_trig_mode_t trig_mode);

/**
 * @brief Benchmarks one cosine implementation against the standard implementation.
 * @param trig_mode Cosine implementation that will be tested.
 */
void KinematicBenchmark_RunCosTest(op_trig_mode_t trig_mode);

/**
 * @brief Benchmarks one matrix multiplication implementation against the baseline.
 * @param matmul_mode Matrix multiplication implementation that will be tested.
 */
void KinematicBenchmark_RunMatMulTest(op_matmul_mode_t matmul_mode);

/**
 * @brief Benchmarks one complete forward kinematics configuration.
 * @param trig_mode Trigonometry implementation used by the benchmark.
 * @param matmul_mode Matrix multiplication implementation used by the benchmark.
 */
void KinematicBenchmark_RunFKTest(op_trig_mode_t trig_mode, op_matmul_mode_t matmul_mode
);

/**
 * @brief Benchmarks one complete inverse kinematics configuration.
 * @param trig_mode Trigonometry implementation used by the benchmark.
 * @param matmul_mode Matrix multiplication implementation used by the benchmark.
 */
void KinematicBenchmark_RunIKTest(op_trig_mode_t trig_mode, op_matmul_mode_t matmul_mode
);

/**
 * @brief Runs the forward and inverse kinematics benchmarks for one named setup.
 * @param config Pointer to the benchmark configuration that will be tested.
 */
void KinematicBenchmark_RunConfig(const kin_benchmark_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* KIN_BENCHMARKS_H_ */
