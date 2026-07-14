/**
 ******************************************************************************
 * @file           : kin_benchmarks.h / kin_benchmarks.c
 * @author         : Niklas Peter
 * @brief          : Manual benchmarks for pure kinematics math on STM32
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

typedef struct
{
    const char *name;
    op_trig_mode_t trig_mode;
    op_matmul_mode_t matmul_mode;
    uint8_t use_direct_formula;
    uint32_t iterations;
} kin_benchmark_config_t;

/**
 * @brief Runs all benchmark configurations and prints results in UART
 */
void Tests_Benchmarks(void);


void KinematicBenchmark_RunSinTest(op_trig_mode_t trig_mode);
void KinematicBenchmark_RunCosTest(op_trig_mode_t trig_mode);

void KinematicBenchmark_RunMatMulTest(op_matmul_mode_t matmul_mode);

void KinematicBenchmark_RunFKTest(
    op_trig_mode_t trig_mode,
    op_matmul_mode_t matmul_mode
);

void KinematicBenchmark_RunIKTest(
    op_trig_mode_t trig_mode,
    op_matmul_mode_t matmul_mode
);

void KinematicBenchmark_RunConfig(const kin_benchmark_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* KIN_BENCHMARKS_H_ */
