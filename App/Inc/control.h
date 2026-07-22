/**
 ******************************************************************************
 * @file           : control.h
 * @author         : Sandro Canalicchio
 * @brief          : Pure controller calculations for Cartesian resolved-rate
 *                   motion and the optional joint-tick PID path.
 ******************************************************************************
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>

#define CONTROL_CARTESIAN_AXIS_COUNT 3U

typedef enum
{
    CONTROL_RESULT_OK = 0,
    CONTROL_RESULT_INVALID_ARGUMENT
} Control_Result_t;

typedef struct
{
    float kp_per_s;
    float max_speed_m_per_s;
    float position_tolerance_m;
} Control_ResolvedRateConfig_t;

typedef struct
{
    float error_m[CONTROL_CARTESIAN_AXIS_COUNT];
    float error_norm_m;
    float delta_position_m[CONTROL_CARTESIAN_AXIS_COUNT];
    uint8_t target_reached;
} Control_ResolvedRateStep_t;

typedef struct
{
    float integral;
    float previous_error;
    float filtered_derivative;
    uint8_t has_previous_error;
} Control_JointTickPid_t;

/**
 * @brief Loads the resolved-rate P-controller default tuning.
 * @param config Configuration structure to initialize; NULL is ignored.
 */
void Control_ResolvedRate_GetDefaultConfig(Control_ResolvedRateConfig_t *config);

/**
 * @brief Calculates one Cartesian P-controller step for resolved-rate motion.
 * @param config Controller gain, velocity limit, and target tolerance.
 * @param target_position_m Desired Cartesian XYZ position in meters.
 * @param current_position_m Current model-based XYZ position in meters.
 * @param dt_s Elapsed control-cycle time in seconds.
 * @param step Output containing error, bounded Cartesian step, and completion.
 * @return CONTROL_RESULT_OK, or CONTROL_RESULT_INVALID_ARGUMENT.
 */
Control_Result_t Control_ResolvedRate_CalculateStep(
    const Control_ResolvedRateConfig_t *config,
    const float target_position_m[CONTROL_CARTESIAN_AXIS_COUNT],
    const float current_position_m[CONTROL_CARTESIAN_AXIS_COUNT],
    float dt_s,
    Control_ResolvedRateStep_t *step);

/**
 * @brief Resets the dynamic state of one joint-tick PID instance.
 * @param pid Controller state to reset; NULL is accepted and ignored.
 */
void Control_JointTickPid_Reset(Control_JointTickPid_t *pid);

/**
 * @brief Calculates a bounded joint-position correction in servo ticks.
 * @param pid Persistent controller state for one joint.
 * @param target_ticks Desired joint position in servo ticks.
 * @param measured_ticks Measured joint position in servo ticks.
 * @param dt_s Elapsed time since the previous update in seconds.
 * @return Incremental joint command in servo ticks, or 0 for invalid input.
 */
float Control_JointTickPid_Update(Control_JointTickPid_t *pid,
                                  float target_ticks,
                                  float measured_ticks,
                                  float dt_s);

#endif /* CONTROL_H_ */
