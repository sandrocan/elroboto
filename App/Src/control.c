/**
 ******************************************************************************
 * @file           : control.c
 * @author         : Sandro Canalicchio
 * @brief          : Pure controller calculations for Cartesian resolved-rate
 *                   motion and an optional joint-tick PID path.
 *
 * The active resolved-rate controller calculates a bounded Cartesian position
 * increment from the current TCP error and measured cycle time. Robot-specific
 * forward kinematics, Jacobian inversion, limits, and servo access remain in
 * the kinematics module. The optional joint-tick PID keeps its dynamic state in
 * a caller-owned Control_JointTickPid_t instance.
 ******************************************************************************
 */

#include "control.h"

#include <math.h>
#include <stddef.h>

#define CONTROL_RESOLVED_RATE_DEFAULT_KP_PER_S              2.0f
#define CONTROL_RESOLVED_RATE_DEFAULT_MAX_SPEED_M_PER_S     0.1f
#define CONTROL_RESOLVED_RATE_DEFAULT_TOLERANCE_M           0.001f

#define CONTROL_JOINT_TICK_PID_KP                         1.0f
#define CONTROL_JOINT_TICK_PID_KI                         0.5f
#define CONTROL_JOINT_TICK_PID_KD                         0.05f
#define CONTROL_JOINT_TICK_PID_MAX_UPDATE_TICKS         100.0f
#define CONTROL_JOINT_TICK_PID_DERIVATIVE_TAU_S           0.05f

/**
 * @brief Limits one controller correction to the configured tick interval.
 * @param output Unbounded incremental joint command in servo ticks.
 * @return Incremental joint command clamped to
 *         +/-CONTROL_JOINT_TICK_PID_MAX_UPDATE_TICKS.
 */
static float Control_ClampJointTickUpdate(float output_ticks);

/**
 * @brief Loads the default tuning for the active Cartesian controller.
 * @param config Configuration structure to initialize; NULL is ignored.
 * @return None.
 */
void Control_ResolvedRate_GetDefaultConfig(Control_ResolvedRateConfig_t *config)
{
    if (config != NULL)
    {
        config->kp_per_s = CONTROL_RESOLVED_RATE_DEFAULT_KP_PER_S;
        config->max_speed_m_per_s =
            CONTROL_RESOLVED_RATE_DEFAULT_MAX_SPEED_M_PER_S;
        config->position_tolerance_m =
            CONTROL_RESOLVED_RATE_DEFAULT_TOLERANCE_M;
    }
}

/**
 * @brief Calculates error, completion state, and a bounded Cartesian step.
 * @param config Controller gain, speed limit, and target tolerance.
 * @param target_position_m Desired Cartesian XYZ position in meters.
 * @param current_position_m Current model-based XYZ position in meters.
 * @param dt_s Elapsed control-cycle time in seconds.
 * @param step Output containing the Cartesian control result.
 * @return CONTROL_RESULT_OK, or CONTROL_RESULT_INVALID_ARGUMENT.
 */
Control_Result_t Control_ResolvedRate_CalculateStep(
    const Control_ResolvedRateConfig_t *config,
    const float target_position_m[CONTROL_CARTESIAN_AXIS_COUNT],
    const float current_position_m[CONTROL_CARTESIAN_AXIS_COUNT],
    float dt_s,
    Control_ResolvedRateStep_t *step)
{
    float error_squared_m2 = 0.0f;
    float step_scale;
    float max_step_m;

    /* Reject invalid tuning, timing, or output storage before calculation. */
    if ((config == NULL) ||
        (target_position_m == NULL) ||
        (current_position_m == NULL) ||
        (step == NULL) ||
        (!isfinite(dt_s)) ||
        (dt_s <= 0.0f) ||
        (!isfinite(config->kp_per_s)) ||
        (config->kp_per_s <= 0.0f) ||
        (!isfinite(config->max_speed_m_per_s)) ||
        (config->max_speed_m_per_s <= 0.0f) ||
        (!isfinite(config->position_tolerance_m)) ||
        (config->position_tolerance_m <= 0.0f))
    {
        return CONTROL_RESULT_INVALID_ARGUMENT;
    }

    /* Calculate the Cartesian error vector and its squared Euclidean norm. */
    for (uint8_t axis = 0U; axis < CONTROL_CARTESIAN_AXIS_COUNT; axis++)
    {
        if ((!isfinite(target_position_m[axis])) ||
            (!isfinite(current_position_m[axis])))
        {
            return CONTROL_RESULT_INVALID_ARGUMENT;
        }

        step->error_m[axis] =
            target_position_m[axis] - current_position_m[axis];
        error_squared_m2 += step->error_m[axis] * step->error_m[axis];
        step->delta_position_m[axis] = 0.0f;
    }

    step->error_norm_m = sqrtf(error_squared_m2);
    step->target_reached =
        (step->error_norm_m <= config->position_tolerance_m) ? 1U : 0U;

    /* A reached target produces no further Cartesian correction. */
    if (step->target_reached != 0U)
    {
        return CONTROL_RESULT_OK;
    }

    /* Apply proportional feedback and limit the step by maximum TCP speed. */
    step_scale = config->kp_per_s * dt_s;
    max_step_m = config->max_speed_m_per_s * dt_s;

    if ((step->error_norm_m * step_scale) > max_step_m)
    {
        step_scale = max_step_m / step->error_norm_m;
    }

    /* Return the bounded Cartesian increment for the kinematics module. */
    for (uint8_t axis = 0U; axis < CONTROL_CARTESIAN_AXIS_COUNT; axis++)
    {
        step->delta_position_m[axis] = step->error_m[axis] * step_scale;
    }

    return CONTROL_RESULT_OK;
}

/**
 * @brief Resets all dynamic state of one PID controller instance.
 * @param pid Controller state to reset; NULL is accepted and ignored.
 * @return None.
 */
void Control_JointTickPid_Reset(Control_JointTickPid_t *pid)
{
    if (pid != NULL)
    {
        /* Start without stored integral action or derivative history. */
        pid->integral = 0.0f;
        pid->previous_error = 0.0f;
        pid->filtered_derivative = 0.0f;
        pid->has_previous_error = 0U;
    }
}

/**
 * @brief Calculates one bounded discrete PID correction.
 * @param pid Persistent controller state for one joint.
 * @param target_ticks Desired joint position in servo ticks.
 * @param measured_ticks Measured joint position in servo ticks.
 * @param dt_s Elapsed time since the previous update in seconds.
 * @return Incremental joint command in servo ticks, or 0 for invalid input.
 */
float Control_JointTickPid_Update(Control_JointTickPid_t *pid,
                                  float target_ticks,
                                  float measured_ticks,
                                  float dt_s)
{
    const float error_ticks = target_ticks - measured_ticks;
    float candidate_integral;
    float raw_derivative = 0.0f;
    float derivative = 0.0f;
    float unclamped_output;
    float clamped_output;

    if ((pid == NULL) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    /* Form an integral candidate; commit it only after saturation is known. */
    candidate_integral = pid->integral + (error_ticks * dt_s);

    /* Suppress the undefined first-sample derivative and filter later values. */
    if (pid->has_previous_error != 0U)
    {
        const float filter_alpha = dt_s /
                                   (CONTROL_JOINT_TICK_PID_DERIVATIVE_TAU_S + dt_s);

        raw_derivative = (error_ticks - pid->previous_error) / dt_s;
        pid->filtered_derivative += filter_alpha * (raw_derivative - pid->filtered_derivative);
        derivative = pid->filtered_derivative;
    }

    pid->previous_error = error_ticks;
    pid->has_previous_error = 1U;

    /* Combine the three terms and constrain the correction sent downstream. */
    unclamped_output = (CONTROL_JOINT_TICK_PID_KP * error_ticks) +
                       (CONTROL_JOINT_TICK_PID_KI * candidate_integral) +
                       (CONTROL_JOINT_TICK_PID_KD * derivative);
    clamped_output = Control_ClampJointTickUpdate(unclamped_output);

    /*
     * Conditional integration prevents windup while the output is saturated
     * in the same direction as the current error. Integration remains enabled
     * when it helps drive the controller back out of saturation.
     */
    if (!(((unclamped_output > CONTROL_JOINT_TICK_PID_MAX_UPDATE_TICKS) &&
           (error_ticks > 0.0f)) ||
          ((unclamped_output < -CONTROL_JOINT_TICK_PID_MAX_UPDATE_TICKS) &&
           (error_ticks < 0.0f))))
    {
        pid->integral = candidate_integral;
    }

    return clamped_output;
}

/**
 * @brief Limits one controller correction to the configured tick interval.
 * @param output_ticks Unbounded incremental joint command in servo ticks.
 * @return Incremental command clamped to the configured positive and negative limits.
 */
static float Control_ClampJointTickUpdate(float output_ticks)
{
    if (output_ticks > CONTROL_JOINT_TICK_PID_MAX_UPDATE_TICKS)
    {
        output_ticks = CONTROL_JOINT_TICK_PID_MAX_UPDATE_TICKS;
    }
    else if (output_ticks < -CONTROL_JOINT_TICK_PID_MAX_UPDATE_TICKS)
    {
        output_ticks = -CONTROL_JOINT_TICK_PID_MAX_UPDATE_TICKS;
    }

    return output_ticks;
}
