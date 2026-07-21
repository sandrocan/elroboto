#include "control.h"

#include <stddef.h>

#define CONTROL_KP    1.0f
#define CONTROL_KI    0.5f
#define CONTROL_KD    0.05f
#define CONTROL_MAX_UPDATE_STEP_TICKS 100.0f
#define CONTROL_DERIVATIVE_FILTER_TAU_S 0.05f

static float Control_ClampUpdateStepTicks(float output);

void Control_Reset(Control_PID_t *pid)
{
    if (pid != NULL)
    {
        pid->integral = 0.0f;
        pid->previous_error = 0.0f;
        pid->filtered_derivative = 0.0f;
        pid->has_previous_error = 0U;
    }
}

float Control_Update(Control_PID_t *pid, float setpoint, float actual, float dt_s)
{
    const float error = setpoint - actual;
    float candidate_integral;
    float raw_derivative = 0.0f;
    float derivative = 0.0f;
    float unclamped_output;
    float clamped_output;

    if ((pid == NULL) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    candidate_integral = pid->integral + (error * dt_s);

    if (pid->has_previous_error != 0U)
    {
        const float filter_alpha = dt_s / (CONTROL_DERIVATIVE_FILTER_TAU_S + dt_s);

        raw_derivative = (error - pid->previous_error) / dt_s;
        pid->filtered_derivative += filter_alpha * (raw_derivative - pid->filtered_derivative);
        derivative = pid->filtered_derivative;
    }

    pid->previous_error = error;
    pid->has_previous_error = 1U;

    unclamped_output = (CONTROL_KP * error) +
                       (CONTROL_KI * candidate_integral) +
                       (CONTROL_KD * derivative);
    clamped_output = Control_ClampUpdateStepTicks(unclamped_output);

    /*
     * Conditional integration prevents windup while the output is saturated
     * in the same direction as the current error. Integration remains enabled
     * when it helps drive the controller back out of saturation.
     */
    if (!(((unclamped_output > CONTROL_MAX_UPDATE_STEP_TICKS) && (error > 0.0f)) ||
          ((unclamped_output < -CONTROL_MAX_UPDATE_STEP_TICKS) && (error < 0.0f))))
    {
        pid->integral = candidate_integral;
    }

    return clamped_output;
}

static float Control_ClampUpdateStepTicks(float output)
{
    if (output > CONTROL_MAX_UPDATE_STEP_TICKS)
    {
        output = CONTROL_MAX_UPDATE_STEP_TICKS;
    }
    else if (output < -CONTROL_MAX_UPDATE_STEP_TICKS)
    {
        output = -CONTROL_MAX_UPDATE_STEP_TICKS;
    }

    return output;
}
