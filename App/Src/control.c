#include "control.h"

#include <stddef.h>

#define CONTROL_KP    1.0f
#define CONTROL_KI    0.0f
#define CONTROL_KD    0.0f
#define CONTROL_DT_S  0.05f

void Control_Reset(Control_PID_t *pid)
{
    if (pid != NULL)
    {
        pid->integral = 0.0f;
        pid->previous_error = 0.0f;
    }
}

float Control_Update(Control_PID_t *pid, float setpoint, float actual)
{
    const float error = setpoint - actual;
    float derivative = 0.0f;

    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->integral += error * CONTROL_DT_S;
    derivative = (error - pid->previous_error) / CONTROL_DT_S;
    pid->previous_error = error;

    return (CONTROL_KP * error) +
           (CONTROL_KI * pid->integral) +
           (CONTROL_KD * derivative);
}
