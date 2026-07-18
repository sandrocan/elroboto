#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>

typedef struct
{
    float integral;
    float previous_error;
    float filtered_derivative;
    uint8_t has_previous_error;
} Control_PID_t;

void Control_Reset(Control_PID_t *pid);
float Control_Update(Control_PID_t *pid, float setpoint, float actual, float dt_s);

#endif /* CONTROL_H_ */
