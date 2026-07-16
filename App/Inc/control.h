#ifndef CONTROL_H_
#define CONTROL_H_

typedef struct
{
    float integral;
    float previous_error;
} Control_PID_t;

void Control_Reset(Control_PID_t *pid);
float Control_Update(Control_PID_t *pid, float setpoint, float actual);

#endif /* CONTROL_H_ */
