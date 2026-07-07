#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "servo.h"

#include <stdint.h>

#define KINEMATICS_ACTIVE_JOINT_COUNT   4U
#define KINEMATICS_MATRIX_SIZE          4U

#define KINEMATICS_TICKS_PER_REV        4096.0f
#define KINEMATICS_DEG_PER_REV          360.0f
#define KINEMATICS_TICKS_PER_DEG        (KINEMATICS_TICKS_PER_REV / KINEMATICS_DEG_PER_REV)
#define KINEMATICS_DEG_PER_TICK        (KINEMATICS_DEG_PER_REV / KINEMATICS_TICKS_PER_REV)

typedef struct
{
    float m[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE];
} Kinematics_Transform_t;

typedef struct
{
    float x;
    float y;
    float z;
} Kinematics_Position_t;

float Kinematics_DegToRad(float degrees);
float Kinematics_RadToDeg(float radians);

int32_t Kinematics_DegToTickDelta(float degrees);
float Kinematics_TickDeltaToDeg(int32_t ticks);

Servo_Result_t Kinematics_RawToAngleDeg(uint8_t joint_id, uint16_t raw_position, float *angle_deg);
Servo_Result_t Kinematics_AngleDegToRaw(uint8_t joint_id, float angle_deg, uint16_t *raw_position);

Servo_Result_t Kinematics_ForwardRad(const float joint_rad[KINEMATICS_ACTIVE_JOINT_COUNT], Kinematics_Transform_t *transform);
Servo_Result_t Kinematics_ForwardDeg(const float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT], Kinematics_Transform_t *transform);
Servo_Result_t Kinematics_GetPosition(const Kinematics_Transform_t *transform, Kinematics_Position_t *position);

Servo_Result_t Kinematics_ReadCurrentJointAnglesDeg(float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT]);
Servo_Result_t Kinematics_ReadCurrentEndEffector(Kinematics_Transform_t *transform);

Servo_Result_t Kinematics_MoveJointRelativeDeg(uint8_t joint_id, float delta_deg, uint16_t speed, uint8_t acceleration);
Servo_Result_t Kinematics_MoveJointRelativeDegAndWait(uint8_t joint_id, float delta_deg, uint16_t speed, uint8_t acceleration, uint16_t tolerance_ticks, uint32_t timeout_ms);
Servo_Result_t Kinematics_MoveJointToAngleDeg(uint8_t joint_id, float angle_deg, uint16_t speed, uint8_t acceleration);
Servo_Result_t Kinematics_MoveJointToAngleDegAndWait(uint8_t joint_id, float angle_deg, uint16_t speed, uint8_t acceleration, uint16_t tolerance_ticks, uint32_t timeout_ms);

Servo_Result_t Kinematics_WaitUntilJointReached(uint8_t joint_id, uint16_t target_raw, uint16_t tolerance_ticks, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H_ */
