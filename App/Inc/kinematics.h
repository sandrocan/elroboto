/**
 ******************************************************************************
 * @file           : kinematics.c /kinematics.h
 * @author         : Niklas Peter
 * @brief          : All functions regarding direct and inverse kinematics
 ******************************************************************************
 */

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

/**
 * @brief Stores a 4x4 homogeneous transformation matrix.
 */
typedef struct
{
    float m[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE];
} Kinematics_Transform_t;

/**
 * @brief Stores a 3D Cartesian position.
 */
typedef struct
{
    float x;
    float y;
    float z;
} Kinematics_Position_t;

/**
 * @brief Converts an angle from degrees to radians.
 * @param degrees Angle in degrees.
 * @return Angle in radians.
 */
float Kinematics_DegToRad(float degrees);

/**
 * @brief Converts an angle from radians to degrees.
 * @param radians Angle in radians.
 * @return Angle in degrees.
 */
float Kinematics_RadToDeg(float radians);

/**
 * @brief Converts a relative angle in degrees to a relative servo tick delta.
 * @param degrees Relative angle in degrees.
 * @return Relative encoder tick difference.
 */
int32_t Kinematics_DegToTickDelta(float degrees);

/**
 * @brief Converts a relative servo tick delta to a relative angle in degrees.
 * @param ticks Relative encoder tick difference.
 * @return Relative angle in degrees.
 */
float Kinematics_TickDeltaToDeg(int32_t ticks);

/**
 * @brief Converts a raw servo position to a joint angle in degrees.
 * @param joint_id Servo ID of the joint.
 * @param raw_position Raw encoder position in ticks.
 * @param angle_deg Pointer where the calculated angle will be stored.
 * @return Servo operation result.
 */
Servo_Result_t Kinematics_RawToAngleDeg(uint8_t joint_id, uint16_t raw_position, float *angle_deg);

/**
 * @brief Converts a joint angle in degrees to a raw servo position.
 * @param joint_id Servo ID of the joint.
 * @param angle_deg Target joint angle in degrees.
 * @param raw_position Pointer where the calculated raw position will be stored.
 * @return Servo operation result.
 */
Servo_Result_t Kinematics_AngleDegToRaw(uint8_t joint_id, float angle_deg, uint16_t *raw_position);

/**
 * @brief Calculates the forward kinematics to the end effector from joint angles in radians.
 * @param joint_rad Array with angles for joints 1 to 4 in radians.
 * @param transform Pointer where the end-effector transform is stored.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_ForwardRad(const float joint_rad[KINEMATICS_ACTIVE_JOINT_COUNT], Kinematics_Transform_t *transform);

/**
 * @brief Calculates the forward kinematics to the end effector from joint angles in degrees.
 * @param joint_deg Array with angles for joints 1 to 4 in degrees.
 * @param transform Pointer where the end-effector transform is stored.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_ForwardDeg(const float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT], Kinematics_Transform_t *transform);

/**
 * @brief Extracts the XYZ position from a homogeneous transform.
 * @param transform Pointer to the transform.
 * @param position Pointer where the XYZ position is stored.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_GetPosition(const Kinematics_Transform_t *transform, Kinematics_Position_t *position);

/**
 * @brief Reads the current raw servo positions and converts joints 1 to 4 to angles in degrees.
 * @param joint_deg Array where the angles for joints 1 to 4 are stored.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_ReadCurrentJointAnglesDeg(float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT]);

/**
 * @brief Reads the current joint angles and calculates the current end-effector transform.
 * @param transform Pointer where the current end-effector transform is stored.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_ReadCurrentEndEffector(Kinematics_Transform_t *transform);

/**
 * @brief Moves one non-fixed joint by a relative angle in degrees.
 * @param joint_id Servo joint ID.
 * @param delta_deg Relative movement in degrees.
 * @param speed Servo movement speed.
 * @param acceleration Servo movement acceleration.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_MoveJointRelativeDeg(uint8_t joint_id, float delta_deg, uint16_t speed, uint8_t acceleration);

/**
 * @brief Moves one non-fixed joint by a relative angle and waits until it reaches the target.
 * @param joint_id Servo joint ID.
 * @param delta_deg Relative movement in degrees.
 * @param speed Servo movement speed.
 * @param acceleration Servo movement acceleration.
 * @param tolerance_ticks Allowed target error in servo ticks.
 * @param timeout_ms Maximum wait time in milliseconds.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_MoveJointRelativeDegAndWait(uint8_t joint_id, float delta_deg, uint16_t speed, uint8_t acceleration, uint16_t tolerance_ticks, uint32_t timeout_ms);

/**
 * @brief Moves one non-fixed joint to an angle relative to its home position.
 * @param joint_id Servo joint ID.
 * @param angle_deg Target angle in degrees relative to home.
 * @param speed Servo movement speed.
 * @param acceleration Servo movement acceleration.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_MoveJointToAngleDeg(uint8_t joint_id, float angle_deg, uint16_t speed, uint8_t acceleration);

/**
 * @brief Moves one non-fixed joint to an angle relative to home and waits until it reaches the target.
 * @param joint_id Servo joint ID.
 * @param angle_deg Target angle in degrees relative to home.
 * @param speed Servo movement speed.
 * @param acceleration Servo movement acceleration.
 * @param tolerance_ticks Allowed target error in servo ticks.
 * @param timeout_ms Maximum wait time in milliseconds.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_MoveJointToAngleDegAndWait(uint8_t joint_id, float angle_deg, uint16_t speed, uint8_t acceleration, uint16_t tolerance_ticks, uint32_t timeout_ms);

/**
 * @brief Polls one joint until its current raw position is close to the target.
 * @param joint_id Servo joint ID.
 * @param target_raw Target raw servo position.
 * @param tolerance_ticks Allowed target error in servo ticks.
 * @param timeout_ms Maximum wait time in milliseconds.
 * @return SERVO_RESULT_OK if the target was reached, otherwise an error code.
 */
Servo_Result_t Kinematics_WaitUntilJointReached(uint8_t joint_id, uint16_t target_raw, uint16_t tolerance_ticks, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H_ */
