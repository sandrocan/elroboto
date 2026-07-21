/**
 ******************************************************************************
 * @file           : kinematics.h
 * @author         : Niklas Peter
 * @brief          : All functions regarding direct and inverse kinematics
 *                   A detailed control flow is described in docs/kinematics.md
 ******************************************************************************
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "servo.h"
#include "operations.h"
#include <stdint.h>

#define KINEMATICS_ACTIVE_JOINT_COUNT   4U

#define KINEMATICS_TICKS_PER_REV        4096.0f
#define KINEMATICS_DEG_PER_REV          360.0f
#define KINEMATICS_TICKS_PER_DEG        (KINEMATICS_TICKS_PER_REV / KINEMATICS_DEG_PER_REV)
#define KINEMATICS_DEG_PER_TICK        (KINEMATICS_DEG_PER_REV / KINEMATICS_TICKS_PER_REV)

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
 * @brief Stores configuration parameters for numerical inverse kinematics.
 */
typedef struct
{
    uint16_t max_iterations;
    float position_tolerance_m;
    float finite_difference_step_deg;
    float damping;
    float max_step_deg;
} Kinematics_IkConfig_t;

/**
 * @brief Callback type used to abort a blocking kinematic wait operation.
 * @return 0 if the motion should continue, non-zero if the motion should abort.
 */
typedef uint8_t (*Kinematics_AbortCallback_t)(void);

/**
 * @brief Diagnostic values produced for one joint during one control cycle.
 */
typedef struct
{
    uint32_t cycle_index;
    float dt_s;
    uint8_t joint_id;
    uint16_t current_position_ticks;
    uint16_t target_position_ticks;
    int32_t error_ticks;
    float controller_output_ticks;
    int32_t applied_correction_ticks;
    uint16_t commanded_position_ticks;
    uint8_t within_tolerance;
    uint8_t command_sent;
    uint8_t joint_limit_clamped;
} Kinematics_ControlTelemetry_t;

/**
 * @brief Callback type used to report per-joint controller diagnostics.
 * @param telemetry Controller values for one joint and control cycle.
 */
typedef void (*Kinematics_ControlTelemetryCallback_t)(const Kinematics_ControlTelemetry_t *telemetry);

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
 * @brief Writes the default numerical inverse kinematics configuration.
 * @param config Pointer where the default IK configuration will be stored.
 * @return None.
 */
void Kinematics_GetDefaultIkConfig(Kinematics_IkConfig_t *config);

/**
 * @brief Calculates joint angles for a target end-effector XYZ position.
 * @param target_position Pointer to the target XYZ position in meters.
 * @param seed_joint_deg Initial joint angle guess for joints 1 to 4 in degrees.
 * @param config Pointer to the IK configuration, or NULL to use defaults.
 * @param result_joint_deg Array where the solved joint angles for joints 1 to 4 are stored.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_InversePositionDeg(const Kinematics_Position_t *target_position, const float seed_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT], const Kinematics_IkConfig_t *config, float result_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT]);

/**
 * @brief Calculates raw servo targets for a target end-effector XYZ position.
 * @param target_position Pointer to the target XYZ position in meters.
 * @param seed_joint_deg Initial joint angle guess for joints 1 to 4 in degrees.
 * @param config Pointer to the IK configuration, or NULL to use defaults.
 * @param result_raw Array where the solved raw servo targets for joints 1 to 4 are stored.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_InversePositionRaw(const Kinematics_Position_t *target_position, const float seed_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT], const Kinematics_IkConfig_t *config, uint16_t result_raw[KINEMATICS_ACTIVE_JOINT_COUNT]);

/**
 * @brief Moves the end effector toward a target XYZ position using inverse kinematics.
 * @param target_position Pointer to the target XYZ position in meters.
 * @param speed Servo movement speed.
 * @param acceleration Servo movement acceleration.
 * @param config Pointer to the IK configuration, or NULL to use defaults.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_MoveEndEffectorToPosition(const Kinematics_Position_t *target_position, uint16_t speed, uint8_t acceleration, const Kinematics_IkConfig_t *config);

/**
 * @brief Moves the end effector toward a target XYZ position and waits for all active joints.
 * @param target_position Pointer to the target XYZ position in meters.
 * @param speed Servo movement speed.
 * @param acceleration Servo movement acceleration.
 * @param tolerance_ticks Allowed target error in servo ticks.
 * @param timeout_ms Maximum wait time in milliseconds per joint.
 * @param config Pointer to the IK configuration, or NULL to use defaults.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_MoveEndEffectorToPositionAndWait(const Kinematics_Position_t *target_position, uint16_t speed, uint8_t acceleration, uint16_t tolerance_ticks, uint32_t timeout_ms, const Kinematics_IkConfig_t *config, Kinematics_AbortCallback_t abort_callback);

/**
 * @brief Moves all active joints using one PID controller per joint.
 * @param target_position Target Cartesian position in meters.
 * @param speed Servo movement speed.
 * @param acceleration Servo movement acceleration.
 * @param tolerance_ticks Allowed target error in servo ticks.
 * @param timeout_ms Maximum movement time in milliseconds.
 * @param config Optional inverse-kinematics configuration.
 * @param abort_callback Optional callback used to abort the movement.
 * @param telemetry_callback Optional callback for per-joint controller diagnostics.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_MoveEndEffectorToPositionControlled(const Kinematics_Position_t *target_position, uint16_t speed, uint8_t acceleration, uint16_t tolerance_ticks, uint32_t timeout_ms, const Kinematics_IkConfig_t *config, Kinematics_AbortCallback_t abort_callback, Kinematics_ControlTelemetryCallback_t telemetry_callback);


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
Servo_Result_t Kinematics_MoveJointRelativeDegAndWait(uint8_t joint_id, float delta_deg, uint16_t speed, uint8_t acceleration, uint16_t tolerance_ticks, uint32_t timeout_ms, Kinematics_AbortCallback_t abort_callback);

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
Servo_Result_t Kinematics_MoveJointToAngleDegAndWait(uint8_t joint_id, float angle_deg, uint16_t speed, uint8_t acceleration, uint16_t tolerance_ticks, uint32_t timeout_ms, Kinematics_AbortCallback_t abort_callback);

/**
 * @brief Polls one joint until its current raw position is close to the target.
 * @param joint_id Servo joint ID.
 * @param target_raw Target raw servo position.
 * @param tolerance_ticks Allowed target error in servo ticks.
 * @param timeout_ms Maximum wait time in milliseconds.
 * @return SERVO_RESULT_OK if the target was reached, otherwise an error code.
 */
Servo_Result_t Kinematics_WaitUntilJointReached(uint8_t joint_id, uint16_t target_raw, uint16_t tolerance_ticks, uint32_t timeout_ms, Kinematics_AbortCallback_t abort_callback);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H_ */
