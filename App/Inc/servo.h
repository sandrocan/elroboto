/**
 ******************************************************************************
 * @file           : servo.h / servo.c
 * @author         : Niklas Peter
 * @brief          : All functions regarding control and communication with Feetech motors
 ******************************************************************************
 */

#ifndef SERVO_H_
#define SERVO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Public structs and failure enum                                            */
/* -------------------------------------------------------------------------- */

typedef enum
{
    SERVO_RESULT_OK = 0,
    SERVO_RESULT_TX_ERROR,
    SERVO_RESULT_RX_TIMEOUT,
    SERVO_RESULT_RX_ERROR,
    SERVO_RESULT_INVALID_HEADER,
    SERVO_RESULT_INVALID_ID,
    SERVO_RESULT_INVALID_CHECKSUM,
    SERVO_RESULT_SERVO_ERROR,
	SERVO_RESULT_POSITION_OUT_OF_RANGE,
	SERVO_RESULT_NOT_INITIALIZED,
	SERVO_RESULT_UNKNOWN_JOINT_ID,
	SERVO_RESULT_JOINT_IS_FIXED,
	SERVO_RESULT_NULL_POINTER,
	SERVO_RESULT_TARGET_NOT_REACHED,
	SERVO_RESULT_ABORTED
} Servo_Result_t;

typedef struct
{
    uint8_t id;
    const char *name;
    uint16_t min_position_ticks;
    uint16_t home_position_ticks;
    uint16_t max_position_ticks;
    uint8_t is_fixed;
} Servo_JointConfig_t;

/**
 * @brief Initializes the servo module and loads the default joint table.
 * @return None.
 */
void Servo_Init(void);

/**
 * @brief Returns the number of configured servo joints.
 * @return Joint count, or 0 if the servo module is not initialized.
 */
uint8_t Servo_GetJointCount(void);

/**
 * @brief Returns a joint configuration by table index.
 * @param index Index inside the joint configuration table.
 * @return Pointer to the joint configuration, or NULL if invalid.
 */
const Servo_JointConfig_t *Servo_GetJointConfigByIndex(uint8_t index);

/**
 * @brief Returns a joint configuration by servo ID.
 * @param id Servo ID to search for.
 * @return Pointer to the joint configuration, or NULL if not found.
 */
const Servo_JointConfig_t *Servo_GetJointConfigById(uint8_t id);

/**
 * @brief Sends a ping command to a servo and validates the response.
 * @param id Servo ID to ping.
 * @return Servo operation result.
 */
Servo_Result_t Servo_Ping(uint8_t id);

/**
 * @brief Enables torque on a servo joint.
 * @param id Servo ID of the joint to lock.
 * @return Servo operation result.
 */
Servo_Result_t Servo_LockJoint(uint8_t id);

/**
 * @brief Disables torque on a servo joint.
 * @param id Servo ID of the joint to unlock.
 * @return Servo operation result.
 */
Servo_Result_t Servo_UnlockJoint(uint8_t id);

/**
 * @brief Blocking helper that drives all configured joints to home.
 *
 * The routine unlocks all joints first, locks and commands all six configured
 * joints to their home ticks, waits until every joint is within tolerance, then
 * locks wrist_roll and gripper again.
 *
 * @return Servo operation result.
 */
Servo_Result_t Servo_DriveHome(void);

/**
 * @brief Reads the current position of a servo joint.
 * @param id Servo ID to read from.
 * @param position Pointer where the current position will be stored.
 * @return Servo operation result.
 */
Servo_Result_t Servo_ReadPosition(uint8_t id, uint16_t *position);

Servo_Result_t Servo_ReadPositionRetry(uint8_t id, uint16_t *position);

/**
 * @brief Sends a checked position command to a configured non-fixed joint.
 * @param id Servo ID of the joint to move.
 * @param position Target position in servo ticks.
 * @param speed Movement speed value.
 * @param acceleration Movement acceleration value.
 * @return Servo operation result.
 */
Servo_Result_t Servo_WritePosition(uint8_t id, uint16_t position, uint16_t speed, uint8_t acceleration);

/**
 * @brief Sends checked position commands to multiple servos in one sync-write packet.
 * @param ids Array containing the servo IDs.
 * @param positions Array containing one target position per servo in ticks.
 * @param joint_count Number of entries in ids and positions.
 * @param speed Movement speed value used for all servos.
 * @param acceleration Movement acceleration value used for all servos.
 * @return Servo operation result.
 */
Servo_Result_t Servo_WritePositionsSync(
    const uint8_t ids[],
    const uint16_t positions[],
    uint8_t joint_count,
    uint16_t speed,
    uint8_t acceleration
);

/**
 * @brief Returns the last received servo response buffer.
 * @return Pointer to the internal last-response buffer.
 */
const uint8_t *Servo_GetLastResponse(void);

/**
 * @brief Returns the length of the last received servo response.
 * @return Last response length in bytes.
 */
uint16_t Servo_GetLastResponseLength(void);

/**
 * @brief Converts a servo result code to readable text.
 * @param result Servo result code.
 * @return String representation of the result code.
 */
const char *Servo_ResultToString(Servo_Result_t result);


#ifdef __cplusplus
}
#endif

#endif /* SERVO_H_ */
