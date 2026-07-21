/**
 ******************************************************************************
 * @file           : servo.c
 * @author         : Niklas Peter
 * @brief          : All functions regarding control and communication with Feetech motors
 ******************************************************************************
 */

#include "servo.h"
#include "uart.h"

#include <stddef.h>
#include <stdio.h>


/* -------------------------------------------------------------------------- */
/* Private defines                                                            */
/* -------------------------------------------------------------------------- */

#define SERVO_INSTRUCTION_READ              0x02U

#define SERVO_REGISTER_PRESENT_POSITION     0x38U

#define SERVO_READ_POSITION_PACKET_LENGTH   8U
#define SERVO_READ_POSITION_RESPONSE_LENGTH 8U

#define SERVO_HOME_POSITION_TOLERANCE       50U
#define SERVO_HOME_TIMEOUT_MS               15000U
#define SERVO_HOME_POLL_INTERVAL_MS         250U

#define SERVO_PING_PACKET_LENGTH    6U
#define SERVO_STATUS_PACKET_LENGTH  6U

#define SERVO_TIMEOUT_TX_MS         100U
#define SERVO_TIMEOUT_RX_MS         100U

#define SERVO_INSTRUCTION_WRITE     0x03U

#define SERVO_REGISTER_TORQUE_ENABLE      0x28U

#define SERVO_TORQUE_DISABLE              0x00U
#define SERVO_TORQUE_ENABLE               0x01U

#define SERVO_WRITE_BYTE_PACKET_LENGTH   8U

#define SERVO_REGISTER_ACC          0x29U
#define SERVO_WRITE_POSITION_SIZE   14U

#define SERVO_HOME_SPEED              200U
#define SERVO_HOME_ACCELERATION       50U
#define SERVO_WRIST_ROLL_ID             5U
#define SERVO_GRIPPER_ID                6U

/* -------------------------------------------------------------------------- */
/* Private variables                                                          */
/* -------------------------------------------------------------------------- */

static const Servo_JointConfig_t *servo_joint_table = NULL;
static uint8_t servo_joint_count = 0U;
static uint8_t servo_is_initialized = 0U;
static uint8_t servo_last_response[SERVO_STATUS_PACKET_LENGTH] = {0};
static uint16_t servo_last_response_length = 0;

/* -------------------------------------------------------------------------- */
/* Private function prototypes                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Finds a joint configuration by servo ID.
 * @param id Servo ID to search for.
 * @return Pointer to the matching joint configuration, or NULL if not found.
 */
static const Servo_JointConfig_t *Servo_FindJointById(uint8_t id);

/**
 * @brief Calculates the servo protocol checksum for a byte block.
 * @param data Pointer to the data bytes.
 * @param length Number of bytes used for checksum calculation.
 * @return Calculated checksum byte.
 */
static uint8_t Servo_CalculateChecksum(const uint8_t *data, uint16_t length);

/**
 * @brief Validates the last received standard servo status packet.
 * @param expected_id Servo ID expected in the response.
 * @return Servo operation result.
 */
static Servo_Result_t Servo_CheckStatusPacket(uint8_t expected_id);

/**
 * @brief Calculates the absolute difference between two uint16_t values.
 * @param a First value.
 * @param b Second value.
 * @return Absolute difference between a and b.
 */
static uint16_t Servo_AbsDiffU16(uint16_t a, uint16_t b);

/**
 * @brief Writes one byte to a servo register and validates the response.
 * @param id Servo ID to write to.
 * @param address Servo register address.
 * @param value Byte value to write.
 * @return Servo operation result.
 */
static Servo_Result_t Servo_WriteByte(uint8_t id, uint8_t address, uint8_t value);

/**
 * @brief Sends a raw position command to a servo.
 * @param id Servo ID to command.
 * @param position Target position in servo ticks.
 * @param speed Movement speed value.
 * @param acceleration Movement acceleration value.
 * @return Servo operation result.
 */
static Servo_Result_t Servo_SendPositionCommand(uint8_t id, uint16_t position, uint16_t speed, uint8_t acceleration);

/**
 * @brief Reads the current raw position of one servo once without retry handling.
 * @param id Servo ID to read from.
 * @param position Pointer where the raw position value is stored.
 * @return Servo operation result.
 */
static Servo_Result_t Servo_ReadPositionOnce(uint8_t id, uint16_t *position);

/* -------------------------------------------------------------------------- */
/* Public functions (Descriptions in Header)                                                        */
/* -------------------------------------------------------------------------- */

void Servo_Init(void)
{
	// FOLLOWER CONFIG
    static const Servo_JointConfig_t default_joint_table[] =
    {
        {1U, "shoulder_pan",    729U, 2047U, 3391U, 0U},
        {2U, "shoulder_lift",   787U, 2047U, 3308U, 0U},
        {3U, "elbow_flex",      900U, 2047U, 2998U, 0U},
        {4U, "wrist_flex",      933U, 2047U, 3228U, 0U},
        {5U, "wrist_roll",     2000U, 2047U, 2200U, 1U},
        {6U, "gripper",         810U,  810U, 2200U, 1U}
    };

//	// LEADER CONFIG
//	static const Servo_JointConfig_t default_joint_table[] =
//	{
//	    {1U, "shoulder_pan",   1086U, 2266U, 3566U, 0U},
//	    {2U, "shoulder_lift",   864U, 2096U, 3271U, 0U},
//	    {3U, "elbow_flex",      387U, 1572U, 2615U, 0U},
//	    {4U, "wrist_flex",      837U, 2066U, 3186U, 0U},
//	    {5U, "wrist_roll",     2000U, 2047U, 2200U, 1U},
//	    {6U, "gripper",        1382U, 1992U, 2654U, 1U}
//	};


    //Write into public variables
    servo_joint_table = default_joint_table;
    servo_joint_count = (uint8_t)(sizeof(default_joint_table) / sizeof(default_joint_table[0]));
    servo_last_response_length = 0U;

    for (uint8_t i = 0; i < SERVO_STATUS_PACKET_LENGTH; i++)
    {
        servo_last_response[i] = 0U;
    }

    servo_is_initialized = 1U;
}

uint8_t Servo_GetJointCount(void)
{
    if (servo_is_initialized == 0U)
    {
        return 0U;
    }

    return servo_joint_count;
}

const Servo_JointConfig_t *Servo_GetJointConfigByIndex(uint8_t index)
{
    if (servo_is_initialized == 0U)
    {
        return NULL;
    }

    if (servo_joint_table == NULL)
    {
        return NULL;
    }

    if (index >= servo_joint_count)
    {
        return NULL;
    }

    return &servo_joint_table[index];
}

const Servo_JointConfig_t *Servo_GetJointConfigById(uint8_t id)
{
    if (servo_is_initialized == 0U)
    {
        return NULL;
    }

    return Servo_FindJointById(id);
}

Servo_Result_t Servo_Ping(uint8_t id)
{
    uint8_t packet[SERVO_PING_PACKET_LENGTH];

    /*
     * Custom Ping packet:
     *
     * Header      FF FF
     * ID          id
     * Length      02
     * Instruction 01 = Ping
     * Checksum    ~(ID + Length + Instruction)
     */
    packet[0] = 0xFFU;
    packet[1] = 0xFFU;
    packet[2] = id;
    packet[3] = 0x02U;
    packet[4] = 0x01U;
    packet[5] = Servo_CalculateChecksum(&packet[2], 3U);

    for (uint8_t i = 0; i < SERVO_STATUS_PACKET_LENGTH; i++)
    {
        servo_last_response[i] = 0U;
    }

    servo_last_response_length = 0;

    //Send ping to the servo
    HAL_StatusTypeDef tx_status = UartServo_SendCommand(
        packet,
        SERVO_PING_PACKET_LENGTH,
        SERVO_TIMEOUT_TX_MS
    );

    if (tx_status != HAL_OK)
    {
        return SERVO_RESULT_TX_ERROR;
    }

    //Receive pong from the servo
    HAL_StatusTypeDef rx_status = UartServo_ReadResponse(
        servo_last_response,
        SERVO_STATUS_PACKET_LENGTH,
        SERVO_TIMEOUT_RX_MS
    );

    if (rx_status == HAL_TIMEOUT)
    {
        return SERVO_RESULT_RX_TIMEOUT;
    }

    if (rx_status != HAL_OK)
    {
        return SERVO_RESULT_RX_ERROR;
    }

    servo_last_response_length = SERVO_STATUS_PACKET_LENGTH;

    //Print the ping result into UART
    return Servo_CheckStatusPacket(id);
}

Servo_Result_t Servo_LockJoint(uint8_t id)
{
	return Servo_WriteByte(id, SERVO_REGISTER_TORQUE_ENABLE, SERVO_TORQUE_ENABLE);
}

Servo_Result_t Servo_UnlockJoint(uint8_t id)
{
	return Servo_WriteByte(id, SERVO_REGISTER_TORQUE_ENABLE, SERVO_TORQUE_DISABLE);
}

Servo_Result_t Servo_DriveHome(void)
{
    //Init result
    Servo_Result_t result;

    if (servo_is_initialized == 0U)
    {
        return SERVO_RESULT_NOT_INITIALIZED;
    }

    if (servo_joint_table == NULL)
    {
        return SERVO_RESULT_NOT_INITIALIZED;
    }

    //Step 1: Disable torque first so the arm can be manually placed before homing
    //without fighting any still-locked joint
    for (uint8_t i = 0U; i < servo_joint_count; i++)
    {
        const Servo_JointConfig_t *joint = &servo_joint_table[i];

        result = Servo_UnlockJoint(joint->id);

        if (result != SERVO_RESULT_OK)
        {
            char text[160];

            snprintf(
                text,
                sizeof(text),
                "Home unlock failed joint %u (%s): result=%s\r\n",
                joint->id,
                joint->name,
                Servo_ResultToString(result)
            );

            UartDebug_SendString(text);
            return result;
        }
    }

    //Step 2: Validate limits and enable torque on every configured joint
    for (uint8_t i = 0U; i < servo_joint_count; i++)
    {
        const Servo_JointConfig_t *joint = &servo_joint_table[i];

        if ((joint->home_position_ticks < joint->min_position_ticks) ||
            (joint->home_position_ticks > joint->max_position_ticks))
        {
            char text[160];

            snprintf(
                text,
                sizeof(text),
                "Home limit invalid joint %u (%s): min=%u home=%u max=%u\r\n",
                joint->id,
                joint->name,
                joint->min_position_ticks,
                joint->home_position_ticks,
                joint->max_position_ticks
            );

            UartDebug_SendString(text);
            return SERVO_RESULT_POSITION_OUT_OF_RANGE;
        }

        result = Servo_LockJoint(joint->id);

        if (result != SERVO_RESULT_OK)
        {
            char text[160];

            snprintf(
                text,
                sizeof(text),
                "Home lock failed joint %u (%s): result=%s\r\n",
                joint->id,
                joint->name,
                Servo_ResultToString(result)
            );

            UartDebug_SendString(text);
            return result;
        }
    }

    //Step 3: Command every configured joint to its home value
    for (uint8_t i = 0U; i < servo_joint_count; i++)
    {
        const Servo_JointConfig_t *joint = &servo_joint_table[i];

        result = Servo_SendPositionCommand(
            joint->id,
            joint->home_position_ticks,
            SERVO_HOME_SPEED,
            SERVO_HOME_ACCELERATION
        );

        if (result != SERVO_RESULT_OK)
        {
            char text[160];

            snprintf(
                text,
                sizeof(text),
                "Home command failed joint %u (%s): result=%s\r\n",
                joint->id,
                joint->name,
                Servo_ResultToString(result)
            );

            UartDebug_SendString(text);
            return result;
        }

        char text[160];

        snprintf(
            text,
            sizeof(text),
            "Home command joint %u (%s): target=%u fixed=%u\r\n",
            joint->id,
            joint->name,
            joint->home_position_ticks,
            joint->is_fixed
        );

        UartDebug_SendString(text);
    }

    //Step 4: Poll encoder positions until every configured joint reached home
    uint32_t start_time = HAL_GetTick();

    while ((HAL_GetTick() - start_time) < SERVO_HOME_TIMEOUT_MS)
    {
        uint8_t all_reached = 1U;

        for (uint8_t i = 0U; i < servo_joint_count; i++)
        {
            const Servo_JointConfig_t *joint = &servo_joint_table[i];
            uint16_t current_position = 0U;
            uint16_t error_ticks = 0U;

            result = Servo_ReadPosition(joint->id, &current_position);

            if (result != SERVO_RESULT_OK)
            {
                char text[160];

                snprintf(
                    text,
                    sizeof(text),
                    "Home read failed joint %u (%s): result=%s\r\n",
                    joint->id,
                    joint->name,
                    Servo_ResultToString(result)
                );

                UartDebug_SendString(text);
                return result;
            }

            error_ticks = Servo_AbsDiffU16(
                current_position,
                joint->home_position_ticks
            );

            char text[160];

            snprintf(
                text,
                sizeof(text),
                "Read joint %u (%s): current=%u target=%u error=%u\r\n",
                joint->id,
                joint->name,
                current_position,
                joint->home_position_ticks,
                error_ticks
            );

            UartDebug_SendString(text);

            if (error_ticks > SERVO_HOME_POSITION_TOLERANCE)
            {
                all_reached = 0U;

                (void)Servo_SendPositionCommand(
                    joint->id,
                    joint->home_position_ticks,
                    SERVO_HOME_SPEED,
                    SERVO_HOME_ACCELERATION
                );
            }

            HAL_Delay(5U);
        }

        if (all_reached != 0U)
        {
            result = Servo_LockJoint(SERVO_WRIST_ROLL_ID);

            if (result != SERVO_RESULT_OK)
            {
                char text[160];

                snprintf(
                    text,
                    sizeof(text),
                    "Home final lock failed joint %u: result=%s\r\n",
                    SERVO_WRIST_ROLL_ID,
                    Servo_ResultToString(result)
                );

                UartDebug_SendString(text);
                return result;
            }

            result = Servo_LockJoint(SERVO_GRIPPER_ID);

            if (result != SERVO_RESULT_OK)
            {
                char text[160];

                snprintf(
                    text,
                    sizeof(text),
                    "Home final lock failed joint %u: result=%s\r\n",
                    SERVO_GRIPPER_ID,
                    Servo_ResultToString(result)
                );

                UartDebug_SendString(text);
                return result;
            }

            UartDebug_SendString("Home reached; locked wrist_roll and gripper\r\n");
            return SERVO_RESULT_OK;
        }

        HAL_Delay(SERVO_HOME_POLL_INTERVAL_MS);
    }

    UartDebug_SendString("Home failed: target not reached before timeout\r\n");
    return SERVO_RESULT_TARGET_NOT_REACHED;
}

Servo_Result_t Servo_ReadPosition(uint8_t id, uint16_t *position)
{
    Servo_Result_t result = SERVO_RESULT_RX_TIMEOUT;

    if (position == NULL)
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    //Try 5 times to read from a frehsly written buffer
    for (uint8_t attempt = 0U; attempt < 5U; attempt++)
    {
        //Clean buffer first so no old bits are set
        UartServo_ClearRxBuffer();

        //Read instruction
        result = Servo_ReadPositionOnce(id, position);

        if (result == SERVO_RESULT_OK)
        {
            return SERVO_RESULT_OK;
        }

        HAL_Delay(20U);
    }

    return result;
}

Servo_Result_t Servo_WritePosition(uint8_t id, uint16_t position, uint16_t speed, uint8_t acceleration)
{
    const Servo_JointConfig_t *joint = NULL;

    //Check if joint is available and valid
    if (servo_is_initialized == 0U)
    {
        return SERVO_RESULT_NOT_INITIALIZED;
    }

    joint = Servo_FindJointById(id);

    if (joint == NULL)
    {
        return SERVO_RESULT_UNKNOWN_JOINT_ID;
    }

    if (joint->is_fixed != 0U)
    {
        return SERVO_RESULT_JOINT_IS_FIXED;
    }

    if ((position < joint->min_position_ticks) || (position > joint->max_position_ticks))
    {
        return SERVO_RESULT_POSITION_OUT_OF_RANGE;
    }

    //Write the package into the UART
    return Servo_SendPositionCommand(
        id,
        position,
        speed,
        acceleration
    );
}

const uint8_t *Servo_GetLastResponse(void)
{
    return servo_last_response;
}

uint16_t Servo_GetLastResponseLength(void)
{
    return servo_last_response_length;
}

const char *Servo_ResultToString(Servo_Result_t result)
{
    //Used to print out the errors based on the enum struct
    switch (result)
    {
        case SERVO_RESULT_OK:
            return "SERVO_RESULT_OK";

        case SERVO_RESULT_TX_ERROR:
            return "SERVO_RESULT_TX_ERROR";

        case SERVO_RESULT_RX_TIMEOUT:
            return "SERVO_RESULT_RX_TIMEOUT";

        case SERVO_RESULT_RX_ERROR:
            return "SERVO_RESULT_RX_ERROR";

        case SERVO_RESULT_INVALID_HEADER:
            return "SERVO_RESULT_INVALID_HEADER";

        case SERVO_RESULT_INVALID_ID:
            return "SERVO_RESULT_INVALID_ID";

        case SERVO_RESULT_INVALID_CHECKSUM:
            return "SERVO_RESULT_INVALID_CHECKSUM";

        case SERVO_RESULT_SERVO_ERROR:
            return "SERVO_RESULT_SERVO_ERROR";

        case SERVO_RESULT_POSITION_OUT_OF_RANGE:
			return "SERVO_RESULT_POSITION_OUT_OF_RANGE";

        case SERVO_RESULT_NOT_INITIALIZED:
            return "SERVO_RESULT_NOT_INITIALIZED";

        case SERVO_RESULT_UNKNOWN_JOINT_ID:
            return "SERVO_RESULT_UNKNOWN_JOINT_ID";

        case SERVO_RESULT_JOINT_IS_FIXED:
            return "SERVO_RESULT_JOINT_IS_FIXED";

        case SERVO_RESULT_NULL_POINTER:
            return "SERVO_RESULT_NULL_POINTER";

        case SERVO_RESULT_TARGET_NOT_REACHED:
            return "SERVO_RESULT_TARGET_NOT_REACHED";

        case SERVO_RESULT_ABORTED:
            return "SERVO_RESULT_ABORTED";

        default:
            return "SERVO_RESULT_UNKNOWN";
    }
}


/* -------------------------------------------------------------------------- */
/* Private functions                                                          */
/* -------------------------------------------------------------------------- */

static uint8_t Servo_CalculateChecksum(const uint8_t *data, uint16_t length)
{
    uint16_t sum = 0U;

    for (uint16_t i = 0; i < length; i++)
    {
        sum += data[i];
    }

    return (uint8_t)(~sum);
}

static Servo_Result_t Servo_CheckStatusPacket(uint8_t expected_id)
{
    uint8_t calculated_checksum = 0U;

    if ((servo_last_response[0] != 0xFFU) ||
        (servo_last_response[1] != 0xFFU))
    {
        return SERVO_RESULT_INVALID_HEADER;
    }

    if (servo_last_response[2] != expected_id)
    {
        return SERVO_RESULT_INVALID_ID;
    }

    calculated_checksum = Servo_CalculateChecksum(&servo_last_response[2], 3U);

    //Check if checksum aligns
    if (servo_last_response[5] != calculated_checksum)
    {
        return SERVO_RESULT_INVALID_CHECKSUM;
    }

    if (servo_last_response[4] != 0x00U)
    {
        return SERVO_RESULT_SERVO_ERROR;
    }

    return SERVO_RESULT_OK;
}

static Servo_Result_t Servo_WriteByte(uint8_t id, uint8_t address, uint8_t value)
{
    uint8_t packet[SERVO_WRITE_BYTE_PACKET_LENGTH];

    /*
     * Write 1 byte packet:
     *
     * FF FF 
     * ID 
     * LENGTH 
     * INSTRUCTION 
     * ADDRESS 
     * VALUE 
     * CHECKSUM
     */

    packet[0] = 0xFFU;
    packet[1] = 0xFFU;
    packet[2] = id;
    packet[3] = 0x04U;
    packet[4] = SERVO_INSTRUCTION_WRITE;
    packet[5] = address;
    packet[6] = value;
    packet[7] = Servo_CalculateChecksum(&packet[2], 5U);

    for (uint8_t i = 0; i < SERVO_STATUS_PACKET_LENGTH; i++)
    {
        servo_last_response[i] = 0U;
    }

    servo_last_response_length = 0;

    //Send the package over UART
    HAL_StatusTypeDef tx_status = UartServo_SendCommand(
        packet,
        SERVO_WRITE_BYTE_PACKET_LENGTH,
        SERVO_TIMEOUT_TX_MS
    );

    if (tx_status != HAL_OK)
    {
        return SERVO_RESULT_TX_ERROR;
    }

    //Receive answer over UART
    HAL_StatusTypeDef rx_status = UartServo_ReadResponse(
        servo_last_response,
        SERVO_STATUS_PACKET_LENGTH,
        SERVO_TIMEOUT_RX_MS
    );

    if (rx_status == HAL_TIMEOUT)
    {
        return SERVO_RESULT_RX_TIMEOUT;
    }

    if (rx_status != HAL_OK)
    {
        return SERVO_RESULT_RX_ERROR;
    }

    servo_last_response_length = SERVO_STATUS_PACKET_LENGTH;

    return Servo_CheckStatusPacket(id);
}

static const Servo_JointConfig_t *Servo_FindJointById(uint8_t id)
{
    if (servo_joint_table == NULL)
    {
        return NULL;
    }

    for (uint8_t i = 0; i < servo_joint_count; i++)
    {
        if (servo_joint_table[i].id == id)
        {
            return &servo_joint_table[i];
        }
    }

    return NULL;
}

static Servo_Result_t Servo_SendPositionCommand(uint8_t id,
                                                uint16_t position,
                                                uint16_t speed,
                                                uint8_t acceleration)
{
    uint8_t packet[SERVO_WRITE_POSITION_SIZE];
    HAL_StatusTypeDef tx_status;

    /*
     * Write 1 full command package:
     *
     * FF FF 
     * ID 
     * LENGTH 
     * INSTRUCTION 
     * ADDRESS 
     * ACC  
     * POSITION (2 bytes)
     * TIME (2 bytes, unused)
     * SPEED (2 bytes)
     * CHECKSUM
     */

    packet[0]  = 0xFFU;
    packet[1]  = 0xFFU;
    packet[2]  = id;
    packet[3]  = 0x0AU;
    packet[4]  = SERVO_INSTRUCTION_WRITE;
    packet[5]  = SERVO_REGISTER_ACC;

    packet[6]  = acceleration;

    packet[7]  = (uint8_t)(position & 0xFFU);
    packet[8]  = (uint8_t)((position >> 8U) & 0xFFU);

    packet[9]  = 0x00U;
    packet[10] = 0x00U;

    packet[11] = (uint8_t)(speed & 0xFFU);
    packet[12] = (uint8_t)((speed >> 8U) & 0xFFU);

    packet[13] = Servo_CalculateChecksum(&packet[2], 11U);

    for (uint8_t i = 0U; i < SERVO_STATUS_PACKET_LENGTH; i++)
    {
        servo_last_response[i] = 0U;
    }

    servo_last_response_length = 0U;

    //Clear buffer before sending
    UartServo_ClearRxBuffer();

    //Send the package over UART
    tx_status = UartServo_SendCommand(
        packet,
        SERVO_WRITE_POSITION_SIZE,
        SERVO_TIMEOUT_TX_MS
    );

    if (tx_status != HAL_OK)
    {
        return SERVO_RESULT_TX_ERROR;
    }

    //Short delay before draining
    HAL_Delay(10U);
    UartServo_ClearRxBuffer();

    return SERVO_RESULT_OK;
}

static Servo_Result_t Servo_ReadPositionOnce(uint8_t id, uint16_t *position)
{
    uint8_t packet[SERVO_READ_POSITION_PACKET_LENGTH];
    uint8_t response[SERVO_READ_POSITION_RESPONSE_LENGTH];
    uint8_t calculated_checksum;
    HAL_StatusTypeDef tx_status;
    HAL_StatusTypeDef rx_status;

    if (position == NULL)
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    *position = 0U;

    /*
     * Read only the position of a servo:
     *
     * FF FF 
     * ID 
     * LENGTH 
     * INSTRUCTION 
     * POSITION (2bytes)
     * CHECKSUM
     */
    packet[0] = 0xFFU;
    packet[1] = 0xFFU;
    packet[2] = id;
    packet[3] = 0x04U;
    packet[4] = SERVO_INSTRUCTION_READ;
    packet[5] = SERVO_REGISTER_PRESENT_POSITION;
    packet[6] = 0x02U;
    packet[7] = Servo_CalculateChecksum(&packet[2], 5U);

    for (uint8_t i = 0U; i < SERVO_READ_POSITION_RESPONSE_LENGTH; i++)
    {
        response[i] = 0U;
    }

    //Tell the servo that we want the position value
    tx_status = UartServo_SendCommand(
        packet,
        SERVO_READ_POSITION_PACKET_LENGTH,
        SERVO_TIMEOUT_TX_MS
    );

    if (tx_status != HAL_OK)
    {
        return SERVO_RESULT_TX_ERROR;
    }

    //Read the responded position
    rx_status = UartServo_ReadResponse(
        response,
        SERVO_READ_POSITION_RESPONSE_LENGTH,
        SERVO_TIMEOUT_RX_MS
    );

    if (rx_status == HAL_TIMEOUT)
    {
        return SERVO_RESULT_RX_TIMEOUT;
    }

    if (rx_status != HAL_OK)
    {
        return SERVO_RESULT_RX_ERROR;
    }

    if ((response[0] != 0xFFU) ||
        (response[1] != 0xFFU))
    {
        return SERVO_RESULT_INVALID_HEADER;
    }

    if (response[2] != id)
    {
        return SERVO_RESULT_INVALID_ID;
    }

    //Verify that the response is valid
    calculated_checksum = Servo_CalculateChecksum(&response[2], 5U);

    if (response[7] != calculated_checksum)
    {
        return SERVO_RESULT_INVALID_CHECKSUM;
    }

    if (response[4] != 0x00U)
    {
        return SERVO_RESULT_SERVO_ERROR;
    }

    *position = (uint16_t)response[5] |
                ((uint16_t)response[6] << 8U);

    return SERVO_RESULT_OK;
}

static uint16_t Servo_AbsDiffU16(uint16_t a, uint16_t b)
{
    if (a >= b)
    {
        return (uint16_t)(a - b);
    }

    return (uint16_t)(b - a);
}
