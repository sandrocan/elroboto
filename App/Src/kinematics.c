#include "kinematics.h"

#include "uart.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Private defines                                                            */
/* -------------------------------------------------------------------------- */

#define KINEMATICS_PI                  3.14159265358979323846f
#define KINEMATICS_ACTIVE_FIRST_ID     1U
#define KINEMATICS_ACTIVE_LAST_ID      4U
#define KINEMATICS_POLL_DELAY_MS       20U

/* -------------------------------------------------------------------------- */
/* Private types                                                              */
/* -------------------------------------------------------------------------- */

typedef struct
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    uint8_t active;
    uint8_t joint_id;
} Kinematics_Link_t;

/* -------------------------------------------------------------------------- */
/* Private variables                                                          */
/* -------------------------------------------------------------------------- */

/*
 * Chain:
 * base_link -> shoulder_link -> upper_arm_link -> lower_arm_link
 * -> wrist_link -> gripper_link -> gripper_frame_link
 *
 * Transform order:
 * T(parent->child) = Trans(x,y,z) * RotZ(yaw) * RotY(pitch) * RotX(roll) * RotZ(q)
 */
static const Kinematics_Link_t kinematics_chain[] =
{
    { 0.0388353f,  -8.97657e-09f, 0.0624f,       KINEMATICS_PI,    4.18253e-17f, -KINEMATICS_PI, 1U, 1U },
    {-0.0303992f,  -0.0182778f,   -0.0542f,     -1.5708f,        -1.5708f,       0.0f,           1U, 2U },
    {-0.11257f,    -0.028f,        1.73763e-16f,-3.63608e-16f,    8.74301e-16f,  1.5708f,        1U, 3U },
    {-0.1349f,      0.0052f,       3.62355e-17f, 4.02456e-15f,    8.67362e-16f, -1.5708f,        1U, 4U },
    { 5.55112e-17f,-0.0611f,       0.0181f,      1.5708f,         0.0486795f,    KINEMATICS_PI,  0U, 5U },
    {-0.0079f,     -0.000218121f, -0.0981274f,   0.0f,            KINEMATICS_PI,  0.0f,           0U, 0U }
};

/* Direction multiplier from mathematical joint angle to raw servo ticks. */
static const int8_t kinematics_joint_direction[KINEMATICS_ACTIVE_JOINT_COUNT] =
{
    1, 1, 1, 1
};

/* -------------------------------------------------------------------------- */
/* Private function prototypes                                                */
/* -------------------------------------------------------------------------- */

static void Kinematics_SetIdentity(Kinematics_Transform_t *out);
static void Kinematics_Multiply(const Kinematics_Transform_t *a, const Kinematics_Transform_t *b, Kinematics_Transform_t *out);
static Kinematics_Transform_t Kinematics_LinkTransform(const Kinematics_Link_t *link, float joint_angle_rad);
static Servo_Result_t Kinematics_GetJointDirection(uint8_t joint_id, int8_t *direction);
static Servo_Result_t Kinematics_CalculateRelativeTargetRaw(uint8_t joint_id, float delta_deg, uint16_t *target_raw);
static int32_t Kinematics_RoundToI32(float value);

/* -------------------------------------------------------------------------- */
/* Public functions                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Converts degrees to radians.
 * @param degrees Angle in degrees.
 * @return Angle in radians.
 */
float Kinematics_DegToRad(float degrees)
{
    return degrees * (KINEMATICS_PI / 180.0f);
}

/**
 * @brief Converts radians to degrees.
 * @param radians Angle in radians.
 * @return Angle in degrees.
 */
float Kinematics_RadToDeg(float radians)
{
    return radians * (180.0f / KINEMATICS_PI);
}

/**
 * @brief Converts a relative angle in degrees to a relative servo tick delta.
 * @param degrees Relative angle in degrees.
 * @return Relative tick delta.
 */
int32_t Kinematics_DegToTickDelta(float degrees)
{
    return Kinematics_RoundToI32(degrees * KINEMATICS_TICKS_PER_DEG);
}

/**
 * @brief Converts a relative servo tick delta to a relative angle in degrees.
 * @param ticks Relative tick delta.
 * @return Relative angle in degrees.
 */
float Kinematics_TickDeltaToDeg(int32_t ticks)
{
    return ((float)ticks) * KINEMATICS_DEG_PER_TICK;
}

/**
 * @brief Converts a raw servo position to a joint angle relative to the home position.
 * @param joint_id Servo joint ID.
 * @param raw_position Raw servo position in ticks.
 * @param angle_deg Pointer where the calculated angle in degrees is stored.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_RawToAngleDeg(uint8_t joint_id, uint16_t raw_position, float *angle_deg)
{
    const Servo_JointConfig_t *joint;
    int8_t direction;
    int32_t delta_ticks;

    if (angle_deg == NULL)
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    joint = Servo_GetJointConfigById(joint_id);
    if (joint == NULL)
    {
        return SERVO_RESULT_UNKNOWN_JOINT_ID;
    }

    if (joint->is_fixed != 0U)
    {
        return SERVO_RESULT_JOINT_IS_FIXED;
    }

    if (Kinematics_GetJointDirection(joint_id, &direction) != SERVO_RESULT_OK)
    {
        return SERVO_RESULT_UNKNOWN_JOINT_ID;
    }

    delta_ticks = (int32_t)raw_position - (int32_t)joint->home_position;
    *angle_deg = Kinematics_TickDeltaToDeg(delta_ticks * (int32_t)direction);

    return SERVO_RESULT_OK;
}

/**
 * @brief Converts a joint angle relative to home into a raw servo position.
 * @param joint_id Servo joint ID.
 * @param angle_deg Target angle in degrees relative to home.
 * @param raw_position Pointer where the raw position is stored.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_AngleDegToRaw(uint8_t joint_id, float angle_deg, uint16_t *raw_position)
{
    const Servo_JointConfig_t *joint;
    int8_t direction;
    int32_t target_raw;
    int32_t delta_ticks;

    if (raw_position == NULL)
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    joint = Servo_GetJointConfigById(joint_id);
    if (joint == NULL)
    {
        return SERVO_RESULT_UNKNOWN_JOINT_ID;
    }

    if (joint->is_fixed != 0U)
    {
        return SERVO_RESULT_JOINT_IS_FIXED;
    }

    if (Kinematics_GetJointDirection(joint_id, &direction) != SERVO_RESULT_OK)
    {
        return SERVO_RESULT_UNKNOWN_JOINT_ID;
    }

    delta_ticks = Kinematics_DegToTickDelta(angle_deg) * (int32_t)direction;
    target_raw = (int32_t)joint->home_position + delta_ticks;

    if ((target_raw < (int32_t)joint->min_position) ||
        (target_raw > (int32_t)joint->max_position))
    {
        return SERVO_RESULT_POSITION_OUT_OF_RANGE;
    }

    *raw_position = (uint16_t)target_raw;
    return SERVO_RESULT_OK;
}

/**
 * @brief Calculates the forward kinematics to the end effector from joint angles in radians.
 * @param joint_rad Array with angles for joints 1 to 4 in radians.
 * @param transform Pointer where the end-effector transform is stored.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_ForwardRad(const float joint_rad[KINEMATICS_ACTIVE_JOINT_COUNT], Kinematics_Transform_t *transform)
{
    Kinematics_Transform_t total;
    uint8_t active_index = 0U;

    if ((joint_rad == NULL) || (transform == NULL))
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    Kinematics_SetIdentity(&total);

    for (uint8_t i = 0U; i < (uint8_t)(sizeof(kinematics_chain) / sizeof(kinematics_chain[0])); i++)
    {
        Kinematics_Transform_t link_transform;
        Kinematics_Transform_t next_total;
        float q = 0.0f;

        if (kinematics_chain[i].active != 0U)
        {
            if (active_index >= KINEMATICS_ACTIVE_JOINT_COUNT)
            {
                return SERVO_RESULT_UNKNOWN_JOINT_ID;
            }

            q = joint_rad[active_index];
            active_index++;
        }

        link_transform = Kinematics_LinkTransform(&kinematics_chain[i], q);
        Kinematics_Multiply(&total, &link_transform, &next_total);
        total = next_total;
    }

    *transform = total;
    return SERVO_RESULT_OK;
}

/**
 * @brief Calculates the forward kinematics to the end effector from joint angles in degrees.
 * @param joint_deg Array with angles for joints 1 to 4 in degrees.
 * @param transform Pointer where the end-effector transform is stored.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_ForwardDeg(const float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT], Kinematics_Transform_t *transform)
{
    float joint_rad[KINEMATICS_ACTIVE_JOINT_COUNT];

    if ((joint_deg == NULL) || (transform == NULL))
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
    {
        joint_rad[i] = Kinematics_DegToRad(joint_deg[i]);
    }

    return Kinematics_ForwardRad(joint_rad, transform);
}

/**
 * @brief Extracts the XYZ position from a homogeneous transform.
 * @param transform Pointer to the transform.
 * @param position Pointer where the XYZ position is stored.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_GetPosition(const Kinematics_Transform_t *transform, Kinematics_Position_t *position)
{
    if ((transform == NULL) || (position == NULL))
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    position->x = transform->m[0][3];
    position->y = transform->m[1][3];
    position->z = transform->m[2][3];

    return SERVO_RESULT_OK;
}

/**
 * @brief Reads the current raw servo positions and converts joints 1 to 4 to angles in degrees.
 * @param joint_deg Array where the angles for joints 1 to 4 are stored.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_ReadCurrentJointAnglesDeg(float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT])
{
    Servo_Result_t result;

    if (joint_deg == NULL)
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    for (uint8_t joint_id = KINEMATICS_ACTIVE_FIRST_ID; joint_id <= KINEMATICS_ACTIVE_LAST_ID; joint_id++)
    {
        uint16_t raw_position = 0U;

        result = Servo_ReadPosition(joint_id, &raw_position);
        if (result != SERVO_RESULT_OK)
        {
            return result;
        }

        result = Kinematics_RawToAngleDeg(joint_id, raw_position, &joint_deg[joint_id - KINEMATICS_ACTIVE_FIRST_ID]);
        if (result != SERVO_RESULT_OK)
        {
            return result;
        }
    }

    return SERVO_RESULT_OK;
}

/**
 * @brief Reads the current joint angles and calculates the current end-effector transform.
 * @param transform Pointer where the current end-effector transform is stored.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_ReadCurrentEndEffector(Kinematics_Transform_t *transform)
{
    Servo_Result_t result;
    float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT];

    if (transform == NULL)
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    result = Kinematics_ReadCurrentJointAnglesDeg(joint_deg);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    return Kinematics_ForwardDeg(joint_deg, transform);
}

/**
 * @brief Moves one non-fixed joint by a relative angle in degrees.
 * @param joint_id Servo joint ID.
 * @param delta_deg Relative movement in degrees.
 * @param speed Servo movement speed.
 * @param acceleration Servo movement acceleration.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_MoveJointRelativeDeg(uint8_t joint_id, float delta_deg, uint16_t speed, uint8_t acceleration)
{
    Servo_Result_t result;
    uint16_t target_raw;

    result = Kinematics_CalculateRelativeTargetRaw(joint_id, delta_deg, &target_raw);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    return Servo_WritePosition(joint_id, target_raw, speed, acceleration);
}

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
Servo_Result_t Kinematics_MoveJointRelativeDegAndWait(uint8_t joint_id, float delta_deg, uint16_t speed, uint8_t acceleration, uint16_t tolerance_ticks, uint32_t timeout_ms)
{
    Servo_Result_t result;
    uint16_t target_raw;

    result = Kinematics_CalculateRelativeTargetRaw(joint_id, delta_deg, &target_raw);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    result = Servo_WritePosition(joint_id, target_raw, speed, acceleration);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    return Kinematics_WaitUntilJointReached(joint_id, target_raw, tolerance_ticks, timeout_ms);
}

/**
 * @brief Moves one non-fixed joint to an angle relative to its home position.
 * @param joint_id Servo joint ID.
 * @param angle_deg Target angle in degrees relative to home.
 * @param speed Servo movement speed.
 * @param acceleration Servo movement acceleration.
 * @return Servo-style result code.
 */
Servo_Result_t Kinematics_MoveJointToAngleDeg(uint8_t joint_id, float angle_deg, uint16_t speed, uint8_t acceleration)
{
    Servo_Result_t result;
    uint16_t target_raw;

    result = Kinematics_AngleDegToRaw(joint_id, angle_deg, &target_raw);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    return Servo_WritePosition(joint_id, target_raw, speed, acceleration);
}

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
Servo_Result_t Kinematics_MoveJointToAngleDegAndWait(uint8_t joint_id, float angle_deg, uint16_t speed, uint8_t acceleration, uint16_t tolerance_ticks, uint32_t timeout_ms)
{
    Servo_Result_t result;
    uint16_t target_raw;

    result = Kinematics_AngleDegToRaw(joint_id, angle_deg, &target_raw);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    result = Servo_WritePosition(joint_id, target_raw, speed, acceleration);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    return Kinematics_WaitUntilJointReached(joint_id, target_raw, tolerance_ticks, timeout_ms);
}

/**
 * @brief Polls one joint until its current raw position is close to the target.
 * @param joint_id Servo joint ID.
 * @param target_raw Target raw servo position.
 * @param tolerance_ticks Allowed target error in servo ticks.
 * @param timeout_ms Maximum wait time in milliseconds.
 * @return SERVO_RESULT_OK if the target was reached, otherwise an error code.
 */
Servo_Result_t Kinematics_WaitUntilJointReached(uint8_t joint_id, uint16_t target_raw, uint16_t tolerance_ticks, uint32_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();

    while ((HAL_GetTick() - start_time) < timeout_ms)
    {
        Servo_Result_t result;
        uint16_t current_raw = 0U;
        uint16_t error_ticks;

        result = Servo_ReadPosition(joint_id, &current_raw);
        if (result != SERVO_RESULT_OK)
        {
            return result;
        }

        if (current_raw >= target_raw)
        {
            error_ticks = (uint16_t)(current_raw - target_raw);
        }
        else
        {
            error_ticks = (uint16_t)(target_raw - current_raw);
        }

        if (error_ticks <= tolerance_ticks)
        {
            return SERVO_RESULT_OK;
        }

        HAL_Delay(KINEMATICS_POLL_DELAY_MS);
    }

    return SERVO_RESULT_TARGET_NOT_REACHED;
}

/* -------------------------------------------------------------------------- */
/* Private functions                                                          */
/* -------------------------------------------------------------------------- */

static void Kinematics_SetIdentity(Kinematics_Transform_t *out)
{
    for (uint8_t i = 0U; i < KINEMATICS_MATRIX_SIZE; i++)
    {
        for (uint8_t j = 0U; j < KINEMATICS_MATRIX_SIZE; j++)
        {
            out->m[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

static void Kinematics_Multiply(const Kinematics_Transform_t *a, const Kinematics_Transform_t *b, Kinematics_Transform_t *out)
{
    Kinematics_Transform_t tmp;

    for (uint8_t i = 0U; i < KINEMATICS_MATRIX_SIZE; i++)
    {
        for (uint8_t j = 0U; j < KINEMATICS_MATRIX_SIZE; j++)
        {
            tmp.m[i][j] = 0.0f;

            for (uint8_t k = 0U; k < KINEMATICS_MATRIX_SIZE; k++)
            {
                tmp.m[i][j] += a->m[i][k] * b->m[k][j];
            }
        }
    }

    *out = tmp;
}

static Kinematics_Transform_t Kinematics_LinkTransform(const Kinematics_Link_t *link, float joint_angle_rad)
{
    Kinematics_Transform_t out;
    float cr = cosf(link->roll);
    float sr = sinf(link->roll);
    float cp = cosf(link->pitch);
    float sp = sinf(link->pitch);
    float cy = cosf(link->yaw);
    float sy = sinf(link->yaw);
    float cq = cosf(joint_angle_rad);
    float sq = sinf(joint_angle_rad);

    /* Rotation = Rz(yaw) * Ry(pitch) * Rx(roll) * Rz(q). */
    float r00 = cy * cp;
    float r01 = cy * sp * sr - sy * cr;
    float r02 = cy * sp * cr + sy * sr;

    float r10 = sy * cp;
    float r11 = sy * sp * sr + cy * cr;
    float r12 = sy * sp * cr - cy * sr;

    float r20 = -sp;
    float r21 = cp * sr;
    float r22 = cp * cr;

    Kinematics_SetIdentity(&out);

    out.m[0][0] = r00 * cq + r01 * sq;
    out.m[0][1] = -r00 * sq + r01 * cq;
    out.m[0][2] = r02;
    out.m[0][3] = link->x;

    out.m[1][0] = r10 * cq + r11 * sq;
    out.m[1][1] = -r10 * sq + r11 * cq;
    out.m[1][2] = r12;
    out.m[1][3] = link->y;

    out.m[2][0] = r20 * cq + r21 * sq;
    out.m[2][1] = -r20 * sq + r21 * cq;
    out.m[2][2] = r22;
    out.m[2][3] = link->z;

    return out;
}

static Servo_Result_t Kinematics_GetJointDirection(uint8_t joint_id, int8_t *direction)
{
    if (direction == NULL)
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    if ((joint_id < KINEMATICS_ACTIVE_FIRST_ID) || (joint_id > KINEMATICS_ACTIVE_LAST_ID))
    {
        return SERVO_RESULT_JOINT_IS_FIXED;
    }

    *direction = kinematics_joint_direction[joint_id - KINEMATICS_ACTIVE_FIRST_ID];
    return SERVO_RESULT_OK;
}

static Servo_Result_t Kinematics_CalculateRelativeTargetRaw(uint8_t joint_id, float delta_deg, uint16_t *target_raw)
{
    const Servo_JointConfig_t *joint;
    Servo_Result_t result;
    int8_t direction;
    uint16_t current_raw = 0U;
    int32_t delta_ticks;
    int32_t target;

    if (target_raw == NULL)
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    joint = Servo_GetJointConfigById(joint_id);
    if (joint == NULL)
    {
        return SERVO_RESULT_UNKNOWN_JOINT_ID;
    }

    if (joint->is_fixed != 0U)
    {
        return SERVO_RESULT_JOINT_IS_FIXED;
    }

    result = Kinematics_GetJointDirection(joint_id, &direction);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    result = Servo_ReadPosition(joint_id, &current_raw);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    delta_ticks = Kinematics_DegToTickDelta(delta_deg) * (int32_t)direction;
    target = (int32_t)current_raw + delta_ticks;

    if ((target < (int32_t)joint->min_position) ||
        (target > (int32_t)joint->max_position))
    {
        return SERVO_RESULT_POSITION_OUT_OF_RANGE;
    }

    *target_raw = (uint16_t)target;
    return SERVO_RESULT_OK;
}

static int32_t Kinematics_RoundToI32(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}
