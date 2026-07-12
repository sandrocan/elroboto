#include "kinematics.h"
#include "uart.h"

#include <stddef.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Private defines                                                            */
/* -------------------------------------------------------------------------- */

#define KINEMATICS_PI                  3.14159265358979323846f
#define KINEMATICS_ACTIVE_FIRST_ID     1U
#define KINEMATICS_ACTIVE_LAST_ID      4U
#define KINEMATICS_POLL_DELAY_MS       20U

#define KINEMATICS_IK_DEFAULT_MAX_ITERATIONS       80U
#define KINEMATICS_IK_DEFAULT_TOLERANCE_M          0.005f
#define KINEMATICS_IK_DEFAULT_FD_STEP_DEG          1.0f
#define KINEMATICS_IK_DEFAULT_DAMPING              0.0015f
#define KINEMATICS_IK_DEFAULT_MAX_STEP_DEG         5.0f

/* -------------------------------------------------------------------------- */
/* Private types                                                              */
/* -------------------------------------------------------------------------- */

typedef struct
{
    Operations_LinkPose_t pose;
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
    { { 0.0388353f,  -8.97657e-09f, 0.0624f,       KINEMATICS_PI,    4.18253e-17f, -KINEMATICS_PI }, 1U, 1U },
    { {-0.0303992f,  -0.0182778f,   -0.0542f,     -1.5708f,        -1.5708f,       0.0f           }, 1U, 2U },
    { {-0.11257f,    -0.028f,        1.73763e-16f,-3.63608e-16f,    8.74301e-16f,  1.5708f        }, 1U, 3U },
    { {-0.1349f,      0.0052f,       3.62355e-17f, 4.02456e-15f,    8.67362e-16f, -1.5708f        }, 1U, 4U },
    { { 5.55112e-17f,-0.0611f,       0.0181f,      1.5708f,         0.0486795f,    KINEMATICS_PI  }, 0U, 5U },
    { {-0.0079f,     -0.000218121f, -0.0981274f,   0.0f,            KINEMATICS_PI,  0.0f           }, 0U, 0U }
};

/* Direction multiplier from mathematical joint angle to raw servo ticks. */
static const int8_t kinematics_joint_direction[KINEMATICS_ACTIVE_JOINT_COUNT] =
{
    1, 1, 1, 1
};

/* -------------------------------------------------------------------------- */
/* Private function prototypes                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Helper: Returns the configured raw tick direction for an active joint.
 * @param joint_id Servo joint ID.
 * @param direction Pointer where the direction multiplier is stored.
 * @return Servo-style result code.
 */
static Servo_Result_t Kinematics_GetJointDirection(uint8_t joint_id, int8_t *direction);

/**
 * @brief Calculates a raw servo target for a relative joint movement.
 * @param joint_id Servo joint ID.
 * @param delta_deg Relative movement angle in degrees.
 * @param target_raw Pointer where the calculated raw target position is stored.
 * @return Servo-style result code.
 */
static Servo_Result_t Kinematics_CalculateRelativeTargetRaw(uint8_t joint_id, float delta_deg, uint16_t *target_raw);

/**
 * @brief Returns the valid angle limits for one active joint.
 * @param joint_id Servo joint ID.
 * @param min_deg Pointer where the minimum angle is stored.
 * @param max_deg Pointer where the maximum angle is stored.
 * @return Servo-style result code.
 */
static Servo_Result_t Kinematics_GetJointLimitsDeg(uint8_t joint_id, float *min_deg, float *max_deg);

/**
 * @brief Limits a float value to a minimum and maximum value.
 * @param value Pointer to the value to limit.
 * @param min_value Minimum allowed value.
 * @param max_value Maximum allowed value.
 * @return None.
 */
static void Kinematics_ClampFloat(float *value, float min_value, float max_value);

/**
 * @brief Calculates the squared XYZ distance between two positions.
 * @param a Pointer to the first position.
 * @param b Pointer to the second position.
 * @return Squared distance.
 */
static float Kinematics_PositionDistanceSquared(const Kinematics_Position_t *a, const Kinematics_Position_t *b);

/**
 * @brief Calculates the end-effector position for joint angles in degrees.
 * @param joint_deg Array with angles for joints 1 to 4 in degrees.
 * @param position Pointer where the end-effector position is stored.
 * @return Servo-style result code.
 */
static Servo_Result_t Kinematics_ForwardPositionDeg(const float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT], Kinematics_Position_t *position);


/* -------------------------------------------------------------------------- */
/* Public functions                                                           */
/* -------------------------------------------------------------------------- */

float Kinematics_DegToRad(float degrees)
{
    return degrees * (KINEMATICS_PI / 180.0f);
}

float Kinematics_RadToDeg(float radians)
{
    return radians * (180.0f / KINEMATICS_PI);
}

int32_t Kinematics_DegToTickDelta(float degrees)
{
    return Operations_RoundToI32(degrees * KINEMATICS_TICKS_PER_DEG);
}

float Kinematics_TickDeltaToDeg(int32_t ticks)
{
    return ((float)ticks) * KINEMATICS_DEG_PER_TICK;
}

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

    delta_ticks = (int32_t)raw_position - (int32_t)joint->home_position_ticks;
    *angle_deg = Kinematics_TickDeltaToDeg(delta_ticks * (int32_t)direction);

    return SERVO_RESULT_OK;
}

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
    target_raw = (int32_t)joint->home_position_ticks + delta_ticks;

    if ((target_raw < (int32_t)joint->min_position_ticks) ||
        (target_raw > (int32_t)joint->max_position_ticks))
    {
        return SERVO_RESULT_POSITION_OUT_OF_RANGE;
    }

    *raw_position = (uint16_t)target_raw;
    return SERVO_RESULT_OK;
}

Servo_Result_t Kinematics_ForwardRad(const float joint_rad[KINEMATICS_ACTIVE_JOINT_COUNT], Kinematics_Transform_t *transform)
{
    Kinematics_Transform_t total;
    uint8_t active_index = 0U;

    if ((joint_rad == NULL) || (transform == NULL))
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    Operations_SetIdentity(&total);

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

        Operations_LinkTransform(&kinematics_chain[i].pose, q, &link_transform);
        Operations_Multiply(&total, &link_transform, &next_total);
        total = next_total;
    }

    *transform = total;
    return SERVO_RESULT_OK;
}

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

void Kinematics_GetDefaultIkConfig(Kinematics_IkConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->max_iterations = KINEMATICS_IK_DEFAULT_MAX_ITERATIONS;
    config->position_tolerance_m = KINEMATICS_IK_DEFAULT_TOLERANCE_M;
    config->finite_difference_step_deg = KINEMATICS_IK_DEFAULT_FD_STEP_DEG;
    config->damping = KINEMATICS_IK_DEFAULT_DAMPING;
    config->max_step_deg = KINEMATICS_IK_DEFAULT_MAX_STEP_DEG;
}

Servo_Result_t Kinematics_InversePositionDeg(const Kinematics_Position_t *target_position, const float seed_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT], const Kinematics_IkConfig_t *config, float result_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT])
{
    Servo_Result_t result;
    Kinematics_IkConfig_t local_config;
    float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
    float min_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
    float max_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
    float tolerance_squared;

    if ((target_position == NULL) || (seed_joint_deg == NULL) || (result_joint_deg == NULL))
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    if (config == NULL)
    {
        Kinematics_GetDefaultIkConfig(&local_config);
    }
    else
    {
        local_config = *config;
    }

    if (local_config.max_iterations == 0U)
    {
        local_config.max_iterations = KINEMATICS_IK_DEFAULT_MAX_ITERATIONS;
    }

    if (local_config.position_tolerance_m <= 0.0f)
    {
        local_config.position_tolerance_m = KINEMATICS_IK_DEFAULT_TOLERANCE_M;
    }

    if (local_config.finite_difference_step_deg <= 0.0f)
    {
        local_config.finite_difference_step_deg = KINEMATICS_IK_DEFAULT_FD_STEP_DEG;
    }

    if (local_config.damping <= 0.0f)
    {
        local_config.damping = KINEMATICS_IK_DEFAULT_DAMPING;
    }

    if (local_config.max_step_deg <= 0.0f)
    {
        local_config.max_step_deg = KINEMATICS_IK_DEFAULT_MAX_STEP_DEG;
    }

    for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
    {
        result = Kinematics_GetJointLimitsDeg((uint8_t)(KINEMATICS_ACTIVE_FIRST_ID + i), &min_deg[i], &max_deg[i]);
        if (result != SERVO_RESULT_OK)
        {
            return result;
        }

        joint_deg[i] = seed_joint_deg[i];
        Kinematics_ClampFloat(&joint_deg[i], min_deg[i], max_deg[i]);
    }

    tolerance_squared = local_config.position_tolerance_m * local_config.position_tolerance_m;

    for (uint16_t iteration = 0U; iteration < local_config.max_iterations; iteration++)
    {
        Kinematics_Position_t current_position;
        float error[3];
        float error_squared;
        float jacobian[3][KINEMATICS_ACTIVE_JOINT_COUNT];
        float a[3][3];
        float a_inv[3][3];
        float v[3];
        float max_abs_delta = 0.0f;

        result = Kinematics_ForwardPositionDeg(joint_deg, &current_position);
        if (result != SERVO_RESULT_OK)
        {
            return result;
        }

        error[0] = target_position->x - current_position.x;
        error[1] = target_position->y - current_position.y;
        error[2] = target_position->z - current_position.z;

        error_squared = Kinematics_PositionDistanceSquared(target_position, &current_position);

        if (error_squared <= tolerance_squared)
        {
            for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
            {
                result_joint_deg[i] = joint_deg[i];
            }

            return SERVO_RESULT_OK;
        }

        for (uint8_t joint_index = 0U; joint_index < KINEMATICS_ACTIVE_JOINT_COUNT; joint_index++)
        {
            float trial_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
            float actual_step_deg;
            Kinematics_Position_t trial_position;

            for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
            {
                trial_joint_deg[i] = joint_deg[i];
            }

            trial_joint_deg[joint_index] = joint_deg[joint_index] + local_config.finite_difference_step_deg;

            if (trial_joint_deg[joint_index] > max_deg[joint_index])
            {
                trial_joint_deg[joint_index] = joint_deg[joint_index] - local_config.finite_difference_step_deg;
            }

            Kinematics_ClampFloat(&trial_joint_deg[joint_index], min_deg[joint_index], max_deg[joint_index]);

            actual_step_deg = trial_joint_deg[joint_index] - joint_deg[joint_index];

            float actual_step_rad = Kinematics_DegToRad(actual_step_deg);

            if ((actual_step_rad > -0.000001f) && (actual_step_rad < 0.000001f))
            {
                jacobian[0][joint_index] = 0.0f;
                jacobian[1][joint_index] = 0.0f;
                jacobian[2][joint_index] = 0.0f;
            }
            else
            {
                result = Kinematics_ForwardPositionDeg(trial_joint_deg, &trial_position);
                if (result != SERVO_RESULT_OK)
                {
                    return result;
                }

                jacobian[0][joint_index] = (trial_position.x - current_position.x) / actual_step_rad;
                jacobian[1][joint_index] = (trial_position.y - current_position.y) / actual_step_rad;
                jacobian[2][joint_index] = (trial_position.z - current_position.z) / actual_step_rad;
            }
        }

        for (uint8_t row = 0U; row < 3U; row++)
        {
            for (uint8_t col = 0U; col < 3U; col++)
            {
                a[row][col] = 0.0f;

                for (uint8_t joint_index = 0U; joint_index < KINEMATICS_ACTIVE_JOINT_COUNT; joint_index++)
                {
                    a[row][col] += jacobian[row][joint_index] * jacobian[col][joint_index];
                }

                if (row == col)
                {
                    a[row][col] += local_config.damping * local_config.damping;
                }
            }
        }

        if (Operations_Invert3x3(a, a_inv) == 0U)
        {
            return SERVO_RESULT_TARGET_NOT_REACHED;
        }

        for (uint8_t row = 0U; row < 3U; row++)
        {
            v[row] = 0.0f;

            for (uint8_t col = 0U; col < 3U; col++)
            {
                v[row] += a_inv[row][col] * error[col];
            }
        }

        for (uint8_t joint_index = 0U; joint_index < KINEMATICS_ACTIVE_JOINT_COUNT; joint_index++)
        {
            float delta_rad = 0.0f;
            float delta_deg;

            for (uint8_t row = 0U; row < 3U; row++)
            {
                delta_rad += jacobian[row][joint_index] * v[row];
            }

            delta_deg = Kinematics_RadToDeg(delta_rad);

            Kinematics_ClampFloat(&delta_deg, -local_config.max_step_deg, local_config.max_step_deg);

            if (delta_deg < 0.0f)
            {
                if (-delta_deg > max_abs_delta)
                {
                    max_abs_delta = -delta_deg;
                }
            }
            else
            {
                if (delta_deg > max_abs_delta)
                {
                    max_abs_delta = delta_deg;
                }
            }

            joint_deg[joint_index] += delta_deg;
            Kinematics_ClampFloat(&joint_deg[joint_index], min_deg[joint_index], max_deg[joint_index]);
        }
    }

    return SERVO_RESULT_TARGET_NOT_REACHED;
}

Servo_Result_t Kinematics_InversePositionRaw(const Kinematics_Position_t *target_position, const float seed_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT], const Kinematics_IkConfig_t *config, uint16_t result_raw[KINEMATICS_ACTIVE_JOINT_COUNT])
{
    Servo_Result_t result;
    float result_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT];

    if ((target_position == NULL) || (seed_joint_deg == NULL) || (result_raw == NULL))
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    result = Kinematics_InversePositionDeg(target_position, seed_joint_deg, config, result_joint_deg);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
    {
        result = Kinematics_AngleDegToRaw((uint8_t)(KINEMATICS_ACTIVE_FIRST_ID + i), result_joint_deg[i], &result_raw[i]);
        if (result != SERVO_RESULT_OK)
        {
            return result;
        }
    }

    return SERVO_RESULT_OK;
}

Servo_Result_t Kinematics_MoveEndEffectorToPosition(const Kinematics_Position_t *target_position, uint16_t speed, uint8_t acceleration, const Kinematics_IkConfig_t *config)
{
    Servo_Result_t result;
    float seed_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
    uint16_t target_raw[KINEMATICS_ACTIVE_JOINT_COUNT];

    if (target_position == NULL)
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    result = Kinematics_ReadCurrentJointAnglesDeg(seed_joint_deg);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    result = Kinematics_InversePositionRaw(target_position, seed_joint_deg, config, target_raw);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
    {
        result = Servo_WritePosition((uint8_t)(KINEMATICS_ACTIVE_FIRST_ID + i), target_raw[i], speed, acceleration);
        if (result != SERVO_RESULT_OK)
        {
            return result;
        }
    }

    return SERVO_RESULT_OK;
}

Servo_Result_t Kinematics_MoveEndEffectorToPositionAndWait(const Kinematics_Position_t *target_position, uint16_t speed, uint8_t acceleration, uint16_t tolerance_ticks, uint32_t timeout_ms, const Kinematics_IkConfig_t *config, Kinematics_AbortCallback_t abort_callback)
{
    Servo_Result_t result;
    float seed_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
    uint16_t target_raw[KINEMATICS_ACTIVE_JOINT_COUNT];

    if (target_position == NULL)
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    result = Kinematics_ReadCurrentJointAnglesDeg(seed_joint_deg);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    result = Kinematics_InversePositionRaw(target_position, seed_joint_deg, config, target_raw);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    if (abort_callback != NULL)
    {
        if (abort_callback() != 0U)
        {
            return SERVO_RESULT_ABORTED;
        }
    }

    for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
    {
    	if (abort_callback != NULL)
    	{
    	    if (abort_callback() != 0U)
    	    {
    	        return SERVO_RESULT_ABORTED;
    	    }
    	}

        result = Servo_WritePosition((uint8_t)(KINEMATICS_ACTIVE_FIRST_ID + i), target_raw[i], speed, acceleration);
        if (result != SERVO_RESULT_OK)
        {
            return result;
        }
    }

    for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
    {
        result = Kinematics_WaitUntilJointReached((uint8_t)(KINEMATICS_ACTIVE_FIRST_ID + i), target_raw[i], tolerance_ticks, timeout_ms, abort_callback);
        if (result != SERVO_RESULT_OK)
        {
            return result;
        }
    }

    return SERVO_RESULT_OK;
}

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

Servo_Result_t Kinematics_MoveJointRelativeDegAndWait(uint8_t joint_id, float delta_deg, uint16_t speed, uint8_t acceleration, uint16_t tolerance_ticks, uint32_t timeout_ms, Kinematics_AbortCallback_t abort_callback)
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

    return Kinematics_WaitUntilJointReached(joint_id, target_raw, tolerance_ticks, timeout_ms, abort_callback);
}

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

Servo_Result_t Kinematics_MoveJointToAngleDegAndWait(uint8_t joint_id, float angle_deg, uint16_t speed, uint8_t acceleration, uint16_t tolerance_ticks, uint32_t timeout_ms, Kinematics_AbortCallback_t abort_callback)
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

    return Kinematics_WaitUntilJointReached(joint_id, target_raw, tolerance_ticks, timeout_ms, abort_callback);
}

Servo_Result_t Kinematics_WaitUntilJointReached(uint8_t joint_id,
                                                uint16_t target_position_ticks,
                                                uint16_t tolerance_ticks,
                                                uint32_t timeout_ms,
                                                Kinematics_AbortCallback_t abort_callback)
{
    uint32_t start_time;
    uint16_t current_position_ticks = 0U;
    uint16_t error_ticks = 0U;
    Servo_Result_t result;

    start_time = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start_time) < timeout_ms)
    {
        if (abort_callback != NULL)
        {
            if (abort_callback() != 0U)
            {
                return SERVO_RESULT_ABORTED;
            }
        }

        result = Servo_ReadPosition(joint_id, &current_position_ticks);
        if (result != SERVO_RESULT_OK)
        {
            return result;
        }

        if (current_position_ticks >= target_position_ticks)
        {
            error_ticks = (uint16_t)(current_position_ticks - target_position_ticks);
        }
        else
        {
            error_ticks = (uint16_t)(target_position_ticks - current_position_ticks);
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

    if ((target < (int32_t)joint->min_position_ticks) ||
        (target > (int32_t)joint->max_position_ticks))
    {
        return SERVO_RESULT_POSITION_OUT_OF_RANGE;
    }

    *target_raw = (uint16_t)target;
    return SERVO_RESULT_OK;
}

static Servo_Result_t Kinematics_GetJointLimitsDeg(uint8_t joint_id, float *min_deg, float *max_deg)
{
    const Servo_JointConfig_t *joint;
    int8_t direction;
    int32_t min_delta_ticks;
    int32_t max_delta_ticks;
    float a;
    float b;

    if ((min_deg == NULL) || (max_deg == NULL))
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

    min_delta_ticks = ((int32_t)joint->min_position_ticks - (int32_t)joint->home_position_ticks) * (int32_t)direction;
    max_delta_ticks = ((int32_t)joint->max_position_ticks - (int32_t)joint->home_position_ticks) * (int32_t)direction;

    a = Kinematics_TickDeltaToDeg(min_delta_ticks);
    b = Kinematics_TickDeltaToDeg(max_delta_ticks);

    if (a <= b)
    {
        *min_deg = a;
        *max_deg = b;
    }
    else
    {
        *min_deg = b;
        *max_deg = a;
    }

    return SERVO_RESULT_OK;
}

static void Kinematics_ClampFloat(float *value, float min_value, float max_value)
{
    if (value == NULL)
    {
        return;
    }

    if (*value < min_value)
    {
        *value = min_value;
    }
    else if (*value > max_value)
    {
        *value = max_value;
    }
}

static float Kinematics_PositionDistanceSquared(const Kinematics_Position_t *a, const Kinematics_Position_t *b)
{
    float dx;
    float dy;
    float dz;

    if ((a == NULL) || (b == NULL))
    {
        return 0.0f;
    }

    dx = a->x - b->x;
    dy = a->y - b->y;
    dz = a->z - b->z;

    return (dx * dx) + (dy * dy) + (dz * dz);
}

static Servo_Result_t Kinematics_ForwardPositionDeg(const float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT], Kinematics_Position_t *position)
{
    Servo_Result_t result;
    Kinematics_Transform_t transform;

    if ((joint_deg == NULL) || (position == NULL))
    {
        return SERVO_RESULT_NULL_POINTER;
    }

    result = Kinematics_ForwardDeg(joint_deg, &transform);
    if (result != SERVO_RESULT_OK)
    {
        return result;
    }

    return Kinematics_GetPosition(&transform, position);
}
