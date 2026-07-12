#include "tests.h"

#include "kinematics.h"

#include <stdio.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Test configuration                                                         */
/* -------------------------------------------------------------------------- */

#define TESTS_HOME_TOLERANCE_TICKS      50U

#define TESTS_DK_MOVE_DEG               10.0f
#define TESTS_DK_SPEED                  250U
#define TESTS_DK_ACCELERATION           100U
#define TESTS_DK_TOLERANCE_TICKS        50U
#define TESTS_DK_TIMEOUT_MS             10000U
#define TESTS_DK_PAUSE_MS               500U

#define TESTS_IK_MOVE_M                 0.030f
#define TESTS_IK_SPEED                  TESTS_DK_SPEED
#define TESTS_IK_ACCELERATION           TESTS_DK_ACCELERATION
#define TESTS_IK_TOLERANCE_TICKS        50U
#define TESTS_IK_TIMEOUT_MS             15000U
#define TESTS_IK_PAUSE_MS               500U
#define TESTS_IK_POSITION_TOL_M         0.003f
#define TESTS_IK_MAX_ITERATIONS         200U
#define TESTS_IK_COMMAND_SETTLE_MS      100U
#define TESTS_IK_SEED_COUNT             5U

/*
 * Servo_JointConfig_t field adapter.
 */
#define TESTS_JOINT_MIN(joint)   ((joint)->min_position_ticks)
#define TESTS_JOINT_HOME(joint)  ((joint)->home_position_ticks)
#define TESTS_JOINT_MAX(joint)   ((joint)->max_position_ticks)

/* -------------------------------------------------------------------------- */
/* Public test functions                                                      */
/* -------------------------------------------------------------------------- */

Servo_Result_t Tests_HomeTest(void)
{
    Servo_Result_t result;
    uint8_t all_home = 1U;
    const uint8_t joint_count = Servo_GetJointCount();

    printf("\r\n========== HOME TEST START ==========\r\n");

    printf("Configured servo joints: %u\r\n", (unsigned int)joint_count);

    for (uint8_t index = 0U; index < joint_count; ++index)
    {
        const Servo_JointConfig_t *joint = Servo_GetJointConfigByIndex(index);

        if (joint == NULL)
        {
            printf("  missing joint table entry at index=%u\r\n",
                   (unsigned int)index);
            printf("========== HOME TEST FAILED ==========\r\n");
            return SERVO_RESULT_UNKNOWN_JOINT_ID;
        }

        printf("  ID=%u %-14s min=%u home=%u max=%u fixed=%u\r\n",
               (unsigned int)joint->id,
               joint->name,
               (unsigned int)TESTS_JOINT_MIN(joint),
               (unsigned int)TESTS_JOINT_HOME(joint),
               (unsigned int)TESTS_JOINT_MAX(joint),
               (unsigned int)joint->is_fixed);
    }

    printf("Initial home check started: tolerance=%u ticks\r\n",
           (unsigned int)TESTS_HOME_TOLERANCE_TICKS);

    for (uint8_t index = 0U; index < joint_count; ++index)
    {
        const Servo_JointConfig_t *joint = Servo_GetJointConfigByIndex(index);
        uint16_t current_position_ticks = 0U;
        uint16_t error_ticks = 0U;

        if (joint == NULL)
        {
            printf("Initial home check failed: missing joint at index=%u\r\n",
                   (unsigned int)index);
            printf("========== HOME TEST FAILED ==========\r\n");
            return SERVO_RESULT_UNKNOWN_JOINT_ID;
        }

        if (joint->is_fixed != 0U)
        {
            printf("Initial home check skipped fixed joint: ID=%u %-14s\r\n",
                   (unsigned int)joint->id,
                   joint->name);
            continue;
        }

        result = Servo_ReadPosition(joint->id, &current_position_ticks);

        if (result != SERVO_RESULT_OK)
        {
            printf("Initial home check read failed: ID=%u %-14s result=%s\r\n",
                   (unsigned int)joint->id,
                   joint->name,
                   Servo_ResultToString(result));
            printf("========== HOME TEST FAILED ==========\r\n");
            return result;
        }

        error_ticks =
            (current_position_ticks >= TESTS_JOINT_HOME(joint)) ?
            (uint16_t)(current_position_ticks - TESTS_JOINT_HOME(joint)) :
            (uint16_t)(TESTS_JOINT_HOME(joint) - current_position_ticks);

        printf("Initial home check: ID=%u %-14s current=%u home=%u error=%u\r\n",
               (unsigned int)joint->id,
               joint->name,
               (unsigned int)current_position_ticks,
               (unsigned int)TESTS_JOINT_HOME(joint),
               (unsigned int)error_ticks);

        if (error_ticks > TESTS_HOME_TOLERANCE_TICKS)
        {
            all_home = 0U;
        }
    }

    if (all_home != 0U)
    {
        printf("Initial home check: all active joints are already home\r\n");
    }
    else
    {
        printf("Initial home check: at least one active joint is not home\r\n");
    }

    printf("Drive home started\r\n");

    result = Servo_DriveHome();

    printf("Drive home finished: result=%s\r\n", Servo_ResultToString(result));

    if (result != SERVO_RESULT_OK)
    {
        printf("========== HOME TEST FAILED ==========\r\n");
        return result;
    }

    all_home = 1U;

    printf("Final home check started: tolerance=%u ticks\r\n",
           (unsigned int)TESTS_HOME_TOLERANCE_TICKS);

    for (uint8_t index = 0U; index < joint_count; ++index)
    {
        const Servo_JointConfig_t *joint = Servo_GetJointConfigByIndex(index);
        uint16_t current_position_ticks = 0U;
        uint16_t error_ticks = 0U;

        if (joint == NULL)
        {
            printf("Final home check failed: missing joint at index=%u\r\n",
                   (unsigned int)index);
            printf("========== HOME TEST FAILED ==========\r\n");
            return SERVO_RESULT_UNKNOWN_JOINT_ID;
        }

        if (joint->is_fixed != 0U)
        {
            printf("Final home check skipped fixed joint: ID=%u %-14s\r\n",
                   (unsigned int)joint->id,
                   joint->name);
            continue;
        }

        result = Servo_ReadPosition(joint->id, &current_position_ticks);

        if (result != SERVO_RESULT_OK)
        {
            printf("Final home check read failed: ID=%u %-14s result=%s\r\n",
                   (unsigned int)joint->id,
                   joint->name,
                   Servo_ResultToString(result));
            printf("========== HOME TEST FAILED ==========\r\n");
            return result;
        }

        error_ticks =
            (current_position_ticks >= TESTS_JOINT_HOME(joint)) ?
            (uint16_t)(current_position_ticks - TESTS_JOINT_HOME(joint)) :
            (uint16_t)(TESTS_JOINT_HOME(joint) - current_position_ticks);

        printf("Final home check: ID=%u %-14s current=%u home=%u error=%u\r\n",
               (unsigned int)joint->id,
               joint->name,
               (unsigned int)current_position_ticks,
               (unsigned int)TESTS_JOINT_HOME(joint),
               (unsigned int)error_ticks);

        if (error_ticks > TESTS_HOME_TOLERANCE_TICKS)
        {
            all_home = 0U;
        }
    }

    if (all_home == 0U)
    {
        printf("Home test result: result=%s\r\n",
               Servo_ResultToString(SERVO_RESULT_TARGET_NOT_REACHED));
        printf("========== HOME TEST FAILED ==========\r\n");
        return SERVO_RESULT_TARGET_NOT_REACHED;
    }

    printf("========== HOME TEST OK ==========\r\n");
    return SERVO_RESULT_OK;
}

Servo_Result_t Tests_DkTest(void)
{
    Servo_Result_t result;
    float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
    Kinematics_Transform_t transform;
    Kinematics_Position_t position;

    printf("\r\n========== DK TEST START ==========\r\n");
    printf("Moving active joints 1..%u by +%.1f deg and back\r\n",
           (unsigned int)KINEMATICS_ACTIVE_JOINT_COUNT,
           TESTS_DK_MOVE_DEG);

    result = Kinematics_ReadCurrentJointAnglesDeg(joint_deg);
    if (result != SERVO_RESULT_OK)
    {
        printf("Read current joint angles failed: result=%s\r\n",
               Servo_ResultToString(result));
        printf("========== DK TEST FAILED ==========\r\n");
        return result;
    }

    printf("Current joint angles: q1=%.2f q2=%.2f q3=%.2f q4=%.2f deg\r\n",
           joint_deg[0],
           joint_deg[1],
           joint_deg[2],
           joint_deg[3]);

    result = Kinematics_ForwardDeg(joint_deg, &transform);
    if (result != SERVO_RESULT_OK)
    {
        printf("Forward kinematics failed: result=%s\r\n",
               Servo_ResultToString(result));
        printf("========== DK TEST FAILED ==========\r\n");
        return result;
    }

    result = Kinematics_GetPosition(&transform, &position);
    if (result != SERVO_RESULT_OK)
    {
        printf("Extract end-effector position failed: result=%s\r\n",
               Servo_ResultToString(result));
        printf("========== DK TEST FAILED ==========\r\n");
        return result;
    }

    printf("End effector position: x=%.6f y=%.6f z=%.6f\r\n",
           position.x,
           position.y,
           position.z);

    for (uint8_t joint_index = 0U; joint_index < KINEMATICS_ACTIVE_JOINT_COUNT; joint_index++)
    {
        uint8_t joint_id = (uint8_t)(joint_index + 1U);

        printf("\r\nDK test: move joint %u by +%.1f deg\r\n",
               (unsigned int)joint_id,
               TESTS_DK_MOVE_DEG);

        result = Kinematics_MoveJointRelativeDegAndWait(
            joint_id,
            TESTS_DK_MOVE_DEG,
            TESTS_DK_SPEED,
            TESTS_DK_ACCELERATION,
            TESTS_DK_TOLERANCE_TICKS,
            TESTS_DK_TIMEOUT_MS,
            NULL
        );

        if (result != SERVO_RESULT_OK)
        {
            printf("DK move +deg failed: joint=%u result=%s\r\n",
                   (unsigned int)joint_id,
                   Servo_ResultToString(result));
            printf("========== DK TEST FAILED ==========\r\n");
            return result;
        }

        result = Kinematics_ReadCurrentJointAnglesDeg(joint_deg);
        if (result != SERVO_RESULT_OK)
        {
            printf("Read current joint angles failed: result=%s\r\n",
                   Servo_ResultToString(result));
            printf("========== DK TEST FAILED ==========\r\n");
            return result;
        }

        printf("Current joint angles: q1=%.2f q2=%.2f q3=%.2f q4=%.2f deg\r\n",
               joint_deg[0],
               joint_deg[1],
               joint_deg[2],
               joint_deg[3]);

        result = Kinematics_ReadCurrentEndEffector(&transform);
        if (result != SERVO_RESULT_OK)
        {
            printf("Read current end effector failed: result=%s\r\n",
                   Servo_ResultToString(result));
            printf("========== DK TEST FAILED ==========\r\n");
            return result;
        }

        result = Kinematics_GetPosition(&transform, &position);
        if (result != SERVO_RESULT_OK)
        {
            printf("Extract end-effector position failed: result=%s\r\n",
                   Servo_ResultToString(result));
            printf("========== DK TEST FAILED ==========\r\n");
            return result;
        }

        printf("End effector position: x=%.6f y=%.6f z=%.6f\r\n",
               position.x,
               position.y,
               position.z);

        HAL_Delay(TESTS_DK_PAUSE_MS);

        printf("DK test: move joint %u by -%.1f deg\r\n",
               (unsigned int)joint_id,
               TESTS_DK_MOVE_DEG);

        result = Kinematics_MoveJointRelativeDegAndWait(
            joint_id,
            -TESTS_DK_MOVE_DEG,
            TESTS_DK_SPEED,
            TESTS_DK_ACCELERATION,
            TESTS_DK_TOLERANCE_TICKS,
            TESTS_DK_TIMEOUT_MS,
            NULL
        );

        HAL_Delay(20U);

        if (result != SERVO_RESULT_OK)
        {
            printf("DK move -deg failed: joint=%u result=%s\r\n",
                   (unsigned int)joint_id,
                   Servo_ResultToString(result));
            printf("========== DK TEST FAILED ==========\r\n");
            return result;
        }

        result = Kinematics_ReadCurrentJointAnglesDeg(joint_deg);
        if (result != SERVO_RESULT_OK)
        {
            printf("Read current joint angles failed: result=%s\r\n",
                   Servo_ResultToString(result));
            printf("========== DK TEST FAILED ==========\r\n");
            return result;
        }

        printf("Current joint angles: q1=%.2f q2=%.2f q3=%.2f q4=%.2f deg\r\n",
               joint_deg[0],
               joint_deg[1],
               joint_deg[2],
               joint_deg[3]);

        result = Kinematics_ReadCurrentEndEffector(&transform);
        if (result != SERVO_RESULT_OK)
        {
            printf("Read current end effector failed: result=%s\r\n",
                   Servo_ResultToString(result));
            printf("========== DK TEST FAILED ==========\r\n");
            return result;
        }

        result = Kinematics_GetPosition(&transform, &position);
        if (result != SERVO_RESULT_OK)
        {
            printf("Extract end-effector position failed: result=%s\r\n",
                   Servo_ResultToString(result));
            printf("========== DK TEST FAILED ==========\r\n");
            return result;
        }

        printf("End effector position: x=%.6f y=%.6f z=%.6f\r\n",
               position.x,
               position.y,
               position.z);

        HAL_Delay(TESTS_DK_PAUSE_MS);
    }

    printf("========== DK TEST OK ==========\r\n");
    return SERVO_RESULT_OK;
}

Servo_Result_t Tests_IkTest(void)
{
    Servo_Result_t result;
    Kinematics_Transform_t transform;
    Kinematics_Transform_t solved_transform;
    Kinematics_Position_t start_position;
    Kinematics_Position_t target_position;
    Kinematics_Position_t move_position;
    Kinematics_Position_t reached_position;
    Kinematics_Position_t solved_position;
    Kinematics_IkConfig_t ik_config;

    float current_seed_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
    float test_seed_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
    float solved_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
    uint16_t target_raw[KINEMATICS_ACTIVE_JOINT_COUNT];

    const float seed_offsets[TESTS_IK_SEED_COUNT][KINEMATICS_ACTIVE_JOINT_COUNT] =
    {
        { 0.0f,   0.0f,   0.0f,   0.0f  },
        { 0.0f,  10.0f, -20.0f,  10.0f  },
        { 0.0f, -10.0f,  20.0f, -10.0f  },
        { 0.0f,  15.0f, -30.0f,  15.0f  },
        { 0.0f, -15.0f,  30.0f, -15.0f  }
    };

    printf("\r\n========== IK TEST START ==========\r\n");
    printf("Moving end effector by %.3f m in -X, -Y and -Z, each time back to start\r\n",
           TESTS_IK_MOVE_M);

    /* Now set up the IK config we want for this hardware test. */
    Kinematics_GetDefaultIkConfig(&ik_config);
    ik_config.position_tolerance_m = TESTS_IK_POSITION_TOL_M;
    ik_config.max_iterations = TESTS_IK_MAX_ITERATIONS;
    ik_config.max_step_deg = 5.0f;
    ik_config.finite_difference_step_deg = 0.5f;
    ik_config.damping = 0.02f;

    /* Now read the current TCP once, so the test knows where it starts. */
    result = Kinematics_ReadCurrentEndEffector(&transform);
    if (result != SERVO_RESULT_OK)
    {
        printf("Read current end effector failed: result=%s\r\n",
               Servo_ResultToString(result));
        printf("========== IK TEST FAILED ==========\r\n");
        return result;
    }

    result = Kinematics_GetPosition(&transform, &start_position);
    if (result != SERVO_RESULT_OK)
    {
        printf("Extract start position failed: result=%s\r\n",
               Servo_ResultToString(result));
        printf("========== IK TEST FAILED ==========\r\n");
        return result;
    }

    printf("Start position: x=%.6f y=%.6f z=%.6f\r\n",
           start_position.x,
           start_position.y,
           start_position.z);

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        target_position = start_position;

        /* Now choose the cartesian target for this axis. */
        if (axis == 0U)
        {
            target_position.x -= TESTS_IK_MOVE_M;
            printf("\r\nIK test: move end effector in -X\r\n");
        }
        else if (axis == 1U)
        {
            target_position.y -= TESTS_IK_MOVE_M;
            printf("\r\nIK test: move end effector in -Y\r\n");
        }
        else
        {
            target_position.z -= TESTS_IK_MOVE_M;
            printf("\r\nIK test: move end effector in -Z\r\n");
        }

        printf("Target position: x=%.6f y=%.6f z=%.6f\r\n",
               target_position.x,
               target_position.y,
               target_position.z);

        for (uint8_t phase = 0U; phase < 2U; phase++)
        {
            uint8_t solution_found = 0U;

            if (phase == 0U)
            {
                move_position = target_position;
                printf("Now move to the IK target.\r\n");
            }
            else
            {
                move_position = start_position;
                printf("Now move back to the start position.\r\n");
                printf("Return target: x=%.6f y=%.6f z=%.6f\r\n",
                       move_position.x,
                       move_position.y,
                       move_position.z);
            }

            /* Now read the current joint angles, because IK needs a useful seed. */
            result = Kinematics_ReadCurrentJointAnglesDeg(current_seed_deg);
            if (result != SERVO_RESULT_OK)
            {
                printf("IK seed read failed: result=%s\r\n",
                       Servo_ResultToString(result));
                printf("========== IK TEST FAILED ==========\r\n");
                return result;
            }

            printf("Current seed angles: q1=%.2f q2=%.2f q3=%.2f q4=%.2f deg\r\n",
                   current_seed_deg[0],
                   current_seed_deg[1],
                   current_seed_deg[2],
                   current_seed_deg[3]);

            /* Now try a few seeds, because straight poses can be annoying for numerical IK. */
            for (uint8_t attempt = 0U; attempt < TESTS_IK_SEED_COUNT; attempt++)
            {
                for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
                {
                    test_seed_deg[i] = current_seed_deg[i] + seed_offsets[attempt][i];
                }

                printf("IK seed attempt %u: q1=%.2f q2=%.2f q3=%.2f q4=%.2f deg\r\n",
                       (unsigned int)attempt,
                       test_seed_deg[0],
                       test_seed_deg[1],
                       test_seed_deg[2],
                       test_seed_deg[3]);

                result = Kinematics_InversePositionDeg(
                    &move_position,
                    test_seed_deg,
                    &ik_config,
                    solved_joint_deg
                );

                if (result == SERVO_RESULT_OK)
                {
                    solution_found = 1U;
                    break;
                }

                printf("IK seed attempt %u failed: result=%s\r\n",
                       (unsigned int)attempt,
                       Servo_ResultToString(result));
            }

            if (solution_found == 0U)
            {
                printf("IK solve failed for all seed attempts before servo movement\r\n");
                printf("========== IK TEST FAILED ==========\r\n");
                return SERVO_RESULT_TARGET_NOT_REACHED;
            }

            printf("Solved joint angles: q1=%.2f q2=%.2f q3=%.2f q4=%.2f deg\r\n",
                   solved_joint_deg[0],
                   solved_joint_deg[1],
                   solved_joint_deg[2],
                   solved_joint_deg[3]);

            /* Now check the math only: solved angles through FK should land near the target. */
            result = Kinematics_ForwardDeg(solved_joint_deg, &solved_transform);
            if (result != SERVO_RESULT_OK)
            {
                printf("Solved FK failed: result=%s\r\n",
                       Servo_ResultToString(result));
                printf("========== IK TEST FAILED ==========\r\n");
                return result;
            }

            result = Kinematics_GetPosition(&solved_transform, &solved_position);
            if (result != SERVO_RESULT_OK)
            {
                printf("Solved FK position extract failed: result=%s\r\n",
                       Servo_ResultToString(result));
                printf("========== IK TEST FAILED ==========\r\n");
                return result;
            }

            printf("Solved FK position: x=%.6f y=%.6f z=%.6f\r\n",
                   solved_position.x,
                   solved_position.y,
                   solved_position.z);

            printf("Solved FK error: dx=%.6f dy=%.6f dz=%.6f\r\n",
                   move_position.x - solved_position.x,
                   move_position.y - solved_position.y,
                   move_position.z - solved_position.z);

            /* Now convert the solved angles to raw servo targets. */
            for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
            {
                uint8_t joint_id = (uint8_t)(i + 1U);

                result = Kinematics_AngleDegToRaw(
                    joint_id,
                    solved_joint_deg[i],
                    &target_raw[i]
                );

                if (result != SERVO_RESULT_OK)
                {
                    printf("Angle to raw failed: joint=%u result=%s\r\n",
                           (unsigned int)joint_id,
                           Servo_ResultToString(result));
                    printf("========== IK TEST FAILED ==========\r\n");
                    return result;
                }

                printf("Target raw: joint=%u raw=%u\r\n",
                       (unsigned int)joint_id,
                       (unsigned int)target_raw[i]);
            }

            /* Now command the real servos. If this fails, it is hardware/communication side. */
            for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
            {
                uint8_t joint_id = (uint8_t)(i + 1U);

                result = Servo_WritePosition(
                    joint_id,
                    target_raw[i],
                    TESTS_IK_SPEED,
                    TESTS_IK_ACCELERATION
                );

                if (result != SERVO_RESULT_OK)
                {
                    printf("Servo write failed: joint=%u result=%s\r\n",
                           (unsigned int)joint_id,
                           Servo_ResultToString(result));
                    printf("========== IK TEST FAILED ==========\r\n");
                    return result;
                }

                HAL_Delay(10U);
            }

            HAL_Delay(TESTS_IK_COMMAND_SETTLE_MS);

            /* Now wait until each joint is close enough to the target raw value. */
            for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
            {
                uint8_t joint_id = (uint8_t)(i + 1U);

                result = Kinematics_WaitUntilJointReached(
                    joint_id,
                    target_raw[i],
                    TESTS_IK_TOLERANCE_TICKS,
                    TESTS_IK_TIMEOUT_MS,
                    NULL
                );

                if (result != SERVO_RESULT_OK)
                {
                    printf("Wait failed: joint=%u target_raw=%u result=%s\r\n",
                           (unsigned int)joint_id,
                           (unsigned int)target_raw[i],
                           Servo_ResultToString(result));
                    printf("========== IK TEST FAILED ==========\r\n");
                    return result;
                }

                HAL_Delay(10U);
            }

            /* Now print the final raw values, so we see what the servos actually did. */
            for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
            {
                uint8_t joint_id = (uint8_t)(i + 1U);
                uint16_t current_raw = 0U;

                result = Servo_ReadPosition(joint_id, &current_raw);
                if (result == SERVO_RESULT_OK)
                {
                    printf("Final raw: joint=%u current=%u target=%u error=%ld\r\n",
                           (unsigned int)joint_id,
                           (unsigned int)current_raw,
                           (unsigned int)target_raw[i],
                           (long)((int32_t)current_raw - (int32_t)target_raw[i]));
                }
                else
                {
                    printf("Final raw read failed: joint=%u result=%s\r\n",
                           (unsigned int)joint_id,
                           Servo_ResultToString(result));
                }

                HAL_Delay(10U);
            }

            /* Now read the real TCP again, so we can compare math target and hardware result. */
            result = Kinematics_ReadCurrentEndEffector(&transform);
            if (result != SERVO_RESULT_OK)
            {
                printf("Read reached end effector failed: result=%s\r\n",
                       Servo_ResultToString(result));
                printf("========== IK TEST FAILED ==========\r\n");
                return result;
            }

            result = Kinematics_GetPosition(&transform, &reached_position);
            if (result != SERVO_RESULT_OK)
            {
                printf("Extract reached position failed: result=%s\r\n",
                       Servo_ResultToString(result));
                printf("========== IK TEST FAILED ==========\r\n");
                return result;
            }

            if (phase == 0U)
            {
                printf("Reached position: x=%.6f y=%.6f z=%.6f\r\n",
                       reached_position.x,
                       reached_position.y,
                       reached_position.z);

                printf("Cartesian error to target: dx=%.6f dy=%.6f dz=%.6f\r\n",
                       move_position.x - reached_position.x,
                       move_position.y - reached_position.y,
                       move_position.z - reached_position.z);
            }
            else
            {
                printf("Returned position: x=%.6f y=%.6f z=%.6f\r\n",
                       reached_position.x,
                       reached_position.y,
                       reached_position.z);

                printf("Cartesian return error: dx=%.6f dy=%.6f dz=%.6f\r\n",
                       move_position.x - reached_position.x,
                       move_position.y - reached_position.y,
                       move_position.z - reached_position.z);

                /* Now accept the real returned position as the next start, because hardware drifts a bit. */
                start_position = reached_position;
            }

            HAL_Delay(TESTS_IK_PAUSE_MS);
        }
    }

    printf("========== IK TEST OK ==========\r\n");
    return SERVO_RESULT_OK;
}
