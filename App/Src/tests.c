#include "tests.h"

#include "kinematics.h"

#include <stdio.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Test configuration                                                         */
/* -------------------------------------------------------------------------- */

#define TESTS_HOME_TOLERANCE_TICKS      50U

#define TESTS_DK_MOVE_DEG               10.0f
#define TESTS_DK_SPEED                  120U
#define TESTS_DK_ACCELERATION           10U
#define TESTS_DK_TOLERANCE_TICKS        15U
#define TESTS_DK_TIMEOUT_MS             10000U
#define TESTS_DK_PAUSE_MS               500U

/*
 * Servo_JointConfig_t field adapter.

 */
#define TESTS_JOINT_MIN(joint)   ((joint)->min_position_ticks)
#define TESTS_JOINT_HOME(joint)  ((joint)->home_position_ticks)
#define TESTS_JOINT_MAX(joint)   ((joint)->max_position_ticks)

/* -------------------------------------------------------------------------- */
/* Public test functions                                                      */
/* -------------------------------------------------------------------------- */

Servo_Result_t Tests_HomeTest(void) {
	Servo_Result_t result;
	uint8_t all_home = 1U;
	const uint8_t joint_count = Servo_GetJointCount();

	printf("\r\n========== HOME TEST START ==========\r\n");

	printf("Configured servo joints: %u\r\n", (unsigned int) joint_count);

	for (uint8_t index = 0U; index < joint_count; ++index) {
		const Servo_JointConfig_t *joint = Servo_GetJointConfigByIndex(index);

		if (joint == NULL) {
			printf("  missing joint table entry at index=%u\r\n",
					(unsigned int) index);
			printf("========== HOME TEST FAILED ==========\r\n");
			return SERVO_RESULT_UNKNOWN_JOINT_ID;
		}

		printf("  ID=%u %-14s min=%u home=%u max=%u fixed=%u\r\n",
				(unsigned int) joint->id, joint->name,
				(unsigned int) TESTS_JOINT_MIN(joint),
				(unsigned int) TESTS_JOINT_HOME(joint),
				(unsigned int) TESTS_JOINT_MAX(joint),
				(unsigned int) joint->is_fixed);
	}

	printf("Initial home check started: tolerance=%u ticks\r\n",
			(unsigned int) TESTS_HOME_TOLERANCE_TICKS);

	for (uint8_t index = 0U; index < joint_count; ++index) {
		const Servo_JointConfig_t *joint = Servo_GetJointConfigByIndex(index);
		uint16_t current_position_ticks = 0U;
		uint16_t error_ticks = 0U;

		if (joint == NULL) {
			printf("Initial home check failed: missing joint at index=%u\r\n",
					(unsigned int) index);
			printf("========== HOME TEST FAILED ==========\r\n");
			return SERVO_RESULT_UNKNOWN_JOINT_ID;
		}

		if (joint->is_fixed != 0U) {
			printf("Initial home check skipped fixed joint: ID=%u %-14s\r\n",
					(unsigned int) joint->id, joint->name);
			continue;
		}

		result = Servo_ReadPosition(joint->id, &current_position_ticks);

		if (result != SERVO_RESULT_OK) {
			printf("Initial home check read failed: ID=%u %-14s result=%s\r\n",
					(unsigned int) joint->id, joint->name,
					Servo_ResultToString(result));
			printf("========== HOME TEST FAILED ==========\r\n");
			return result;
		}

		error_ticks =
				(current_position_ticks >= TESTS_JOINT_HOME(joint)) ?
						(uint16_t) (current_position_ticks
								- TESTS_JOINT_HOME(joint)) :
						(uint16_t) (TESTS_JOINT_HOME(joint)
								- current_position_ticks);

		printf(
				"Initial home check: ID=%u %-14s current=%u home=%u error=%u\r\n",
				(unsigned int) joint->id, joint->name,
				(unsigned int) current_position_ticks,
				(unsigned int) TESTS_JOINT_HOME(joint),
				(unsigned int) error_ticks);

		if (error_ticks > TESTS_HOME_TOLERANCE_TICKS) {
			all_home = 0U;
		}
	}

	if (all_home != 0U) {
		printf("Initial home check: all active joints are already home\r\n");
	} else {
		printf("Initial home check: at least one active joint is not home\r\n");
	}

	printf("Drive home started\r\n");

	result = Servo_DriveHome();

	printf("Drive home finished: result=%s\r\n", Servo_ResultToString(result));

	if (result != SERVO_RESULT_OK) {
		printf("========== HOME TEST FAILED ==========\r\n");
		return result;
	}

	all_home = 1U;

	printf("Final home check started: tolerance=%u ticks\r\n",
			(unsigned int) TESTS_HOME_TOLERANCE_TICKS);

	for (uint8_t index = 0U; index < joint_count; ++index) {
		const Servo_JointConfig_t *joint = Servo_GetJointConfigByIndex(index);
		uint16_t current_position_ticks = 0U;
		uint16_t error_ticks = 0U;

		if (joint == NULL) {
			printf("Final home check failed: missing joint at index=%u\r\n",
					(unsigned int) index);
			printf("========== HOME TEST FAILED ==========\r\n");
			return SERVO_RESULT_UNKNOWN_JOINT_ID;
		}

		if (joint->is_fixed != 0U) {
			printf("Final home check skipped fixed joint: ID=%u %-14s\r\n",
					(unsigned int) joint->id, joint->name);
			continue;
		}

		result = Servo_ReadPosition(joint->id, &current_position_ticks);

		if (result != SERVO_RESULT_OK) {
			printf("Final home check read failed: ID=%u %-14s result=%s\r\n",
					(unsigned int) joint->id, joint->name,
					Servo_ResultToString(result));
			printf("========== HOME TEST FAILED ==========\r\n");
			return result;
		}

		error_ticks =
				(current_position_ticks >= TESTS_JOINT_HOME(joint)) ?
						(uint16_t) (current_position_ticks
								- TESTS_JOINT_HOME(joint)) :
						(uint16_t) (TESTS_JOINT_HOME(joint)
								- current_position_ticks);

		printf("Final home check: ID=%u %-14s current=%u home=%u error=%u\r\n",
				(unsigned int) joint->id, joint->name,
				(unsigned int) current_position_ticks,
				(unsigned int) TESTS_JOINT_HOME(joint),
				(unsigned int) error_ticks);

		if (error_ticks > TESTS_HOME_TOLERANCE_TICKS) {
			all_home = 0U;
		}
	}

	if (all_home == 0U) {
		printf("Home test result: result=%s\r\n",
				Servo_ResultToString(SERVO_RESULT_TARGET_NOT_REACHED));
		printf("========== HOME TEST FAILED ==========\r\n");
		return SERVO_RESULT_TARGET_NOT_REACHED;
	}

	printf("========== HOME TEST OK ==========\r\n");
	return SERVO_RESULT_OK;
}

Servo_Result_t Tests_DkTest(void) {
	Servo_Result_t result;
	float joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT];
	Kinematics_Transform_t transform;
	Kinematics_Position_t position;

	printf("\r\n========== DK TEST START ==========\r\n");
	printf("Moving active joints 1..4 by +%.1f deg and back\r\n",
	TESTS_DK_MOVE_DEG);

	result = Kinematics_ReadCurrentJointAnglesDeg(joint_deg);

	if (result != SERVO_RESULT_OK) {
		printf("Read current joint angles failed: result=%s\r\n",
				Servo_ResultToString(result));
		printf("========== DK TEST FAILED ==========\r\n");
		return result;
	}

	printf("Current joint angles: q1=%.2f q2=%.2f q3=%.2f q4=%.2f deg\r\n",
			joint_deg[0], joint_deg[1], joint_deg[2], joint_deg[3]);

	result = Kinematics_ReadCurrentEndEffector(&transform);

	if (result != SERVO_RESULT_OK) {
		printf("Read current end effector failed: result=%s\r\n",
				Servo_ResultToString(result));
		printf("========== DK TEST FAILED ==========\r\n");
		return result;
	}

	result = Kinematics_GetPosition(&transform, &position);

	if (result != SERVO_RESULT_OK) {
		printf("Extract end-effector position failed: result=%s\r\n",
				Servo_ResultToString(result));
		printf("========== DK TEST FAILED ==========\r\n");
		return result;
	}

	printf("End effector position: x=%.6f y=%.6f z=%.6f\r\n", position.x,
			position.y, position.z);

	for (uint8_t joint_id = 1U; joint_id <= KINEMATICS_ACTIVE_JOINT_COUNT;
			++joint_id) {
		printf("\r\nDK test: move joint %u by +%.1f deg\r\n",
				(unsigned int) joint_id,
				TESTS_DK_MOVE_DEG);

		result = Kinematics_MoveJointRelativeDegAndWait(joint_id,
		TESTS_DK_MOVE_DEG,
		TESTS_DK_SPEED,
		TESTS_DK_ACCELERATION,
		TESTS_DK_TOLERANCE_TICKS,
		TESTS_DK_TIMEOUT_MS);

		if (result != SERVO_RESULT_OK) {
			printf("DK move +deg failed: joint=%u result=%s\r\n",
					(unsigned int) joint_id, Servo_ResultToString(result));
			printf("========== DK TEST FAILED ==========\r\n");
			return result;
		}

		result = Kinematics_ReadCurrentJointAnglesDeg(joint_deg);

		if (result != SERVO_RESULT_OK) {
			printf("Read current joint angles failed: result=%s\r\n",
					Servo_ResultToString(result));
			printf("========== DK TEST FAILED ==========\r\n");
			return result;
		}

		printf("Current joint angles: q1=%.2f q2=%.2f q3=%.2f q4=%.2f deg\r\n",
				joint_deg[0], joint_deg[1], joint_deg[2], joint_deg[3]);

		result = Kinematics_ReadCurrentEndEffector(&transform);

		if (result != SERVO_RESULT_OK) {
			printf("Read current end effector failed: result=%s\r\n",
					Servo_ResultToString(result));
			printf("========== DK TEST FAILED ==========\r\n");
			return result;
		}

		result = Kinematics_GetPosition(&transform, &position);

		if (result != SERVO_RESULT_OK) {
			printf("Extract end-effector position failed: result=%s\r\n",
					Servo_ResultToString(result));
			printf("========== DK TEST FAILED ==========\r\n");
			return result;
		}

		printf("End effector position: x=%.6f y=%.6f z=%.6f\r\n", position.x,
				position.y, position.z);

		HAL_Delay(TESTS_DK_PAUSE_MS);

		printf("DK test: move joint %u by -%.1f deg\r\n",
				(unsigned int) joint_id,
				TESTS_DK_MOVE_DEG);

		result = Kinematics_MoveJointRelativeDegAndWait(joint_id,
				-TESTS_DK_MOVE_DEG,
				TESTS_DK_SPEED,
				TESTS_DK_ACCELERATION,
				TESTS_DK_TOLERANCE_TICKS,
				TESTS_DK_TIMEOUT_MS);

		if (result != SERVO_RESULT_OK) {
			printf("DK move -deg failed: joint=%u result=%s\r\n",
					(unsigned int) joint_id, Servo_ResultToString(result));
			printf("========== DK TEST FAILED ==========\r\n");
			return result;
		}

		result = Kinematics_ReadCurrentJointAnglesDeg(joint_deg);

		if (result != SERVO_RESULT_OK) {
			printf("Read current joint angles failed: result=%s\r\n",
					Servo_ResultToString(result));
			printf("========== DK TEST FAILED ==========\r\n");
			return result;
		}

		printf("Current joint angles: q1=%.2f q2=%.2f q3=%.2f q4=%.2f deg\r\n",
				joint_deg[0], joint_deg[1], joint_deg[2], joint_deg[3]);

		result = Kinematics_ReadCurrentEndEffector(&transform);

		if (result != SERVO_RESULT_OK) {
			printf("Read current end effector failed: result=%s\r\n",
					Servo_ResultToString(result));
			printf("========== DK TEST FAILED ==========\r\n");
			return result;
		}

		result = Kinematics_GetPosition(&transform, &position);

		if (result != SERVO_RESULT_OK) {
			printf("Extract end-effector position failed: result=%s\r\n",
					Servo_ResultToString(result));
			printf("========== DK TEST FAILED ==========\r\n");
			return result;
		}

		printf("End effector position: x=%.6f y=%.6f z=%.6f\r\n", position.x,
				position.y, position.z);

		HAL_Delay(TESTS_DK_PAUSE_MS);
	}

	printf("========== DK TEST OK ==========\r\n");
	return SERVO_RESULT_OK;
}
