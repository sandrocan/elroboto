/**
 ******************************************************************************
 * @file           : operations.h
 * @author         : Niklas Peter
 * @brief          : Runtime math operations for kinematics
 ******************************************************************************
 */

#ifndef OPERATIONS_H_
#define OPERATIONS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Public defines                                                             */
/* -------------------------------------------------------------------------- */

#define KINEMATICS_MATRIX_SIZE          4U

#define OPERATIONS_PI                   3.14159265358979323846f
#define OPERATIONS_DETERMINANT_EPSILON  1.0e-12f

/* -------------------------------------------------------------------------- */
/* Public types                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Stores a 4x4 homogeneous transformation matrix.
 */
typedef struct
{
    float m[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE];
} Kinematics_Transform_t;

/**
 * @brief Stores the fixed pose parameters of one kinematic link.
 */
typedef struct
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
} Operations_LinkPose_t;

/* -------------------------------------------------------------------------- */
/* Public functions                                                           */
/* -------------------------------------------------------------------------- */

void Operations_SetIdentity(Kinematics_Transform_t *out);

void Operations_Multiply(
    const Kinematics_Transform_t *a,
    const Kinematics_Transform_t *b,
    Kinematics_Transform_t *out
);

/**
 * @brief Builds one local transform using standard sinf/cosf.
 */
void Operations_LinkTransform(
    const Operations_LinkPose_t *link,
    float joint_angle_rad,
    Kinematics_Transform_t *out
);

/**
 * @brief Builds one local transform using ARM FastMath if enabled.
 *
 * If OPERATIONS_USE_CMSIS_DSP is not defined, this function falls back to
 * standard sinf/cosf so the project stays buildable.
 */
void Operations_LinkTransformFastMath(
    const Operations_LinkPose_t *link,
    float joint_angle_rad,
    Kinematics_Transform_t *out
);

int32_t Operations_RoundToI32(float value);

uint8_t Operations_Invert3x3(const float in[3][3], float out[3][3]);

#ifdef __cplusplus
}
#endif

#endif /* OPERATIONS_H_ */
