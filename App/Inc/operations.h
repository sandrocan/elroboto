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

/**
 * @brief Initializes a homogeneous transform as the 4x4 identity matrix.
 * @param out Transform to initialize; NULL is ignored.
 * @return None.
 */
void Operations_SetIdentity(Kinematics_Transform_t *out);

/**
 * @brief Multiplies two homogeneous transforms.
 * @param a Left-hand transform.
 * @param b Right-hand transform.
 * @param out Product transform.
 * @return None. Invalid pointers are ignored.
 */
void Operations_Multiply(
    const Kinematics_Transform_t *a,
    const Kinematics_Transform_t *b,
    Kinematics_Transform_t *out
);

/**
 * @brief Builds one local transform using standard sinf/cosf.
 * @param link Fixed translation and orientation of the link.
 * @param joint_angle_rad Joint rotation in radians.
 * @param out Calculated local transform.
 * @return None.
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
 * @param link Fixed translation and orientation of the link.
 * @param joint_angle_rad Joint rotation in radians.
 * @param out Calculated local transform.
 * @return None.
 */
void Operations_LinkTransformFastMath(
    const Operations_LinkPose_t *link,
    float joint_angle_rad,
    Kinematics_Transform_t *out
);

/**
 * @brief Rounds a floating-point value to the nearest signed 32-bit integer.
 * @param value Value to round.
 * @return Rounded integer value.
 */
int32_t Operations_RoundToI32(float value);

/**
 * @brief Calculates the inverse of a nonsingular 3x3 matrix.
 * @param in Matrix to invert.
 * @param out Calculated inverse on success.
 * @return 1 on success, otherwise 0.
 */
uint8_t Operations_Invert3x3(const float in[3][3], float out[3][3]);

#ifdef __cplusplus
}
#endif

#endif /* OPERATIONS_H_ */
