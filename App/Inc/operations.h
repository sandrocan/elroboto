/**
 ******************************************************************************
 * @file           : operations.h / operations.c
 * @author         : Niklas Peter
 * @brief          : All relevant math operations for kinematics
 * 					 -> Additional versions of the same operations for
 * 					 optimization purposes
 ******************************************************************************
 */
#ifndef OPERATIONS_H_
#define OPERATIONS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define KINEMATICS_MATRIX_SIZE   4U
#define OPERATIONS_PI            3.14159265358979323846f
#define OPERATIONS_DETERMINANT_EPSILON   1.0e-12f

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

/**
 * @brief Sets a transform matrix to the 4x4 identity matrix.
 * @param out Pointer to the transform matrix to initialize.
 * @return None.
 */
void Operations_SetIdentity(Kinematics_Transform_t *out);

/**
 * @brief Multiplies two 4x4 transform matrices using row-by-column multiplication.
 * @param a Pointer to the left input transform matrix.
 * @param b Pointer to the right input transform matrix.
 * @param out Pointer where the resulting transform matrix will be stored.
 * @return None.
 */
void Operations_Multiply(const Kinematics_Transform_t *a, const Kinematics_Transform_t *b, Kinematics_Transform_t *out);

/**
 * @brief Builds the local transform for one kinematic chain element.
 * @param link Pointer to the fixed link pose parameters.
 * @param joint_angle_rad Joint angle in radians for active joints, or 0 for fixed links.
 * @param out Pointer where the local transform will be stored.
 * @return None.
 */
void Operations_LinkTransform(const Operations_LinkPose_t *link, float joint_angle_rad, Kinematics_Transform_t *out);

/**
 * @brief Rounds a float value to the nearest int32_t value.
 * @param value Float value to round.
 * @return Rounded integer value.
 */
int32_t Operations_RoundToI32(float value);

/**
 * @brief Inverts a 3x3 matrix.
 * @param in Input 3x3 matrix.
 * @param out Output 3x3 inverse matrix.
 * @return 1 if the matrix was inverted, otherwise 0.
 */
uint8_t Operations_Invert3x3(const float in[3][3], float out[3][3]);

#ifdef __cplusplus
}
#endif

#endif /* OPERATIONS_H_ */
