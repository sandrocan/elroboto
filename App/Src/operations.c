#include "operations.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#if defined(OPERATIONS_USE_CMSIS_DSP)
#include "arm_math.h"
#endif

/* -------------------------------------------------------------------------- */
/* Private function prototypes                                                */
/* -------------------------------------------------------------------------- */

static void Operations_Mat4Copy(
    const float src[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    float dst[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE]
);

static void Operations_MultiplyHomogeneousInternal(
    const float A[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    const float B[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    float C[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE]
);

static void Operations_LinkTransformInternal(
    const Operations_LinkPose_t *link,
    float joint_angle_rad,
    Kinematics_Transform_t *out,
    uint8_t use_fast_math
);

static float Operations_Sin(float x, uint8_t use_fast_math);
static float Operations_Cos(float x, uint8_t use_fast_math);

/* -------------------------------------------------------------------------- */
/* Public functions                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initializes a homogeneous transform as the 4x4 identity matrix.
 * @param out Transform to initialize; NULL is ignored.
 * @return None. Writes the identity matrix to out.
 */
void Operations_SetIdentity(Kinematics_Transform_t *out)
{
    if (out == NULL)
    {
        return;
    }

    out->m[0][0] = 1.0f;
    out->m[0][1] = 0.0f;
    out->m[0][2] = 0.0f;
    out->m[0][3] = 0.0f;

    out->m[1][0] = 0.0f;
    out->m[1][1] = 1.0f;
    out->m[1][2] = 0.0f;
    out->m[1][3] = 0.0f;

    out->m[2][0] = 0.0f;
    out->m[2][1] = 0.0f;
    out->m[2][2] = 1.0f;
    out->m[2][3] = 0.0f;

    out->m[3][0] = 0.0f;
    out->m[3][1] = 0.0f;
    out->m[3][2] = 0.0f;
    out->m[3][3] = 1.0f;
}

/**
 * @brief Multiplies two homogeneous transforms.
 * @param a Left-hand transform.
 * @param b Right-hand transform.
 * @param out Product transform; no output is written for invalid pointers.
 * @return None.
 */
void Operations_Multiply(
    const Kinematics_Transform_t *a,
    const Kinematics_Transform_t *b,
    Kinematics_Transform_t *out
)
{
    if ((a == NULL) || (b == NULL) || (out == NULL))
    {
        return;
    }

    /*
     * Production path:
     * Always use the benchmark-selected homogeneous transform multiplication.
     */
    Operations_MultiplyHomogeneousInternal(a->m, b->m, out->m);
}

/**
 * @brief Builds a local link transform using standard trigonometric functions.
 * @param link Fixed translation and orientation of the link.
 * @param joint_angle_rad Joint rotation in radians.
 * @param out Calculated local homogeneous transform.
 * @return None.
 */
void Operations_LinkTransform(
    const Operations_LinkPose_t *link,
    float joint_angle_rad,
    Kinematics_Transform_t *out
)
{
    /*
     * Standard precision path:
     * Uses sinf/cosf.
     */
    Operations_LinkTransformInternal(link, joint_angle_rad, out, 0U);
}

/**
 * @brief Builds a local link transform using ARM FastMath when available.
 * @param link Fixed translation and orientation of the link.
 * @param joint_angle_rad Joint rotation in radians.
 * @param out Calculated local homogeneous transform.
 * @return None.
 */
void Operations_LinkTransformFastMath(
    const Operations_LinkPose_t *link,
    float joint_angle_rad,
    Kinematics_Transform_t *out
)
{
    /*
     * Fast compute path:
     * Uses ARM FastMath if OPERATIONS_USE_CMSIS_DSP is enabled.
     * Otherwise falls back to sinf/cosf.
     */
    Operations_LinkTransformInternal(link, joint_angle_rad, out, 1U);
}

/**
 * @brief Rounds a floating-point value to the nearest signed 32-bit integer.
 * @param value Value to round.
 * @return Rounded integer value.
 */
int32_t Operations_RoundToI32(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}

/**
 * @brief Calculates the inverse of a nonsingular 3x3 matrix.
 * @param in Matrix to invert.
 * @param out Calculated inverse when the operation succeeds.
 * @return 1 on success, or 0 for NULL pointers or a near-singular matrix.
 */
uint8_t Operations_Invert3x3(const float in[3][3], float out[3][3])
{
    float det;

    if ((in == NULL) || (out == NULL))
    {
        return 0U;
    }

    det =
        (in[0][0] * ((in[1][1] * in[2][2]) - (in[1][2] * in[2][1]))) -
        (in[0][1] * ((in[1][0] * in[2][2]) - (in[1][2] * in[2][0]))) +
        (in[0][2] * ((in[1][0] * in[2][1]) - (in[1][1] * in[2][0])));

    if ((det > -OPERATIONS_DETERMINANT_EPSILON) &&
        (det < OPERATIONS_DETERMINANT_EPSILON))
    {
        return 0U;
    }

    out[0][0] = ((in[1][1] * in[2][2]) - (in[1][2] * in[2][1])) / det;
    out[0][1] = ((in[0][2] * in[2][1]) - (in[0][1] * in[2][2])) / det;
    out[0][2] = ((in[0][1] * in[1][2]) - (in[0][2] * in[1][1])) / det;

    out[1][0] = ((in[1][2] * in[2][0]) - (in[1][0] * in[2][2])) / det;
    out[1][1] = ((in[0][0] * in[2][2]) - (in[0][2] * in[2][0])) / det;
    out[1][2] = ((in[0][2] * in[1][0]) - (in[0][0] * in[1][2])) / det;

    out[2][0] = ((in[1][0] * in[2][1]) - (in[1][1] * in[2][0])) / det;
    out[2][1] = ((in[0][1] * in[2][0]) - (in[0][0] * in[2][1])) / det;
    out[2][2] = ((in[0][0] * in[1][1]) - (in[0][1] * in[1][0])) / det;

    return 1U;
}

/* -------------------------------------------------------------------------- */
/* Private functions                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Implements local link-transform construction for both trigonometric backends.
 * @param link Fixed translation and orientation of the link.
 * @param joint_angle_rad Joint rotation in radians.
 * @param out Calculated local homogeneous transform.
 * @param use_fast_math Nonzero to request the FastMath backend.
 * @return None.
 */
static void Operations_LinkTransformInternal(
    const Operations_LinkPose_t *link,
    float joint_angle_rad,
    Kinematics_Transform_t *out,
    uint8_t use_fast_math
)
{
    float cr;
    float sr;
    float cp;
    float sp;
    float cy;
    float sy;
    float cq;
    float sq;

    float r00;
    float r01;
    float r02;
    float r10;
    float r11;
    float r12;
    float r20;
    float r21;
    float r22;

    if ((link == NULL) || (out == NULL))
    {
        return;
    }

    cr = Operations_Cos(link->roll, use_fast_math);
    sr = Operations_Sin(link->roll, use_fast_math);

    cp = Operations_Cos(link->pitch, use_fast_math);
    sp = Operations_Sin(link->pitch, use_fast_math);

    cy = Operations_Cos(link->yaw, use_fast_math);
    sy = Operations_Sin(link->yaw, use_fast_math);

    cq = Operations_Cos(joint_angle_rad, use_fast_math);
    sq = Operations_Sin(joint_angle_rad, use_fast_math);

    /*
     * Fixed link orientation:
     *
     * R_fixed = Rz(yaw) * Ry(pitch) * Rx(roll)
     */
    r00 = cy * cp;
    r01 = (cy * sp * sr) - (sy * cr);
    r02 = (cy * sp * cr) + (sy * sr);

    r10 = sy * cp;
    r11 = (sy * sp * sr) + (cy * cr);
    r12 = (sy * sp * cr) - (cy * sr);

    r20 = -sp;
    r21 = cp * sr;
    r22 = cp * cr;

    /*
     * Full local transform:
     *
     * T = Trans(x,y,z) * Rz(yaw) * Ry(pitch) * Rx(roll) * Rz(q)
     */
    out->m[0][0] = (r00 * cq) + (r01 * sq);
    out->m[0][1] = (-r00 * sq) + (r01 * cq);
    out->m[0][2] = r02;
    out->m[0][3] = link->x;

    out->m[1][0] = (r10 * cq) + (r11 * sq);
    out->m[1][1] = (-r10 * sq) + (r11 * cq);
    out->m[1][2] = r12;
    out->m[1][3] = link->y;

    out->m[2][0] = (r20 * cq) + (r21 * sq);
    out->m[2][1] = (-r20 * sq) + (r21 * cq);
    out->m[2][2] = r22;
    out->m[2][3] = link->z;

    out->m[3][0] = 0.0f;
    out->m[3][1] = 0.0f;
    out->m[3][2] = 0.0f;
    out->m[3][3] = 1.0f;
}

/**
 * @brief Evaluates sine with the selected standard or FastMath backend.
 * @param x Angle in radians.
 * @param use_fast_math Nonzero to request the FastMath backend.
 * @return Sine of x.
 */
static float Operations_Sin(float x, uint8_t use_fast_math)
{
#if defined(OPERATIONS_USE_CMSIS_DSP)
    if (use_fast_math != 0U)
    {
        return arm_sin_f32(x);
    }
#else
    (void)use_fast_math;
#endif

    return sinf(x);
}

/**
 * @brief Evaluates cosine with the selected standard or FastMath backend.
 * @param x Angle in radians.
 * @param use_fast_math Nonzero to request the FastMath backend.
 * @return Cosine of x.
 */
static float Operations_Cos(float x, uint8_t use_fast_math)
{
#if defined(OPERATIONS_USE_CMSIS_DSP)
    if (use_fast_math != 0U)
    {
        return arm_cos_f32(x);
    }
#else
    (void)use_fast_math;
#endif

    return cosf(x);
}

/**
 * @brief Multiplies homogeneous matrix data with an alias-safe temporary buffer.
 * @param A Left-hand 4x4 homogeneous matrix.
 * @param B Right-hand 4x4 homogeneous matrix.
 * @param C Product matrix.
 * @return None.
 */
static void Operations_MultiplyHomogeneousInternal(
    const float A[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    const float B[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    float C[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE]
)
{
    float tmp[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE];

    /*
     * Homogeneous transform multiplication:
     *
     * A = [ Ra  pa ]
     *     [  0   1 ]
     *
     * B = [ Rb  pb ]
     *     [  0   1 ]
     *
     * C = [ Ra*Rb   Ra*pb + pa ]
     *     [   0          1     ]
     */

    for (uint8_t row = 0U; row < 3U; row++)
    {
        const float a0 = A[row][0];
        const float a1 = A[row][1];
        const float a2 = A[row][2];

        tmp[row][0] = (a0 * B[0][0]) + (a1 * B[1][0]) + (a2 * B[2][0]);
        tmp[row][1] = (a0 * B[0][1]) + (a1 * B[1][1]) + (a2 * B[2][1]);
        tmp[row][2] = (a0 * B[0][2]) + (a1 * B[1][2]) + (a2 * B[2][2]);

        tmp[row][3] = (a0 * B[0][3]) + (a1 * B[1][3]) + (a2 * B[2][3]) + A[row][3];
    }

    tmp[3][0] = 0.0f;
    tmp[3][1] = 0.0f;
    tmp[3][2] = 0.0f;
    tmp[3][3] = 1.0f;

    Operations_Mat4Copy(tmp, C);
}

/**
 * @brief Copies all elements of one 4x4 matrix to another.
 * @param src Source matrix.
 * @param dst Destination matrix.
 * @return None.
 */
static void Operations_Mat4Copy(
    const float src[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    float dst[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE]
)
{
    for (uint8_t row = 0U; row < KINEMATICS_MATRIX_SIZE; row++)
    {
        for (uint8_t col = 0U; col < KINEMATICS_MATRIX_SIZE; col++)
        {
            dst[row][col] = src[row][col];
        }
    }
}
