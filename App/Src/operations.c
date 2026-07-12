#include "operations.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#if defined(OPERATIONS_USE_CMSIS_DSP)
#include "arm_math.h"
#endif

/* -------------------------------------------------------------------------- */
/* Private defines                                                            */
/* -------------------------------------------------------------------------- */

#define OP_LOOKUP_TABLE_SIZE       256U
#define OP_LOOKUP_TABLE_LAST_INDEX OP_LOOKUP_TABLE_SIZE
#define OP_LOOKUP_INDEX_SCALE      ((float)OP_LOOKUP_TABLE_SIZE / OPERATIONS_TWO_PI)

/* -------------------------------------------------------------------------- */
/* Private variables                                                          */
/* -------------------------------------------------------------------------- */

static float op_sin_table[OP_LOOKUP_TABLE_SIZE + 1U];
static uint8_t op_lookup_is_initialized = 0U;

/* -------------------------------------------------------------------------- */
/* Private function prototypes                                                */
/* -------------------------------------------------------------------------- */

static void op_lookup_init(void);
static float op_wrap_0_to_2pi(float x);
static void op_mat4_copy(
    const float src[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    float dst[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE]
);

/* -------------------------------------------------------------------------- */
/* Existing project-compatible public functions                               */
/* -------------------------------------------------------------------------- */

void Operations_SetIdentity(Kinematics_Transform_t *out)
{
    if (out == NULL)
    {
        return;
    }

    op_mat4_identity(out->m);
}

void Operations_Multiply(const Kinematics_Transform_t *a, const Kinematics_Transform_t *b, Kinematics_Transform_t *out)
{
    if ((a == NULL) || (b == NULL) || (out == NULL))
    {
        return;
    }

    /* Keep the old project behavior: plain row-by-column multiplication. */
    op_mat4_mul_baseline(a->m, b->m, out->m);
}

void Operations_LinkTransform(const Operations_LinkPose_t *link, float joint_angle_rad, Kinematics_Transform_t *out)
{
    /* Keep the old project behavior: normal sinf/cosf precision. */
    Operations_LinkTransformMode(link, joint_angle_rad, out, OP_TRIG_STANDARD);
}

int32_t Operations_RoundToI32(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}

uint8_t Operations_Invert3x3(const float in[3][3], float out[3][3])
{
    float det;

    if ((in == NULL) || (out == NULL))
    {
        return 0U;
    }

    det =
        in[0][0] * ((in[1][1] * in[2][2]) - (in[1][2] * in[2][1])) -
        in[0][1] * ((in[1][0] * in[2][2]) - (in[1][2] * in[2][0])) +
        in[0][2] * ((in[1][0] * in[2][1]) - (in[1][1] * in[2][0]));

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
/* Benchmark / configurable trigonometry                                      */
/* -------------------------------------------------------------------------- */

float op_sin_standard(float x)
{
    return sinf(x);
}

float op_cos_standard(float x)
{
    return cosf(x);
}

float op_sin_arm_fast(float x)
{
#if defined(OPERATIONS_USE_CMSIS_DSP)
    return arm_sin_f32(x);
#else
    /* Fallback keeps the project buildable when CMSIS-DSP is not linked. */
    return sinf(x);
#endif
}

float op_cos_arm_fast(float x)
{
#if defined(OPERATIONS_USE_CMSIS_DSP)
    return arm_cos_f32(x);
#else
    /* Fallback keeps the project buildable when CMSIS-DSP is not linked. */
    return cosf(x);
#endif
}

float op_sin_lookup(float x)
{
    float wrapped;
    float scaled;
    uint32_t index;
    float fraction;
    float a;
    float b;

    op_lookup_init();

    wrapped = op_wrap_0_to_2pi(x);
    scaled = wrapped * OP_LOOKUP_INDEX_SCALE;
    index = (uint32_t)scaled;

    if (index >= OP_LOOKUP_TABLE_SIZE)
    {
        index = 0U;
        fraction = 0.0f;
    }
    else
    {
        fraction = scaled - (float)index;
    }

    a = op_sin_table[index];
    b = op_sin_table[index + 1U];

    return a + ((b - a) * fraction);
}

float op_cos_lookup(float x)
{
    return op_sin_lookup(x + (0.5f * OPERATIONS_PI));
}

float op_sin(float x, op_trig_mode_t mode)
{
    switch (mode)
    {
        case OP_TRIG_ARM_FAST:
            return op_sin_arm_fast(x);

        case OP_TRIG_LOOKUP:
            return op_sin_lookup(x);

        case OP_TRIG_STANDARD:
        default:
            return op_sin_standard(x);
    }
}

float op_cos(float x, op_trig_mode_t mode)
{
    switch (mode)
    {
        case OP_TRIG_ARM_FAST:
            return op_cos_arm_fast(x);

        case OP_TRIG_LOOKUP:
            return op_cos_lookup(x);

        case OP_TRIG_STANDARD:
        default:
            return op_cos_standard(x);
    }
}

const char *op_trig_mode_name(op_trig_mode_t mode)
{
    switch (mode)
    {
        case OP_TRIG_ARM_FAST:
            return "arm_fast";

        case OP_TRIG_LOOKUP:
            return "lookup";

        case OP_TRIG_STANDARD:
        default:
            return "standard";
    }
}

const char *op_matmul_mode_name(op_matmul_mode_t mode)
{
    switch (mode)
    {
        case OP_MATMUL_OPTIMIZED:
            return "optimized";

        case OP_MATMUL_HOMOGENEOUS:
            return "homogeneous";

        case OP_MATMUL_BASELINE:
        default:
            return "baseline";
    }
}

/* -------------------------------------------------------------------------- */
/* Benchmark / configurable matrix math                                       */
/* -------------------------------------------------------------------------- */

void op_mat4_identity(float M[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE])
{
    if (M == NULL)
    {
        return;
    }

    for (uint8_t row = 0U; row < KINEMATICS_MATRIX_SIZE; row++)
    {
        for (uint8_t col = 0U; col < KINEMATICS_MATRIX_SIZE; col++)
        {
            M[row][col] = (row == col) ? 1.0f : 0.0f;
        }
    }
}

void op_mat4_mul_baseline(
    const float A[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    const float B[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    float C[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE])
{
    float tmp[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE];

    if ((A == NULL) || (B == NULL) || (C == NULL))
    {
        return;
    }

    for (uint8_t row = 0U; row < KINEMATICS_MATRIX_SIZE; row++)
    {
        for (uint8_t col = 0U; col < KINEMATICS_MATRIX_SIZE; col++)
        {
            tmp[row][col] = 0.0f;

            for (uint8_t k = 0U; k < KINEMATICS_MATRIX_SIZE; k++)
            {
                tmp[row][col] += A[row][k] * B[k][col];
            }
        }
    }

    op_mat4_copy(tmp, C);
}

void op_mat4_mul_optimized(
    const float A[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    const float B[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    float C[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE])
{
    float tmp[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE];

    if ((A == NULL) || (B == NULL) || (C == NULL))
    {
        return;
    }

    for (uint8_t row = 0U; row < KINEMATICS_MATRIX_SIZE; row++)
    {
        const float a0 = A[row][0];
        const float a1 = A[row][1];
        const float a2 = A[row][2];
        const float a3 = A[row][3];

        tmp[row][0] = (a0 * B[0][0]) + (a1 * B[1][0]) + (a2 * B[2][0]) + (a3 * B[3][0]);
        tmp[row][1] = (a0 * B[0][1]) + (a1 * B[1][1]) + (a2 * B[2][1]) + (a3 * B[3][1]);
        tmp[row][2] = (a0 * B[0][2]) + (a1 * B[1][2]) + (a2 * B[2][2]) + (a3 * B[3][2]);
        tmp[row][3] = (a0 * B[0][3]) + (a1 * B[1][3]) + (a2 * B[2][3]) + (a3 * B[3][3]);
    }

    op_mat4_copy(tmp, C);
}

void op_mat4_mul_homogeneous(
    const float A[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    const float B[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    float C[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE])
{
    float tmp[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE];

    if ((A == NULL) || (B == NULL) || (C == NULL))
    {
        return;
    }

    /*
     * Homogeneous transform structure:
     * A = [Ra pa], B = [Rb pb], C = [Ra*Rb Ra*pb + pa]
     *     [0  1 ]      [0  1 ]      [0     1        ]
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

    op_mat4_copy(tmp, C);
}

void op_mat4_mul(
    const float A[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    const float B[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    float C[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    op_matmul_mode_t mode)
{
    switch (mode)
    {
        case OP_MATMUL_OPTIMIZED:
            op_mat4_mul_optimized(A, B, C);
            break;

        case OP_MATMUL_HOMOGENEOUS:
            op_mat4_mul_homogeneous(A, B, C);
            break;

        case OP_MATMUL_BASELINE:
        default:
            op_mat4_mul_baseline(A, B, C);
            break;
    }
}

void Operations_LinkTransformMode(
    const Operations_LinkPose_t *link,
    float joint_angle_rad,
    Kinematics_Transform_t *out,
    op_trig_mode_t trig_mode)
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

    cr = op_cos(link->roll, trig_mode);
    sr = op_sin(link->roll, trig_mode);
    cp = op_cos(link->pitch, trig_mode);
    sp = op_sin(link->pitch, trig_mode);
    cy = op_cos(link->yaw, trig_mode);
    sy = op_sin(link->yaw, trig_mode);
    cq = op_cos(joint_angle_rad, trig_mode);
    sq = op_sin(joint_angle_rad, trig_mode);

    /* Rotation = Rz(yaw) * Ry(pitch) * Rx(roll) * Rz(q). */
    r00 = cy * cp;
    r01 = (cy * sp * sr) - (sy * cr);
    r02 = (cy * sp * cr) + (sy * sr);

    r10 = sy * cp;
    r11 = (sy * sp * sr) + (cy * cr);
    r12 = (sy * sp * cr) - (cy * sr);

    r20 = -sp;
    r21 = cp * sr;
    r22 = cp * cr;

    op_mat4_identity(out->m);

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
}

/* -------------------------------------------------------------------------- */
/* Private functions                                                          */
/* -------------------------------------------------------------------------- */

static void op_lookup_init(void)
{
    if (op_lookup_is_initialized != 0U)
    {
        return;
    }

    for (uint32_t i = 0U; i <= OP_LOOKUP_TABLE_SIZE; i++)
    {
        float angle = ((float)i * OPERATIONS_TWO_PI) / (float)OP_LOOKUP_TABLE_SIZE;
        op_sin_table[i] = sinf(angle);
    }

    /* Make sure the final sample closes the table exactly. */
    op_sin_table[OP_LOOKUP_TABLE_LAST_INDEX] = op_sin_table[0];
    op_lookup_is_initialized = 1U;
}

static float op_wrap_0_to_2pi(float x)
{
    float wrapped;

    wrapped = fmodf(x, OPERATIONS_TWO_PI);

    if (wrapped < 0.0f)
    {
        wrapped += OPERATIONS_TWO_PI;
    }

    return wrapped;
}

static void op_mat4_copy(
    const float src[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE],
    float dst[KINEMATICS_MATRIX_SIZE][KINEMATICS_MATRIX_SIZE])
{
    for (uint8_t row = 0U; row < KINEMATICS_MATRIX_SIZE; row++)
    {
        for (uint8_t col = 0U; col < KINEMATICS_MATRIX_SIZE; col++)
        {
            dst[row][col] = src[row][col];
        }
    }
}
