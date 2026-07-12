#include "operations.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Public functions                                                           */
/* -------------------------------------------------------------------------- */

void Operations_SetIdentity(Kinematics_Transform_t *out)
{
    if (out == NULL)
    {
        return;
    }

    for (uint8_t i = 0U; i < KINEMATICS_MATRIX_SIZE; i++)
    {
        for (uint8_t j = 0U; j < KINEMATICS_MATRIX_SIZE; j++)
        {
            out->m[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void Operations_Multiply(const Kinematics_Transform_t *a, const Kinematics_Transform_t *b, Kinematics_Transform_t *out)
{
    Kinematics_Transform_t tmp;

    if ((a == NULL) || (b == NULL) || (out == NULL))
    {
        return;
    }

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

void Operations_LinkTransform(const Operations_LinkPose_t *link, float joint_angle_rad, Kinematics_Transform_t *out)
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

    cr = cosf(link->roll);
    sr = sinf(link->roll);
    cp = cosf(link->pitch);
    sp = sinf(link->pitch);
    cy = cosf(link->yaw);
    sy = sinf(link->yaw);
    cq = cosf(joint_angle_rad);
    sq = sinf(joint_angle_rad);

    /* Rotation = Rz(yaw) * Ry(pitch) * Rx(roll) * Rz(q). */
    r00 = cy * cp;
    r01 = cy * sp * sr - sy * cr;
    r02 = cy * sp * cr + sy * sr;

    r10 = sy * cp;
    r11 = sy * sp * sr + cy * cr;
    r12 = sy * sp * cr - cy * sr;

    r20 = -sp;
    r21 = cp * sr;
    r22 = cp * cr;

    Operations_SetIdentity(out);

    out->m[0][0] = r00 * cq + r01 * sq;
    out->m[0][1] = -r00 * sq + r01 * cq;
    out->m[0][2] = r02;
    out->m[0][3] = link->x;

    out->m[1][0] = r10 * cq + r11 * sq;
    out->m[1][1] = -r10 * sq + r11 * cq;
    out->m[1][2] = r12;
    out->m[1][3] = link->y;

    out->m[2][0] = r20 * cq + r21 * sq;
    out->m[2][1] = -r20 * sq + r21 * cq;
    out->m[2][2] = r22;
    out->m[2][3] = link->z;
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
