/**
 * ESP-Drone Firmware
 * Utilities to simplify unit testing
 *
 */

#pragma once

// Include "xtensa_math.h". This header generates some warnings, especially in
// unit tests. We hide them to avoid noise.
//TODO: NEED ESP32 SUPPORT
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "xtensa_math.h"
#pragma GCC diagnostic pop

#include "cfassert.h"
#include "debug_assert.h"


#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)

#define MIN(a, b) ((b) < (a) ? (b) : (a))
#define MAX(a, b) ((b) > (a) ? (b) : (a))

// Matrix data must be aligned on 4 byte bundaries
static inline void assert_aligned_4_bytes(const xtensa_matrix_instance_f32 *matrix)
{
    const uint32_t address = (uint32_t)matrix->pData;
    ASSERT((address & 0x3) == 0);
}

static inline void mat_trans(const xtensa_matrix_instance_f32 *pSrc, xtensa_matrix_instance_f32 *pDst)
{
    assert_aligned_4_bytes(pSrc);
    assert_aligned_4_bytes(pDst);

    ASSERT(XTENSA_MATH_SUCCESS == xtensa_mat_trans_f32(pSrc, pDst));
}

static inline void mat_inv(const xtensa_matrix_instance_f32 *pSrc, xtensa_matrix_instance_f32 *pDst)
{
    assert_aligned_4_bytes(pSrc);
    assert_aligned_4_bytes(pDst);

    ASSERT(XTENSA_MATH_SUCCESS == xtensa_mat_inverse_f32(pSrc, pDst));
}

static inline void mat_mult(const xtensa_matrix_instance_f32 *pSrcA, const xtensa_matrix_instance_f32 *pSrcB, xtensa_matrix_instance_f32 *pDst)
{
    assert_aligned_4_bytes(pSrcA);
    assert_aligned_4_bytes(pSrcB);
    assert_aligned_4_bytes(pDst);

    ASSERT(XTENSA_MATH_SUCCESS == xtensa_mat_mult_f32(pSrcA, pSrcB, pDst));
}

static inline float xtensa_sqrt(float32_t in)
{
    float pOut = 0;
    xtensa_status result = xtensa_sqrt_f32(in, &pOut);
    ASSERT(XTENSA_MATH_SUCCESS == result);
    return pOut;
}

static inline float limPos(float in)
{
    if (in < 0.0f) {
        return 0.0f;
    }

    return in;
}

static inline float clip1(float a)
{
    if (a < -1.0f) {
        return -1.0f;
    }

    if (a > 1.0f) {
        return 1.0f;
    }

    return a;
}
