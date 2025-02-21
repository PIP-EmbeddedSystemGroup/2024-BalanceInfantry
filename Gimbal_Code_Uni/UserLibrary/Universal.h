#ifndef _UNIVERSAL_H_
#define _UNIVERSAL_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"
#include "arm_math.h"

#define RAD2DEG(rad) ((rad) / (2 * PI) * 360.f)
#define DEG2RAD(deg) ((deg) / 360.f * (2 * PI))

typedef arm_matrix_instance_f32 matrix_t;
#define MAT_ELEM(m, r, c) ((m).pData[(r - 1) * (m).numCols + (c - 1)])
#define MAT_INIT(m, r, c) do{arm_mat_init_f32(&m, r, c, pvPortMalloc(r * c * sizeof(float32_t))); memset(m.pData, 0, r * c * sizeof(float32_t));}while(0)
#define MAT_DEINIT(m) vPortFree((m).pData)
#define MAT_SET_SIZE(m, r, c) do{(m).numRows = r; (m).numCols = c;}while(0)
#define MAT_CPY(src, dst) memcpy((dst).pData, (src).pData, (src).numRows * (src).numCols * sizeof(float32_t))
#define MAT_ADD(m1, m2, m) arm_mat_add_f32(&m1, &m2, m)
#define MAT_SUB(m1, m2, m) arm_mat_sub_f32(&m1, &m2, m)
#define MAT_SCL(m1, val, m) arm_mat_scale_f32(&m1, val, m)
#define MAT_MPY(m1, m2, m) arm_mat_mult_f32(&m1, &m2, m)
#define MAT_TRS(m1, m) arm_mat_trans_f32(&m1, m)
#define MAT_INV(m1, m) arm_mat_inverse_f32(&m1, m)

int Constrain(int val, int max, int min);
float ConstrainF(float val, float max, float min);
int AngleWrap(int val, int max);
float AngleWrapF(float val, float max);
int AngleDiff(int a, int b, int max);
float AngleDiffF(float a, float b, float max);
void HanningWindow_Init(float* out, int n);

#endif
