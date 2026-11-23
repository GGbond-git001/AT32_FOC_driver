#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#include "arm_math.h"
#include <stdint.h>
#include <math.h>
#include "foc_const.h" //常用数字引入

/* 常用数学常数 */
#define RAD_TO_DEG   57.29577951308232f  //180.0f / PI

/* 坐标系结构体定义 */
typedef struct {
    float a;
    float b;
    float c;
} abc_t;

typedef struct {
    float alpha;
    float beta;
} alphabeta_t;

typedef struct {
    float d;
    float q;
} dq_t;

typedef struct {
    float cos_a;
    float sin_a;
} Angle_t;

/**
 * @brief  计算角度的sin和cos
 *         theat为弧度制
 */
static inline Angle_t FOC_Angle_Calc(float theta_rad)
{
    Angle_t res;
    arm_sin_cos_f32(theta_rad * RAD_TO_DEG, &res.sin_a, &res.cos_a);
    return res;
}

float FOC_Normalize_Angle(float angle);
void FOC_Clarke(const abc_t *abc, alphabeta_t *ab);
void FOC_Park(const alphabeta_t *ab, dq_t *dq, Angle_t *theta);
void FOC_Inv_Park(const dq_t *dq, alphabeta_t *ab ,Angle_t *theta);

#endif 