#include "foc_math.h"


/**
 * @brief  将电角度归一化到 [0, 2*PI) 区间
 */
float FOC_Normalize_Angle(float angle)
{
    float a = fmodf(angle, FOC_2PI);
    return (a >= 0.0f) ? a : (a + FOC_2PI);
}

/**
 * @brief  等幅值 Clarke 变换 (abc -> alpha/beta)
 *         
 */
void FOC_Clarke(const abc_t *abc, alphabeta_t *ab)
{
    ab->alpha = abc->a;
    ab->beta  = (abc->a + 2.0f * abc->b) * FOC_1_SQRT3;
}

/**
 * @brief  Park 变换 (alpha/beta -> d/q)
 *         theta为弧度制
 */
void FOC_Park(const alphabeta_t *ab, dq_t *dq, Angle_t *theta)
{
    dq->d =  ab->alpha * theta->cos_a + ab->beta * theta->sin_a;
    dq->q = -ab->alpha * theta->sin_a + ab->beta * theta->cos_a;
}

/**
 * @brief  逆 Park 变换 (d/q -> alpha/beta) 
 *         theta为弧度制
 */
void FOC_Inv_Park(const dq_t *dq, alphabeta_t *ab ,Angle_t *theta)
{ 	
    ab->alpha = dq->d * theta->cos_a - dq->q * theta->sin_a;
    ab->beta  = dq->d * theta->sin_a + dq->q * theta->cos_a;
}
