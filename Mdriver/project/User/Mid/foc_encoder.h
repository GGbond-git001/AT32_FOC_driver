#ifndef __FOC_ENCODER_H
#define __FOC_ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "foc_math.h"

#define MT6816_CPR          16384.0f
#define MT6816_RAW_TO_RAD   (FOC_2PI / MT6816_CPR)

typedef struct {
    uint8_t  pole_pairs;        /* 电机极对数 */
    int8_t   dir;               /* 编码器安装方向: +1 正向, -1 反向 */
    uint16_t raw_cnt;           /* 14位绝对角度计数值 (0 ~ 16383) */
    uint16_t raw_prev;          /* 上一拍计数值 */
    bool     no_mag_warn;       /* 弱磁报警标志 (true表示磁铁过远或缺失) */
    bool     parity_ok;         /* 偶校验通过标志 */
    uint32_t err_cnt;           /* 通信校验错误累计次数 */

    float    mech_angle;        /* 机械单圈角度 [0, 2*PI) (rad) */
    float    total_mech_angle;  /* 机械多圈角度(rad) */
    float    elec_angle_offset; /* 电角度零位偏置 (rad) */
    float    elec_angle;        /* 校准后的实时电角度 [0, 2*PI) (rad) */
    float    mech_speed;        /* 滤波后的机械角速度 (rad/s) */
    float    elec_speed;        /* 滤波后的电角速度 (rad/s) */
} foc_encoder_t;

extern foc_encoder_t FOC_Encoder;

void FOC_Encoder_Init(uint8_t pole_pairs, int8_t dir);
bool FOC_Encoder_Update(uint16_t raw_frame16, float dt);
bool FOC_Encoder_Read_Avg_Raw(uint16_t *avg_raw_out);
bool FOC_Encoder_Set_Zero_From_Two_Raw(uint16_t raw_cw, uint16_t raw_ccw);
bool FOC_Encoder_Set_Zero_Offset(void);
bool FOC_Encoder_Auto_Calibrate(float v_bus, float align_v);

#endif
