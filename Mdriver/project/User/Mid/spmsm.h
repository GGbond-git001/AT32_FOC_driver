#ifndef __SPMSM_H
#define __SPMSM_H

#include <stdint.h>
#include <stdbool.h>
#include "foc_math.h"
#include "pid.h"

/* 参数辨识状态机 */
typedef enum {
    SPMSM_ID_IDLE = 0,
    /* 电参数辨识阶段 */
    SPMSM_ID_ELEC_SETTLE,   /* 等待正弦激励进入稳态 (>5*tau) */
    SPMSM_ID_ELEC_SAMPLE,   /* 整周期内积投影采样 */
    SPMSM_ID_ELEC_DONE,     /* 电参数辨识完成 */
    /* 机械参数辨识阶段*/
    SPMSM_ID_MECH_ACCEL,    /* 加速阶段 (t1 ~ t2) */
    SPMSM_ID_MECH_CRUISE,   /* 匀速阶段 (t3 ~ t4) + 磁链辨识 */
    SPMSM_ID_MECH_DECEL,    /* 关管自由减速阶段 (t5 ~ t6) */
    SPMSM_ID_MECH_DONE,     /* 全参数辨识完成 */
    SPMSM_ID_ERROR          /* 过流或计算异常保护 */
} spmsm_id_state_t;

/* 辨识激励与保护配置 */
typedef struct {
    /* 电参数辨识配置 */
    float    inj_amp_v;       /* d轴注入正弦电压幅值 U (V) */
    float    inj_freq_hz;     /* d轴注入正弦电压频率 fn (Hz) */
    uint16_t settle_cycles;   /* 稳态等待周期数（建议 10~30） */
    uint16_t sample_cycles;   /* 采样积分整周期数（建议 20~50） */
    float    i_limit_a;       /* 辨识过程过流保护阈值 (A) */

    /* 电流环与机械参数辨识配置 */
    float    curr_loop_bw_hz; /* 电流环整定带宽 fc (Hz)，如 500.0f / 1000.0f */
    float    mech_iq_ref_a;   /* 机械参数辨识恒定 q 轴激励电流 IQ (A) */
} spmsm_id_cfg_t;

/* 辨识结果结构体（存放最终校准参数） */
typedef struct {
    /* 电参数 */
    float Rs;                 /* 定子相电阻 (Ohm) */
    float Ls;                 /* 定子相电感 (H) */
    float psi_f;              /* 永磁体磁链 (Wb) */

    /* 机械参数 */
    float J;                  /* 转动惯量 (kg·m^2) */
    float Bm;                 /* 粘滞阻尼系数 (N·m·s/rad) */
    float Cm;                 /* 库伦摩擦系数 (N·m) */

    /* 辅助诊断与自动整定结果 */
    float elec_phase_deg;     /* 补偿后的阻抗相角 (deg)，理想区间 45°~60° */
    float kp_curr;            /* 零极点对消整定出的电流环 Kp */
    float ki_curr;            /* 零极点对消整定出的电流环 Ki */

    bool  elec_valid;         /* Rs/Ls 有效标志 */
    bool  mech_valid;         /* psi_f/J/Bm/Cm 有效标志 */
} spmsm_param_t;

/* 机械参数辨识中间积分变量*/
typedef struct {
    float te_int[3];          /* 加速/匀速/减速三阶段电磁转矩积分 */
    float d_omega[3];         /* 三阶段机械角速度变化量 */
    float d_theta[3];         /* 三阶段机械转角变化量（需多圈累计角度） */
    float d_time[3];          /* 三阶段持续时间 */
    float flux_num_int;       /* 匀速阶段磁链分子积分: int(uq - Rs*iq)dt */
    float flux_den_int;       /* 匀速阶段磁链分母积分: int(omega_e)dt */
    float stage_timer;        /* 当前阶段计时器 (s) */
} spmsm_mech_ctx_t;

/* SPMSM 参数辨识主句柄 */
typedef struct {
    spmsm_id_state_t state;
    spmsm_id_cfg_t   cfg;
    spmsm_param_t    param;

    /* 电参数辨识内部累加器 */
    float    phase_rad;       /* 注入正弦信号实时相位 [0, 2*PI) */
    uint32_t tick_cnt;        /* 当前阶段采样点数计数 */
    uint32_t settle_ticks;    /* 稳态等待总点数 */
    uint32_t sample_ticks;    /* 整周期采样总点数 */

    double   sum_id_cos;      /* 累加 sum(id * cos(wt)) */
    double   sum_id_sin;      /* 累加 sum(id * sin(wt)) */

    /* 机械参数辨识上下文与电流环 PID */
    spmsm_mech_ctx_t mech_ctx;
    PID_TypeDef      pid_id;
    PID_TypeDef      pid_iq;
} spmsm_id_t;

extern spmsm_id_t SPMSM_ID;

/* 初始化与电参数辨识接口 */
void SPMSM_ID_Init(spmsm_id_t *id, const spmsm_id_cfg_t *cfg);
void SPMSM_ElecID_Start(spmsm_id_t *id, float fs_hz);
spmsm_id_state_t SPMSM_ElecID_Step(spmsm_id_t *id, float i_d, float i_q, float dt,
                                   float *ud_out, float *uq_out);

/* 根据辨识出的 Rs/Ls 自动整定 dq 轴电流环 PID */
bool SPMSM_Apply_CurrentLoop_PI(spmsm_id_t *id, float v_bus);

/* 机械参数辨识 */
bool SPMSM_MechID_Start(spmsm_id_t *id, float v_bus);
spmsm_id_state_t SPMSM_MechID_Step(spmsm_id_t *id, float i_d, float i_q,
                                   float mech_speed, float total_mech_angle,
                                   float elec_speed, float dt,
                                   float *ud_out, float *uq_out, bool *pwm_enable);

#endif
