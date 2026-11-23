#include "spmsm.h"
#include <string.h>
#include <math.h>

spmsm_id_t SPMSM_ID;

void SPMSM_ID_Init(spmsm_id_t *id, const spmsm_id_cfg_t *cfg)
{
    if (id == NULL || cfg == NULL) return;
    memset(id, 0, sizeof(spmsm_id_t));

    id->cfg   = *cfg;
    id->state = SPMSM_ID_IDLE;
}

void SPMSM_ElecID_Start(spmsm_id_t *id, float fs_hz)
{
    if (id == NULL || id->cfg.inj_freq_hz <= 0.0f || fs_hz <= 0.0f) return;

    id->phase_rad = 0.0f;
    id->tick_cnt  = 0;

    /* 取整周期对应的采样点数，保证三角函数系在 [0, T] 上严格正交 */
    float ticks_per_cycle = fs_hz / id->cfg.inj_freq_hz;
    id->settle_ticks = (uint32_t)(ticks_per_cycle * (float)id->cfg.settle_cycles + 0.5f);
    id->sample_ticks = (uint32_t)(ticks_per_cycle * (float)id->cfg.sample_cycles + 0.5f);

    id->sum_id_cos = 0.0;
    id->sum_id_sin = 0.0;

    id->param.elec_valid = false;
    id->state = SPMSM_ID_ELEC_SETTLE;
}

/**
 * @brief  24kHz 中断中调用的电参数辨识单步函数
 * @note   调用前需确保转子已静止在已知电角度（零位已校准）
 */
spmsm_id_state_t SPMSM_ElecID_Step(spmsm_id_t *id, float i_d, float i_q, float dt,
                                   float *ud_out, float *uq_out)
{
    if (id == NULL || ud_out == NULL || uq_out == NULL) {
        return SPMSM_ID_ERROR;
    }

    *ud_out = 0.0f;
    *uq_out = 0.0f;

    if (id->state != SPMSM_ID_ELEC_SETTLE && id->state != SPMSM_ID_ELEC_SAMPLE) {
        return id->state;
    }

    /* 过流保护 */
    if (fabsf(i_d) > id->cfg.i_limit_a || fabsf(i_q) > id->cfg.i_limit_a) {
        id->state = SPMSM_ID_ERROR;
        return id->state;
    }

    /* 计算当前正弦激励与正交基函数值 */
    Angle_t sc = FOC_Angle_Calc(id->phase_rad);
    float ud_cmd = id->cfg.inj_amp_v * sc.sin_a;

    *ud_out = ud_cmd;
    *uq_out = 0.0f;

    /* 相位递增并归一化 */
    id->phase_rad += FOC_2PI * id->cfg.inj_freq_hz * dt;
    if (id->phase_rad >= FOC_2PI) {
        id->phase_rad -= FOC_2PI;
    }

    if (id->state == SPMSM_ID_ELEC_SETTLE) {
        id->tick_cnt++;
        if (id->tick_cnt >= id->settle_ticks) {
            id->tick_cnt = 0;
            id->state = SPMSM_ID_ELEC_SAMPLE;
        }
    } else if (id->state == SPMSM_ID_ELEC_SAMPLE) {
        /* 仅累加基波正交投影项：天然滤除直流偏置与死区高次谐波 */
        id->sum_id_cos += (double)i_d * (double)sc.cos_a;
        id->sum_id_sin += (double)i_d * (double)sc.sin_a;

        id->tick_cnt++;
        if (id->tick_cnt >= id->sample_ticks) {
            double n_inv = 2.0 / (double)id->sample_ticks;
            /*
             * ud = U * sin(wt)
             * id = I * sin(wt - phi) = I*cos(phi)*sin(wt) - I*sin(phi)*cos(wt)
             * 因此：id_ex = -I*sin(phi), id_ey = I*cos(phi)
             */
            double id_ex = id->sum_id_cos * n_inv;
            double id_ey = id->sum_id_sin * n_inv;
            double i_amp = sqrt(id_ex * id_ex + id_ey * id_ey);

            if (i_amp > 1e-5) {
                /* 1. 基波阻抗幅值 Am = U / I */
                double Am = (double)id->cfg.inj_amp_v / i_amp;

                /* 2. 基波相位差 phi_raw = atan2(I*sin(phi), I*cos(phi)) */
                double phi_raw = atan2(-id_ex, id_ey);

                /* 3. 扣除数字控制 1.5 拍 PWM 滞后: delta_phi = 3 * PI * T_pwm * fn */
                double delta_phi_pwm = 3.0 * (double)FOC_PI * (double)dt * (double)id->cfg.inj_freq_hz;
                double phi_m = phi_raw - delta_phi_pwm;
                if (phi_m < 0.0) {
                    phi_m = 0.0;
                } else if (phi_m > (double)FOC_PI * 0.5) {
                    phi_m = (double)FOC_PI * 0.5;
                }

                /* 4. 计算 Rs 与 Ls */
                double omega = (double)FOC_2PI * (double)id->cfg.inj_freq_hz;
                id->param.Rs = (float)(Am * cos(phi_m));
                id->param.Ls = (float)((Am / omega) * sin(phi_m));
                id->param.elec_phase_deg = (float)(phi_m * (double)RAD_TO_DEG);
                id->param.elec_valid = true;

                *ud_out = 0.0f;
                *uq_out = 0.0f;
                id->state = SPMSM_ID_ELEC_DONE;
            } else {
                id->state = SPMSM_ID_ERROR;
            }
        }
    }

    return id->state;
}

/**
 * @brief  零极点对消法，使用辨识结果整定 dq 轴电流环 PID
 */
bool SPMSM_Apply_CurrentLoop_PI(spmsm_id_t *id, float v_bus)
{
    if (id == NULL || !id->param.elec_valid || v_bus < 1.0f) {
        return false;
    }

    float wc = FOC_2PI * id->cfg.curr_loop_bw_hz;
    id->param.kp_curr = wc * id->param.Ls;
    id->param.ki_curr = wc * id->param.Rs;

    float u_max = v_bus * FOC_1_SQRT3; /* SVPWM 内切圆最大相电压幅值 */
    float pid_param[3] = { id->param.kp_curr, id->param.ki_curr, 0.0f };

    PID_Init(&id->pid_id, pid_param, u_max, u_max, 1.0f);
    PID_Init(&id->pid_iq, pid_param, u_max, u_max, 1.0f);

    return true;
}

/* ==================== 机械参数辨识预留接口 ==================== */

bool SPMSM_MechID_Start(spmsm_id_t *id, float v_bus)
{
    if (id == NULL || !id->param.elec_valid) {
        return false;
    }
    if (!SPMSM_Apply_CurrentLoop_PI(id, v_bus)) {
        return false;
    }

    memset(&id->mech_ctx, 0, sizeof(spmsm_mech_ctx_t));
    id->param.mech_valid = false;
    id->state = SPMSM_ID_MECH_ACCEL;
    return true;
}

spmsm_id_state_t SPMSM_MechID_Step(spmsm_id_t *id, float i_d, float i_q,
                                   float mech_speed, float total_mech_angle,
                                   float elec_speed, float dt,
                                   float *ud_out, float *uq_out, bool *pwm_enable)
{
    /* TODO: 后续实现加速(t1~t2) -> 匀速(t3~t4, 辨识psi_f) -> 关管自由减速(t5~t6) -> 3x3伴随矩阵求解 J/Bm/Cm */
    (void)id; (void)i_d; (void)i_q;
    (void)mech_speed; (void)total_mech_angle; (void)elec_speed; (void)dt;
    if (ud_out)     *ud_out = 0.0f;
    if (uq_out)     *uq_out = 0.0f;
    if (pwm_enable) *pwm_enable = true;
    return SPMSM_ID_IDLE;
}
