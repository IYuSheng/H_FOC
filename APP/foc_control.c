#include "foc_control.h"

#if HFI_ENABLE
static inline void foc_current_control_hfi(float target_d, float target_q);
#endif
static inline void foc_current_control(float target_d, float target_q);
static inline void foc_speed_control(float target_speed);
static inline void foc_position_control(float target_position);

/**
 * @brief FOC 控制任务
 */
void vFOCControlTask(void *pvParameters)
{
  foc_ctrl.target_speed = 300.0f;     // 设置目标速度
  foc_ctrl.out_q = 0.3f;             // 开环测试时的 q 轴输出电压
  foc_ctrl.out_d = 0.0f;             // 开环测试时的 d 轴输出电压
  foc_ctrl.target_q = 0.25f;         // 设置目标 Q 轴电流
  foc_ctrl.target_d = 0.0f;          // 设置目标 D 轴电流
  foc_ctrl.target_position = 6.28f;  // 设置目标位置

  // 初始化电流环 PI 参数
  foc_current_pi_init(I_D_P_GAIN, I_Q_P_GAIN, I_I_GAIN, I_I_LIMIT);

  // 初始化速度环 PI 参数
  foc_speed_pi_init(SPEED_P_GAIN, SPEED_I_GAIN, SPEED_I_LIMIT);

  // 初始化位置环 PI 参数
  foc_position_pi_init(POSITION_P_GAIN, POSITION_I_GAIN, POSITION_I_LIMIT);

  #if FLUX_OBSERVER_ENABLE
    // 初始化磁链观测器
    flux_observer_init(&g_flux_obs);
  #endif

  #if SMO_OBSERVER_ENABLE
    // 初始化滑模观测器
    SMO_Pare_init();
  #endif

  #if HFI_ENABLE
    // 初始化 HFI 观测器和 PLL
    hfi_observer_init(&g_hfi_obs, HFI_INJECT_VOLTAGE);
    hfi_pll_init(&g_hfi_pll, HFI_PLL_KP, HFI_PLL_KI);
  #endif

  #if FOC_PARAMETER_IDENTIFICATION_ENABLE
    // 启动电机参数辨识
    foc_motor_param_ident_start(0.8f, 0.5f, 500.0f, 0.4f, 0.05f);
  #endif

  for (;;)
  {
    #if DEBUG_MODE
    debug_printf("%.4f,%.4f,%.4f,%.4f,%.4f", foc_ctrl.target_q, foc_ctrl.out_q, foc_ctrl.abc_dq.current_d, foc_ctrl.target_speed, g_flux_obs.omega_filt);
    // debug_printf("%.4f,%.4f", g_flux_obs.omega_filt, g_smo_obs.omega);
    // debug_printf("%.4f,%.4f,%.4f,%.4f", g_smo_obs.e_alpha, g_smo_obs.e_beta, alpha_beta.alpha_i, alpha_beta.beta_i);
    // debug_printf("%.4f,%.4f", g_speed, foc_ctrl.angle);
    // debug_printf("%.4f,%.4f", alpha_beta.alpha_i, alpha_beta.beta_i);

    // debug_printf("%.4f,%.4f,%.4f", foc_datai.ia, foc_datai.ib, foc_datai.ic);
    // debug_printf("%.4f,%.4f,%.4f", foc_datav.va, foc_datav.vb, foc_datav.vc);
    // debug_printf("%.4f, %.4f", foc_datai.ia, foc_datav.va);
    // debug_printf("%.4f", foc_datav.vbus);
    // debug_printf("%d,%d,%d", svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);

    // debug_printf("%.4f, %.4f", g_flux_obs.theta, hall_data.angle);
    // debug_printf("%.4f, %.4f", g_flux_obs.eta1, g_flux_obs.eta2);
    // debug_printf("%.4f,%.4f, %.4f, %.4f", g_flux_obs.theta, g_smo_obs.theta_comp, g_smo_obs.e_alpha, g_smo_obs.e_beta);
    // debug_printf("%.4f,%.4f,%.4f,%.4f", g_hfi_pll.theta_est + PI, g_flux_obs.theta, g_hfi_obs.i_env_alpha, g_hfi_obs.i_env_beta);
    #endif

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void foc_control(void)
{
    #if FOC_PARAMETER_IDENTIFICATION_ENABLE
        // 电机参数辨识
        if (foc_motor_param_ident_is_running())
        {
            foc_motor_parameter_ident_step();
        }
    #else
        #if HFI_ENABLE
            // HFI 电流闭环
            foc_current_control_hfi(foc_ctrl.target_d, foc_ctrl.target_q);
        #else
            // FOC 速度开环
            // foc_open_loop_control();

            // FOC 电流闭环
            foc_current_control(foc_ctrl.target_d, foc_ctrl.target_q);
        #endif
    #endif
}

void foc_control_out(void)
{
    foc_speed_control(foc_ctrl.target_speed);
    // foc_position_control(foc_ctrl.target_position);
}

/**
 * @brief FOC 速度开环控制
 */
void foc_open_loop_control(void)
{
    static float32_t angle_accum = 0.0f;  // 电角度积分累加器

    clark_transform(&foc_datai, &foc_datav, &alpha_beta);

    /************************** 1. 开环电角度计算 **************************/
    // 机械转速转换为电角速度：omega_e = 2*pi*(n_rpm/60)*极对数
    arm_scale_f32(&foc_ctrl.target_speed, SPEED_FACTOR, &foc_ctrl.speed, 1);

    // 角度积分更新：theta = theta + omega_e * Ts
    float32_t ctrl_period = PWM_PERIOD_S;
    angle_accum += foc_ctrl.speed * ctrl_period;
    angle_accum = angle_normalize(angle_accum);
    foc_ctrl.angle = angle_accum;

    // 获取 Hall 电角度和速度
    hall_update_PLL(&hall_data);

    // 更新磁链观测器
    flux_observer_update(&g_flux_obs,
                         alpha_beta.alpha, alpha_beta.beta,      // 电压（上次输出）
                         alpha_beta.alpha_i, alpha_beta.beta_i,  // 电流
                         PWM_PERIOD_S);

    /************************** 2. Park 变换 **************************/
    float sin_theta, cos_theta;
    float angle_deg = foc_ctrl.angle * RAD_TO_DEG;
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

    park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

    // foc_ctrl.out_d = foc_id_pid_calculate(foc_ctrl.target_d, foc_ctrl.abc_dq.current_d);
    // foc_ctrl.out_q = foc_iq_pid_calculate(foc_ctrl.target_q, foc_ctrl.abc_dq.current_q);

    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);

    /************************** 3. SVPWM 核心计算 **************************/
    // 扇区判断
    svpwm.sector = svpwm_sector_calc(&alpha_beta);

    // 计算基本矢量和零矢量作用时间
    svpwm_calc_times(&alpha_beta, &svpwm, foc_datav.vbus);
    svpwm_duty_calc(&svpwm);

    // 输出 PWM 到定时器
    bsp_pwm_set_duty(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}

/**
 * @brief 电流闭环控制
 */
static inline void foc_current_control(float target_d, float target_q)
{
    clark_transform(&foc_datai, &foc_datav, &alpha_beta);

    #if FLUX_OBSERVER_ENABLE    // 非线性磁链观测器
    flux_observer_update(&g_flux_obs,
                         alpha_beta.alpha, alpha_beta.beta,
                         alpha_beta.alpha_i, alpha_beta.beta_i,
                         PWM_PERIOD_S);
    foc_ctrl.angle = g_flux_obs.theta;
    foc_ctrl.omega = g_flux_obs.omega_filt;
    #elif SMO_OBSERVER_ENABLE   // SMO 观测器
    flux_observer_update(&g_flux_obs,
                         alpha_beta.alpha, alpha_beta.beta,
                         alpha_beta.alpha_i, alpha_beta.beta_i,
                         PWM_PERIOD_S);
    foc_ctrl.angle = g_flux_obs.theta;
    foc_ctrl.omega = g_flux_obs.omega_filt;

    SMO_bemf_angle(&alpha_beta);

    // foc_ctrl.angle = g_smo_obs.theta_comp;
    // foc_ctrl.omega = g_smo_obs.omega;
    #else   // HALL + PLL
    hall_update_PLL(&hall_data);
    foc_ctrl.angle = hall_data.angle;
    foc_ctrl.omega = hall_data.elec_speed;
    #endif

    float sin_theta, cos_theta;
    float angle_deg = foc_ctrl.angle * RAD_TO_DEG;
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

    park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

    foc_ctrl.abc_dq.current_d_filt =
        foc_ctrl.abc_dq.current_d * 0.1f + foc_ctrl.abc_dq.current_d_filt * 0.9f;
    foc_ctrl.abc_dq.current_q_filt =
        foc_ctrl.abc_dq.current_q * 0.1f + foc_ctrl.abc_dq.current_q_filt * 0.9f;

    float vd_ff = -foc_ctrl.omega * MOTOR_INDUCTANCE_Lq *
                  foc_ctrl.abc_dq.current_q_filt;
    float vq_ff = foc_ctrl.omega *
                  (MOTOR_INDUCTANCE_Ld * foc_ctrl.abc_dq.current_d_filt +
                   MOTOR_FLUX_LINKAGE);

    foc_ctrl.out_d = foc_id_pid_calculate(target_d, foc_ctrl.abc_dq.current_d) + vd_ff;
    foc_ctrl.out_q = foc_iq_pid_calculate(target_q, foc_ctrl.abc_dq.current_q) + vq_ff;

    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);

    svpwm.sector = svpwm_sector_calc(&alpha_beta);
    svpwm_calc_times(&alpha_beta, &svpwm, foc_datav.vbus);
    svpwm_duty_calc(&svpwm);
    bsp_pwm_set_duty(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}

/**
 * @brief FOC 速度闭环控制函数
 */
static inline void foc_speed_control(float target_speed)
{
    foc_ctrl.speed = foc_ctrl.omega;

    // 使用 PI 控制器计算目标 q 轴电流。
    foc_ctrl.target_q = foc_speed_pid_calculate(target_speed, foc_ctrl.speed);
}

/**
 * @brief FOC 位置闭环控制函数
 */
static inline void foc_position_control(float target_position)
{
    // 获取当前电角度
    foc_ctrl.angle = hall_data.angle;

    // 使用 PI 控制器计算输出
    foc_ctrl.out_q = foc_position_pid_calculate(target_position, foc_ctrl.angle);
}

#if HFI_ENABLE
/**
 * @brief FOC 电流闭环控制函数（集成 HFI）
 * @param target_d 目标 D 轴电流
 * @param target_q 目标 Q 轴电流
 * @note 低速或无位置传感器场景下，使用 HFI 估算转子位置。
 */
static inline void foc_current_control_hfi(float target_d, float target_q)
{
    clark_transform(&foc_datai, &foc_datav, &alpha_beta);

    float sin_theta, cos_theta;
    float vd_pi, vq_pi;
    float i_d_fund, i_q_fund;  // 基频电流分量
    float angle_deg;

    // 更新磁链观测器，用于与 HFI 结果配合或切换。
    // hall_update_PLL(&hall_data);
    flux_observer_update(&g_flux_obs,
                         alpha_beta.alpha, alpha_beta.beta,
                         alpha_beta.alpha_i, alpha_beta.beta_i,
                         PWM_PERIOD_S);

    #if HFI_STANDALONE_MODE
        hfi_observer_update(&g_hfi_obs, alpha_beta.alpha_i, alpha_beta.beta_i);
        float u_inject = hfi_get_inject_voltage(&g_hfi_obs);

        // HFI 状态机处理
        switch (g_hfi_obs.state) {
            case HFI_STATE_CONVERGE:
                hfi_pll_update(&g_hfi_pll, &g_hfi_obs, PWM_PERIOD_S);

                foc_ctrl.angle = g_hfi_pll.theta_est;
                angle_deg = foc_ctrl.angle * RAD_TO_DEG;
                arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

                park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

                // 收敛阶段仅保留注入量，PI 输出置零。
                vd_pi = 0.0f;
                vq_pi = 0.0f;

                // 根据相位误差判断是否收敛。
                if (fabsf(g_hfi_pll.error) < 0.05f) {
                    if (++g_hfi_obs.converge_cnt > 100) {
                        g_hfi_obs.state = HFI_STATE_NSD;
                        g_hfi_obs.converge_cnt = 0;
                    }
                } else {
                    g_hfi_obs.converge_cnt = 0;
                }
                break;

            case HFI_STATE_NSD:
                // 使用当前估计角度执行极性判定。
                foc_ctrl.angle = g_hfi_pll.theta_est;

                angle_deg = foc_ctrl.angle * RAD_TO_DEG;
                arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);
                park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

                if (!g_hfi_obs.polarity_determined) {
                    hfi_nsd_check(&g_hfi_obs, &g_hfi_pll,
                                  foc_ctrl.abc_dq.current_d,
                                  foc_ctrl.abc_dq.current_q);
                }

                // NSD 阶段降低注入幅值，减小对电机的扰动。
                u_inject *= 0.2f;
                vd_pi = g_hfi_obs.nsd_target_d;
                vq_pi = 0.0f;

                if (g_hfi_obs.polarity_determined) {
                    g_hfi_obs.state = HFI_STATE_RUN;
                }
                break;

            case HFI_STATE_RUN:
                hfi_pll_update(&g_hfi_pll, &g_hfi_obs, PWM_PERIOD_S);
                foc_ctrl.angle = g_hfi_pll.theta_est + PI;
                angle_deg = foc_ctrl.angle * RAD_TO_DEG;
                arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);
                park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

                hfi_extract_fundamental(&g_hfi_obs,
                                        foc_ctrl.abc_dq.current_d,
                                        foc_ctrl.abc_dq.current_q,
                                        &i_d_fund, &i_q_fund);

                vd_pi = foc_id_pid_calculate(target_d, i_d_fund);
                vq_pi = foc_iq_pid_calculate(target_q, i_q_fund);
                break;
        }

        foc_ctrl.out_d = vd_pi + u_inject;
        foc_ctrl.out_q = vq_pi;
    #else  // HFI 混合模式
        // ========== 模式切换参数配置 ==========
        #define SPEED_HFI_LOW       80.0f   // 从纯 HFI 进入混合区的速度阈值
        #define SPEED_HFI_HIGH      120.0f  // 从混合区切到磁链观测器的速度阈值
        #define SPEED_FILT_ALPHA    0.1f    // 速度滤波系数

        // ========== 状态变量 ==========
        static uint8_t mode_transition_cnt = 0;  // 切换防抖计数器
        static uint8_t hfi_settle_cnt = 0;       // HFI 收敛稳定计数器

        // 更新 HFI 观测器和 PLL
        hfi_observer_update(&g_hfi_obs, alpha_beta.alpha_i, alpha_beta.beta_i);
        hfi_pll_update(&g_hfi_pll, &g_hfi_obs, PWM_PERIOD_S);

        // 更新磁链观测器
        flux_observer_update(&g_flux_obs, alpha_beta.alpha, alpha_beta.beta,
                             alpha_beta.alpha_i, alpha_beta.beta_i, PWM_PERIOD_S);

        // 获取 HFI 注入电压
        float u_inject = hfi_get_inject_voltage(&g_hfi_obs);

        // 极性尚未确定时，继续做 NSD 判定
        if (!g_hfi_obs.polarity_determined) {
            hfi_nsd_check(&g_hfi_obs, &g_hfi_pll,
                          foc_ctrl.abc_dq.current_d, foc_ctrl.abc_dq.current_q);
        }

        // 取两种估算速度中较小的一项作为切换依据，降低误切换概率。
        float speed_inst = (fabsf(g_hfi_pll.omega_est_filt) < fabsf(g_flux_obs.omega_filt)) ?
                           fabsf(g_hfi_pll.omega_est_filt) :
                           fabsf(g_flux_obs.omega_filt);

        // ========== 模式切换逻辑 ==========
        switch (foc_ctrl.sensor_mode) {
            case SENSORLESS_STATE_HFI:  // 纯 HFI 模式
                if (speed_inst > SPEED_HFI_LOW) {
                    if (++mode_transition_cnt > 100) {
                        foc_ctrl.sensor_mode = SENSORLESS_STATE_MIX;
                        mode_transition_cnt = 0;
                    }
                } else {
                    mode_transition_cnt = 0;
                }
                break;

            case SENSORLESS_STATE_MIX:  // HFI 与磁链观测器混合模式
                if (speed_inst < SPEED_HFI_LOW) {
                    if (++mode_transition_cnt > 100) {
                        foc_ctrl.sensor_mode = SENSORLESS_STATE_HFI;
                        mode_transition_cnt = 0;
                    }
                } else if (speed_inst > SPEED_HFI_HIGH) {
                    if (++mode_transition_cnt > 100) {
                        foc_ctrl.sensor_mode = SENSORLESS_STATE_FLUX;
                        mode_transition_cnt = 0;
                    }
                } else {
                    mode_transition_cnt = 0;
                }
                break;

            case SENSORLESS_STATE_FLUX:  // 纯磁链观测器模式
                if (speed_inst < SPEED_HFI_HIGH) {
                    if (++mode_transition_cnt > 100) {
                        foc_ctrl.sensor_mode = SENSORLESS_STATE_MIX;
                        hfi_settle_cnt = 0;
                        g_hfi_pll.theta_est = g_flux_obs.theta_hat;
                        g_hfi_pll.i_term = 0.0f;               // 清除积分项，避免历史误差带入
                        g_hfi_pll.omega_est = g_flux_obs.omega_filt;  // 继承当前速度估算
                        mode_transition_cnt = 0;
                    }
                } else {
                    mode_transition_cnt = 0;
                }
                break;
        }

        // ========== 角度融合与模式执行 ==========
        float theta_hfi = g_hfi_pll.theta_est;
        float theta_flux = g_flux_obs.theta_hat;
        float ratio = 0.0f;  // 融合权重：0 为纯 HFI，1 为纯磁链观测器

        switch (foc_ctrl.sensor_mode) {
            case SENSORLESS_STATE_HFI:  // 纯 HFI 模式
                foc_ctrl.angle = theta_hfi;
                foc_ctrl.sensor_mode = SENSORLESS_STATE_HFI;
                ratio = 0.0f;
                break;

            case SENSORLESS_STATE_MIX:  // 混合模式，按速度加权融合角度
                if (hfi_settle_cnt < 100) {
                    // 刚进入混合区时先等待 HFI 角度稳定。
                    foc_ctrl.angle = g_hfi_pll.theta_est;
                    hfi_settle_cnt++;
                } else {
                    ratio = (speed_inst - SPEED_HFI_LOW) /
                            (SPEED_HFI_HIGH - SPEED_HFI_LOW);
                    ratio = (ratio < 0.0f) ? 0.0f : (ratio > 1.0f) ? 1.0f : ratio;

                    foc_ctrl.angle = (1.0f - ratio) * theta_hfi + ratio * theta_flux;
                    foc_ctrl.angle = angle_normalize_pi(foc_ctrl.angle);
                }

                foc_ctrl.sensor_mode = SENSORLESS_STATE_MIX;
                break;

            case SENSORLESS_STATE_FLUX:  // 纯磁链观测器模式
                foc_ctrl.angle = theta_flux;
                foc_ctrl.sensor_mode = SENSORLESS_STATE_FLUX;
                u_inject = 0.0f;  // 关闭高频注入
                ratio = 1.0f;
                break;
        }

        // ========== 坐标变换 ==========
        CORDIC_SinCos_Rad(foc_ctrl.angle, &sin_theta, &cos_theta);
        park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

        // ========== 电流反馈选择 ==========
        float i_d_fb, i_q_fb;

        if (ratio < 0.5f) {
            // HFI 主导时，使用提取出的基频电流作为反馈。
            hfi_extract_fundamental(&g_hfi_obs, foc_ctrl.abc_dq.current_d,
                                    foc_ctrl.abc_dq.current_q, &i_d_fund, &i_q_fund);
            i_d_fb = i_d_fund + g_hfi_obs.nsd_target_d;
            i_q_fb = i_q_fund;
        } else {
            // 磁链观测器主导时，直接使用采样电流。
            i_d_fb = foc_ctrl.abc_dq.current_d;
            i_q_fb = foc_ctrl.abc_dq.current_q;
        }

        // ========== PI 计算 ==========
        vd_pi = foc_id_pid_calculate(target_d, i_d_fb);
        vq_pi = foc_iq_pid_calculate(target_q, i_q_fb);

        // ========== 注入电压叠加 ==========
        // 混合区保留注入量，纯磁链观测器模式下注入量为 0。
        foc_ctrl.out_d = vd_pi + u_inject;
        foc_ctrl.out_q = vq_pi;
    #endif

    // 统一后处理：反 Park、SVPWM 与 PWM 输出
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);
    svpwm.sector = svpwm_sector_calc(&alpha_beta);
    svpwm_calc_times(&alpha_beta, &svpwm, foc_datav.vbus);
    svpwm_duty_calc(&svpwm);
    bsp_pwm_set_duty(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}
#endif
