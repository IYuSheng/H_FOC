#include "foc_control.h"

#if HFI_ENABLE
static inline void foc_current_control_hfi(float target_d, float target_q);
#endif
static inline void foc_current_control(float target_d, float target_q);
static inline void foc_current_control_H(float target_current_d, float target_current_q);
static inline void foc_speed_control(float target_speed);
static inline void foc_position_control(float target_position);

/**
 * @brief FOC 控制任务
 */
void vFOCControlTask(void *pvParameters)
{
  foc_ctrl.target_speed = 30.0f;   // 设置目标速度
  foc_ctrl.out_q = 0.3f;           // 开环测试时的 q 轴输出电压
  foc_ctrl.out_d = 0.0f;           // 开环测试时的 d 轴输出电压
  foc_ctrl.target_q = 0.4f;        // 设置目标 Q 轴电流
  foc_ctrl.target_d = 0.0f;        // 设置目标 D 轴电流
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
  SMO_Pare_init();
  #endif

  #if HFI_ENABLE
    hfi_observer_init(&g_hfi_obs, HFI_INJECT_VOLTAGE);
    hfi_pll_init(&g_hfi_pll, HFI_PLL_KP, HFI_PLL_KI);
  #endif

  #if FOC_PARAMETER_IDENTIFICATION_ENABLE
    foc_motor_param_ident_start(0.8f, 0.5f, 500.0f, 0.4f, 0.05f);
  #endif
  
  for (;;)
    {
    #if DEBUG_MODE

    // debug_printf("%.4f,%.4f,%.4f,%.4f,%.4f", foc_ctrl.target_q, foc_ctrl.abc_dq.current_q, foc_ctrl.abc_dq.current_d, foc_ctrl.out_q, g_flux_obs.theta);
    // debug_log("%.4f,%.4f", g_flux_obs.omega_filt, Angle_SMOPare.Ui);
    // debug_log("%.4f,%.4f,%.4f,%.4f", Angle_SMOPare.Ealpha, Angle_SMOPare.Ebeta, alpha_beta.alpha_i, alpha_beta.beta_i);
    // debug_log("%.4f,%.4f", g_speed, foc_ctrl.angle);
    // debug_log("%.4f,%.4f", alpha_beta.alpha_i, alpha_beta.beta_i);
    
    // debug_printf("%.4f,%.4f,%.4f", foc_datai.ia, foc_datai.ib, foc_datai.ic);
    // debug_printf("%.4f,%.4f,%.4f", foc_datav.va, foc_datav.vb, foc_datav.vc);
    // debug_printf("%.4f, %.4f", foc_ctrl.angle, bemf_angle);
    // debug_printf("%.4f, %.4f", foc_datai.ia, foc_datav.va);
    // debug_printf("%.4f", foc_datav.vbus);
    // debug_printf("%d,%d,%d", svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);

    debug_log("%.4f,%.4f, %.4f, %.4f", g_flux_obs.theta, Angle_SMOPare.Theta_pre, Angle_SMOPare.Ealpha, Angle_SMOPare.Ebeta);
    // debug_log("%.4f,%.4f,%.4f,%.4f", g_hfi_pll.theta_est + PI, g_flux_obs.theta, g_hfi_obs.i_env_alpha, g_hfi_obs.i_env_beta);
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
        // FOC 电流闭环（低速 Hall，高速观测器）
        // foc_current_control_H(foc_ctrl.target_d, foc_ctrl.target_q);
        #endif
    #endif
}

void foc_control_out(void)
{
    // foc_speed_control(foc_ctrl.target_speed);
    // foc_position_control(foc_ctrl.target_position);
}

/**
 * @brief FOC 速度开环控制
 */
void foc_open_loop_control(void)
{
    static float32_t angle_accum = 0.0f;  // 电角度积分累加器
    
    clark_transform(&foc_datai,&foc_datav, &alpha_beta);

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

    #if FLUX_OBSERVER_ENABLE
    flux_observer_update(&g_flux_obs,
                         alpha_beta.alpha, alpha_beta.beta,
                         alpha_beta.alpha_i, alpha_beta.beta_i,
                         PWM_PERIOD_S);
    foc_ctrl.angle = g_flux_obs.theta;
    foc_ctrl.omega = g_flux_obs.omega_filt;
    float sin_theta, cos_theta;
    float angle_deg = foc_ctrl.angle * RAD_TO_DEG;
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

    #elif SMO_OBSERVER_ENABLE
    flux_observer_update(&g_flux_obs,
                         alpha_beta.alpha, alpha_beta.beta,
                         alpha_beta.alpha_i, alpha_beta.beta_i,
                         PWM_PERIOD_S);
    foc_ctrl.angle = g_flux_obs.theta;
    foc_ctrl.omega = g_flux_obs.omega_filt;

    SMO_bemf_angle(&alpha_beta);

    // foc_ctrl.angle = Angle_SMOPare.Theta_pre + _PI_2;
    // foc_ctrl.omega = Angle_SMOPare.Ui;

    float sin_theta, cos_theta;
    float angle_deg = foc_ctrl.angle * RAD_TO_DEG;
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);
    #else
    hall_update_PLL(&hall_data);

    foc_ctrl.angle = hall_data.angle;
    foc_ctrl.omega = hall_data.elec_speed;
    float sin_theta, cos_theta;
    float angle_deg = foc_ctrl.angle * RAD_TO_DEG;
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);
    #endif

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
 * @brief 鐢垫祦闂幆鎺у埗(浣庨€烪ALL锛岄珮閫熻娴嬪櫒)
 */
static inline void foc_current_control_H(float target_current_d, float target_current_q)
{
    // 婊炵幆鍒囨崲闃堝€硷紙闃叉棰戠箒鎶栧姩锛?
    const float W_HALL_TO_OBS = 120.0f;   // HALL 鈫?瑙傛祴鍣?鍒囨崲閫熷害
    const float W_OBS_TO_HALL = 80.0f;   // 瑙傛祴鍣?鈫?HALL 鍒囨崲閫熷害锛堟粸鐜笅闄愶級
    static uint8_t sensor_state = SENSOR_HALL;

    /************************** 0. Clark鍙樻崲 **************************/
    clark_transform(&foc_datai, &foc_datav, &alpha_beta);

    /************************** 1. 浼犳劅鍣ㄦ洿鏂颁笌鍒囨崲 **************************/
    
    // 鏇存柊HALL
    hall_update_PLL(&hall_data);
    
    // 鏇存柊瑙傛祴鍣?
    flux_observer_update(&g_flux_obs,
                         alpha_beta.alpha, alpha_beta.beta,
                         alpha_beta.alpha_i, alpha_beta.beta_i,
                         PWM_PERIOD_S);
    
    // 婊炵幆鐘舵€佹満鍒囨崲
    switch (sensor_state) {
        case SENSOR_HALL:
            // 褰撳墠鐢℉ALL锛屽垽鏂槸鍚﹂渶瑕佸垏鎹㈠埌瑙傛祴鍣?
            if (hall_data.elec_speed > W_HALL_TO_OBS) {
                sensor_state = SENSOR_BLEND;
            }
            foc_ctrl.angle = hall_data.angle;
            foc_ctrl.omega = hall_data.elec_speed;
            break;
            
        case SENSOR_BLEND:
            // 娣峰悎杩囨浮鍖?
            if (hall_data.elec_speed < W_OBS_TO_HALL) {
                // 閫熷害鍥炶惤锛屽垏鍥濰ALL
                sensor_state = SENSOR_HALL;
            } else if (hall_data.elec_speed > W_HALL_TO_OBS) {
                // 閫熷害瓒冲锛屽畬鍏ㄥ垏鎹㈠埌瑙傛祴鍣?
                sensor_state = SENSOR_OBSERVER;
            } else {
                // 娣峰悎妯″紡涓嬮€熷害鍙堣繘婊炵幆鍖猴紝娣峰悎瑙掑害
                float ratio = (hall_data.elec_speed - W_OBS_TO_HALL) / (W_HALL_TO_OBS - W_OBS_TO_HALL);
                foc_ctrl.angle = angle_normalize(hall_data.angle * (1 - ratio) + g_flux_obs.theta * ratio);
                foc_ctrl.omega = hall_data.elec_speed;
            }
            break;
            
        case SENSOR_OBSERVER:
            // 褰撳墠鐢ㄨ娴嬪櫒锛屽垽鏂槸鍚﹂渶瑕佸垏鍥?
            if (g_flux_obs.omega_filt < W_OBS_TO_HALL) {
                sensor_state = SENSOR_BLEND;
            }
            foc_ctrl.angle = g_flux_obs.theta;
            foc_ctrl.omega = g_flux_obs.omega_filt;
            break;
    }

    /************************** 2. Park鍙樻崲 **************************/
    float sin_theta, cos_theta;
    float angle_deg = foc_ctrl.angle * RAD_TO_DEG;
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

    // 鐢垫祦Park鍙樻崲
    park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

    /************************** 3. 鍓嶉瑙ｈ€?**************************/
    // 婊ゆ尝DQ鐢垫祦
    foc_ctrl.abc_dq.current_d_filt = foc_ctrl.abc_dq.current_d * 0.1f + 
                                     foc_ctrl.abc_dq.current_d_filt * 0.9f;
    foc_ctrl.abc_dq.current_q_filt = foc_ctrl.abc_dq.current_q * 0.1f + 
                                     foc_ctrl.abc_dq.current_q_filt * 0.9f;
    
    // 璁＄畻鍓嶉鐢靛帇
    float vd_ff = -foc_ctrl.omega * MOTOR_INDUCTANCE_Lq * foc_ctrl.abc_dq.current_q_filt;
    float vq_ff =  foc_ctrl.omega * (MOTOR_INDUCTANCE_Ld * foc_ctrl.abc_dq.current_d_filt + MOTOR_FLUX_LINKAGE);

    /************************** 4. 鐢垫祦鐜疨ID **************************/
    
    foc_ctrl.out_d = foc_id_pid_calculate(target_current_d, foc_ctrl.abc_dq.current_d) + vd_ff;
    foc_ctrl.out_q = foc_iq_pid_calculate(target_current_q, foc_ctrl.abc_dq.current_q) + vq_ff;

    /************************** 5. 鍙峆ark + SVPWM **************************/
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);

    // SVPWM
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
    foc_ctrl.speed = g_flux_obs.omega_filt;
    // 使用 PI 控制器计算目标 q 轴电流
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
 * @brief FOC鐢垫祦闂幆鎺у埗鍑芥暟锛堥泦鎴怘FI锛?
 * @param target_d 鐩爣D杞寸數娴?
 * @param target_q 鐩爣Q杞寸數娴?
 * @note 褰撶紪鐮佸櫒澶辨晥鎴栦綆閫熸椂鑷姩鍒囨崲鍒癏FI妯″紡
 */
static inline void foc_current_control_hfi(float target_d, float target_q)
{
    clark_transform(&foc_datai,&foc_datav, &alpha_beta);

    float sin_theta, cos_theta;
    float vd_pi, vq_pi;
    float i_d_fund, i_q_fund; // 鍩洪鐢垫祦
    float angle_deg;

    // 娴嬭瘯鐢?
    // hall_update_PLL(&hall_data);
    flux_observer_update(&g_flux_obs,
                         alpha_beta.alpha, alpha_beta.beta,
                         alpha_beta.alpha_i, alpha_beta.beta_i,
                         PWM_PERIOD_S);

    #if HFI_STANDALONE_MODE
        hfi_observer_update(&g_hfi_obs, alpha_beta.alpha_i, alpha_beta.beta_i);
        float u_inject = hfi_get_inject_voltage(&g_hfi_obs);

        // 鐘舵€佹満澶勭悊
        switch (g_hfi_obs.state) {
            case HFI_STATE_CONVERGE:
                hfi_pll_update(&g_hfi_pll, &g_hfi_obs, PWM_PERIOD_S);

                foc_ctrl.angle = g_hfi_pll.theta_est;
                angle_deg = foc_ctrl.angle * RAD_TO_DEG;
                arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

                park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

                // 寮€鐜細浠呮敞鍏ワ紝PI 杈撳嚭寮哄埗涓?
                vd_pi = 0.0f;
                vq_pi = 0.0f;

                // 鏀舵暃鍒ゆ柇锛堝熀浜庤宸級
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
                // 浣跨敤鏀舵暃鍚庣殑鍥哄畾瑙掑害
                foc_ctrl.angle = g_hfi_pll.theta_est;

                angle_deg = foc_ctrl.angle * RAD_TO_DEG;
                arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

                park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

                if (!g_hfi_obs.polarity_determined) {
                    hfi_nsd_check(&g_hfi_obs, &g_hfi_pll,
                                foc_ctrl.abc_dq.current_d,
                                foc_ctrl.abc_dq.current_q);
                }

                u_inject *= 0.2f; // NSD闃舵闄嶄綆娉ㄥ叆鐢靛帇锛屽噺灏戝鐢垫満鐨勫啿鍑?
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
    #else  // HFI娣峰悎妯″紡
        // ========== 妯″紡鍒囨崲鍙傛暟閰嶇疆 ==========
        #define SPEED_HFI_LOW       80.0f   // HFI涓婇檺锛堣繘鍏ユ粸鐜尯锛?
        #define SPEED_HFI_HIGH      120.0f   // 纾侀摼瑙傛祴鍣ㄤ笅闄愶紙閫€鍑烘粸鐜尯锛?
        #define SPEED_FILT_ALPHA    0.1f     // 閫熷害婊ゆ尝绯绘暟
        
        // ========== 閫熷害浼拌涓庢护娉?==========
        static uint8_t mode_transition_cnt = 0;  // 妯″紡鍒囨崲闃叉姈璁℃暟鍣?
        static uint8_t hfi_settle_cnt = 0;       // HFI绋冲畾璁℃暟鍣?
    
        // 鏇存柊瑙傛祴鍣?
        hfi_observer_update(&g_hfi_obs, alpha_beta.alpha_i, alpha_beta.beta_i);
        hfi_pll_update(&g_hfi_pll, &g_hfi_obs, PWM_PERIOD_S);

        flux_observer_update(&g_flux_obs, alpha_beta.alpha, alpha_beta.beta,
                            alpha_beta.alpha_i, alpha_beta.beta_i, PWM_PERIOD_S);
        
        // 鑾峰彇HFI娉ㄥ叆鐢靛帇
        float u_inject = hfi_get_inject_voltage(&g_hfi_obs);
        
        // HFI鏋佹€ф娴?
        if (!g_hfi_obs.polarity_determined) {
            hfi_nsd_check(&g_hfi_obs, &g_hfi_pll, 
                        foc_ctrl.abc_dq.current_d, foc_ctrl.abc_dq.current_q);
        }
        
        // 閫熷害浼拌锛氬彇涓よ€呰緝灏忓€?
        float speed_inst = (fabsf(g_hfi_pll.omega_est_filt) < fabsf(g_flux_obs.omega_filt)) ? 
                        fabsf(g_hfi_pll.omega_est_filt) : fabsf(g_flux_obs.omega_filt);
        
        // ========== 婊炵幆鍒囨崲閫昏緫 ==========
        switch (foc_ctrl.sensor_mode) {
            case SENSORLESS_STATE_HFI:  // 褰撳墠绾疕FI妯″紡
                if (speed_inst > SPEED_HFI_LOW) {
                    if (++mode_transition_cnt > 100) {  // 5ms闃叉姈
                        foc_ctrl.sensor_mode = SENSORLESS_STATE_MIX;  // 杩涘叆婊炵幆鍖?
                        mode_transition_cnt = 0;
                    }
                } else {
                    mode_transition_cnt = 0;
                }
                break;
                
            case SENSORLESS_STATE_MIX:  // 褰撳墠婊炵幆娣峰悎妯″紡
                if (speed_inst < SPEED_HFI_LOW) {
                    if (++mode_transition_cnt > 100) {
                        foc_ctrl.sensor_mode = SENSORLESS_STATE_HFI;  // 鍥炲埌绾疕FI
                        mode_transition_cnt = 0;
                    }
                } else if (speed_inst > SPEED_HFI_HIGH) {
                    if (++mode_transition_cnt > 100) {
                        foc_ctrl.sensor_mode = SENSORLESS_STATE_FLUX;  // 杩涘叆绾閾?
                        mode_transition_cnt = 0;
                    }
                } else {
                    mode_transition_cnt = 0;
                }
                break;
                
            case SENSORLESS_STATE_FLUX:  // 褰撳墠绾閾炬ā寮?
                if (speed_inst < SPEED_HFI_HIGH) {
                    if (++mode_transition_cnt > 100) {
                        foc_ctrl.sensor_mode = SENSORLESS_STATE_MIX;  // 鍥炲埌婊炵幆鍖?
                        hfi_settle_cnt = 0;  // 閲嶇疆HFI绋冲畾璁℃暟鍣?
                        g_hfi_pll.theta_est = g_flux_obs.theta_hat;
                        g_hfi_pll.i_term = 0.0f;  // 娓呯┖绉垎椤癸紝閬垮厤绱Н璇樊
                        g_hfi_pll.omega_est = g_flux_obs.omega_filt;  // 缁ф壙閫熷害浼拌
                        mode_transition_cnt = 0;
                    }
                } else {
                    mode_transition_cnt = 0;
                }
                break;
        }
        
        // ========== 瑙掑害铻嶅悎涓庢ā寮忔墽琛?==========
        float theta_hfi = g_hfi_pll.theta_est;
        float theta_flux = g_flux_obs.theta_hat;
        float ratio = 0.0f;  // 纾侀摼鏉冮噸锛?=绾疕FI, 1=绾閾撅級
        
        switch (foc_ctrl.sensor_mode) {
            case SENSORLESS_STATE_HFI:  // 绾疕FI妯″紡
                foc_ctrl.angle = theta_hfi;
                foc_ctrl.sensor_mode = SENSORLESS_STATE_HFI;
                ratio = 0.0f;
                break;

            case SENSORLESS_STATE_MIX:  // 婊炵幆娣峰悎妯″紡 - 瑙掑害鍔犳潈铻嶅悎
                {
                    if (hfi_settle_cnt < 100) {  // 5ms鏀舵暃鏃堕棿
                        foc_ctrl.angle = g_hfi_pll.theta_est;
                        hfi_settle_cnt++;
                    }
                    else
                    {
                        // 绾挎€у姞鏉冿細閫熷害瓒婇珮閫燂紝纾侀摼鏉冮噸瓒婂ぇ
                        ratio = (speed_inst - SPEED_HFI_LOW) / (SPEED_HFI_HIGH - SPEED_HFI_LOW);
                        ratio = (ratio < 0.0f) ? 0.0f : (ratio > 1.0f) ? 1.0f : ratio;
                        
                        // 鍔犳潈铻嶅悎瑙掑害
                        foc_ctrl.angle = (1.0f - ratio) * theta_hfi + ratio * theta_flux;
                        foc_ctrl.angle = angle_normalize_pi(foc_ctrl.angle);
                    }
                    
                    foc_ctrl.sensor_mode = SENSORLESS_STATE_MIX;
                }
                break;
                
            case SENSORLESS_STATE_FLUX:  // 绾閾捐娴嬪櫒妯″紡
                foc_ctrl.angle = theta_flux;
                foc_ctrl.sensor_mode = SENSORLESS_STATE_FLUX;
                u_inject = 0.0f;  // 绾閾炬ā寮忓叧闂敞鍏?
                ratio = 1.0f;
                break;
        }
        
        // ========== 鍧愭爣鍙樻崲 ==========
        CORDIC_SinCos_Rad(foc_ctrl.angle, &sin_theta, &cos_theta);
        park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);
        
        // ========== 鐢垫祦鍙嶉閫夋嫨 ==========
        float i_d_fb, i_q_fb;
        
        if (ratio < 0.5f) {
            // HFI涓诲锛氭彁鍙栧熀棰戠數娴侊紙婊ら櫎楂橀娉ㄥ叆鍒嗛噺锛?
            hfi_extract_fundamental(&g_hfi_obs, foc_ctrl.abc_dq.current_d, 
                                foc_ctrl.abc_dq.current_q, &i_d_fund, &i_q_fund);
            i_d_fb = i_d_fund + g_hfi_obs.nsd_target_d;
            i_q_fb = i_q_fund;
        } else {
            // 纾侀摼涓诲锛氱洿鎺ヤ娇鐢ㄩ噰鏍风數娴?
            i_d_fb = foc_ctrl.abc_dq.current_d;
            i_q_fb = foc_ctrl.abc_dq.current_q;
        }
        
        // ========== PI璁＄畻 ==========
        vd_pi = foc_id_pid_calculate(target_d, i_d_fb);
        vq_pi = foc_iq_pid_calculate(target_q, i_q_fb);
        
        // ========== 娉ㄥ叆鐢靛帇琛板噺 ==========
        // 婊炵幆鍖哄唴绾挎€ц“鍑忥紝绾閾炬ā寮忔椂瀹屽叏鍏抽棴娉ㄥ叆
        foc_ctrl.out_d = vd_pi + u_inject;
        foc_ctrl.out_q = vq_pi;

    #endif

    // 缁熶竴鐨勫悗澶勭悊
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);
    svpwm.sector = svpwm_sector_calc(&alpha_beta);
    svpwm_calc_times(&alpha_beta, &svpwm, foc_datav.vbus);
    svpwm_duty_calc(&svpwm);
    bsp_pwm_set_duty(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}
#endif

