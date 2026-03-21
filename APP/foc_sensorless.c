#include "foc_conversion.h"
#include "foc_sensorless.h"

/* ---------------------非线性磁链观测器---------------------- */

FluxObserver_t g_flux_obs;

/**
 * @brief 磁链观测器初始化
 * @param obs 观测器状态结构体指针
 */
void flux_observer_init(FluxObserver_t *obs)
{
    obs->x1 = 0.0f;
    obs->x2 = 0.0f;
    obs->cos_theta = 1.0f;
    obs->sin_theta = 0.0f;
}

/**
 * @brief 磁链观测器更新函数
 * @param obs 观测器状态结构体指针
 * @param u_alpha α轴电压输入
 * @param u_beta β轴电压输入
 * @param i_alpha α轴电流输入
 * @param i_beta β轴电流输入
 * @param Ts 采样周期，单位 s
 */
void flux_observer_update(FluxObserver_t *obs, float u_alpha, float u_beta,
                          float i_alpha, float i_beta, float Ts)
{
    static ButterworthLPF_t omega_flux_filt;
    static uint8_t filt_inited = 0;

    // 预留固定角度补偿项，便于后续修正观测器与实际相位的偏差。
    #define ANGLE_OFFSET_COMPENSATION  0.0f
    
    // 首次进入时初始化角速度低通滤波器。
    if (!filt_inited) {
        butterworth_init(&omega_flux_filt, 10.0f, PWM_FREQ);
        filt_inited = 1;
    }

    // ===== 1. 非线性磁链观测 =====
    float y1 = u_alpha - FLUX_R_S * i_alpha;
    float y2 = u_beta  - FLUX_R_S * i_beta;
    
    // eta = x - Ls * i，表示估计磁链与电流模型磁链之间的偏差。
    obs->eta1 = obs->x1 - MOTOR_INDUCTANCE_Ld * i_alpha;
    obs->eta2 = obs->x2 - MOTOR_INDUCTANCE_Lq * i_beta;
    
    // 误差向量模长平方，用于构造非线性校正项。
    float eta_norm_sq = obs->eta1 * obs->eta1 + obs->eta2 * obs->eta2;

    // 非线性校正项：gamma / 2 * eta * (phi_m^2 - ||eta||^2)
    float error_term = FLUX_GAMMA_K * (FLUX_PHI_M_2 - eta_norm_sq);
    
    // 采用欧拉积分更新观测器内部状态。
    obs->x1 += Ts * (y1 + error_term * obs->eta1);
    obs->x2 += Ts * (y2 + error_term * obs->eta2);
    
    // ===== 2. PLL 提取角度 =====
    // 输入：eta = phi_m * [cos(theta), sin(theta)]
    // 输出：theta_hat、cos(theta_hat)、sin(theta_hat)
    
    // 计算角度误差：
    // sin(theta - theta_hat) ≈ sin(theta) * cos(theta_hat) - cos(theta) * sin(theta_hat)
    float sin_theta_real = obs->eta2 * _1_FLUX_PHI_M;
    float cos_theta_real = obs->eta1 * _1_FLUX_PHI_M;
  
    // 采用小角度近似作为 PLL 的相位误差输入。
    float angle_error = sin_theta_real * obs->cos_theta - cos_theta_real * obs->sin_theta;
    
    // PLL 的 PI 调节器，积分项做限幅避免 windup。
    obs->omega_integ += PLL_KI * angle_error * Ts;
    if (obs->omega_integ > MAX_SPEED) obs->omega_integ = MAX_SPEED;
    if (obs->omega_integ < -MAX_SPEED) obs->omega_integ = -MAX_SPEED;

    float omega_pll = obs->omega_integ + SMO_PLL_KP * angle_error;

    // 对 PLL 的原始角速度估计做低通，减小速度抖动。
    obs->omega_filt = butterworth_filter(&omega_flux_filt, omega_pll);

    // 对角速度积分得到观测角度。
    obs->theta_hat += omega_pll * Ts;
    
    // 角度归一化到 -pi ~ pi。
    obs->theta_hat = angle_normalize_pi(obs->theta_hat);

    float base_offset = (obs->omega_filt > 0) ? FLUX_ANGLE_OFFSET_ACTURE  : -FLUX_ANGLE_OFFSET_ACTURE;

    obs->theta = angle_normalize(obs->theta_hat - base_offset);

    // 归一化到 0~360°，用于 arm_sin_cos_f32。
    float theta_deg = angle_normalize_360(rad2deg(obs->theta_hat));

    // 更新输出正余弦。
    arm_sin_cos_f32(theta_deg, &obs->sin_theta, &obs->cos_theta);
}

/* ---------------------SMO 滑模观测器---------------------- */

SMO_MotorPare_t SMO_MotorPare;
Ppll_obj_t Angle_SMOPare;

static float32_t smo_sat(float32_t err, float32_t band)
{
    // 边界层饱和函数：
    // 误差较小时保留线性区，降低抖振；
    // 误差较大时退化成符号函数，加快收敛速度。
    if (band <= 1e-6f) {
        if (err > 0.0f) return 1.0f;
        if (err < 0.0f) return -1.0f;
        return 0.0f;
    }

    if (err > band) return 1.0f;
    if (err < -band) return -1.0f;
    return err / band;
}

static void Pll_Compute(Ppll_obj_t *ptHandle, float Coff_Sine, float Coff_Cos)
{
    // 这里将估计反电动势矢量送入 PLL，提取电角度和电角速度。
    float cos_value = arm_cos_f32(ptHandle->Theta);
    float sin_value = arm_sin_f32(ptHandle->Theta);

    ptHandle->Err = Coff_Sine * sin_value - Coff_Cos * cos_value;
    ptHandle->Interg += ptHandle->Err * ptHandle->tPll.Ki;

    if (ptHandle->Interg > SMO_PLL_INT_LIMIT) ptHandle->Interg = SMO_PLL_INT_LIMIT;
    if (ptHandle->Interg < -SMO_PLL_INT_LIMIT) ptHandle->Interg = -SMO_PLL_INT_LIMIT;

    ptHandle->Ui = ptHandle->Err * ptHandle->tPll.Kp + ptHandle->Interg;

    if (ptHandle->Ui > SMO_PLL_INT_LIMIT) ptHandle->Ui = SMO_PLL_INT_LIMIT;
    if (ptHandle->Ui < -SMO_PLL_INT_LIMIT) ptHandle->Ui = -SMO_PLL_INT_LIMIT;

    ptHandle->Theta += ptHandle->Ui * SMO_MotorPare.Ts;
    ptHandle->Theta = angle_normalize(ptHandle->Theta);

    ptHandle->Speed_Rpm = ptHandle->tPll.Speed_coeff * ptHandle->Ui;
    ptHandle->SpeedLpf_Rpm += ptHandle->tPll.Kslf *
                              (ptHandle->Speed_Rpm - ptHandle->SpeedLpf_Rpm);
}

void SMO_Reset(void)
{
    // 清空所有中间状态，重新启动观测器时避免带入旧状态。
    Angle_SMOPare.EstIalpha = 0.0f;
    Angle_SMOPare.EstIbeta = 0.0f;
    Angle_SMOPare.IalphaError = 0.0f;
    Angle_SMOPare.IbetaError = 0.0f;
    Angle_SMOPare.Zalpha = 0.0f;
    Angle_SMOPare.Zbeta = 0.0f;
    Angle_SMOPare.Ealpha = 0.0f;
    Angle_SMOPare.Ebeta = 0.0f;
    Angle_SMOPare.Theta = 0.0f;
    Angle_SMOPare.Theta_pre = 0.0f;
    Angle_SMOPare.Err = 0.0f;
    Angle_SMOPare.Interg = 0.0f;
    Angle_SMOPare.Ui = 0.0f;
    Angle_SMOPare.Speed_Rpm = 0.0f;
    Angle_SMOPare.SpeedLpf_Rpm = 0.0f;
}

void SMO_Pare_init(void)
{
    // 从工程配置中读取电机参数，并离散化电流模型：
    //   di/dt = (u - e - R*i - z) / L
    //   i(k+1) = F * i(k) + G * (u - e - z)
    SMO_MotorPare.Rs = MOTOR_RESISTANCE;
    SMO_MotorPare.Ls = MOTOR_INDUCTANCE;
    SMO_MotorPare.Ts = PWM_PERIOD_S;
    SMO_MotorPare.POLES = MOTOR_POLE_PAIRS;

    if (SMO_MotorPare.Ls < 1e-9f) {
        SMO_MotorPare.Ls = 1e-9f;
    }
    if (SMO_MotorPare.POLES == 0U) {
        SMO_MotorPare.POLES = 1U;
    }

    SMO_MotorPare.Fsmopos = 1.0f -
                            (SMO_MotorPare.Rs * SMO_MotorPare.Ts / SMO_MotorPare.Ls);
    SMO_MotorPare.Gsmopos = SMO_MotorPare.Ts / SMO_MotorPare.Ls;

    // 滑模增益 Kslide 决定观测器“拉回误差”的力度。
    // 过小会跟踪无力，过大则容易带来抖振和反电动势削顶。
    Angle_SMOPare.Kslide = SMO_MotorPare.Rs * SMO_SLIDE_GAIN_FACTOR;
    // 反电动势低通系数，用于从开关量 z 中提取平滑的 e_alpha/e_beta。
    Angle_SMOPare.Kslf_emf = SMO_EMF_FILTER_COEFF;
    // 边界层宽度，影响饱和函数的线性区大小。
    Angle_SMOPare.E0 = SMO_CURRENT_ERR_BAND;

    // PLL 参数用于把反电动势矢量转换成角度和速度。
    Angle_SMOPare.tPll.Kp = SMO_PLL_KP;
    Angle_SMOPare.tPll.Ki = SMO_PLL_KI;
    Angle_SMOPare.tPll.Speed_coeff = 60.0f /
                                     (2.0f * SMO_MotorPare.POLES * PI);
    Angle_SMOPare.tPll.Kslf = SMO_PLL_SPEED_FILTER_COEFF;

    SMO_Reset();
}

float32_t SMO_bemf_angle_from_voltage_current(float32_t u_alpha, float32_t u_beta,
                                              float32_t i_alpha, float32_t i_beta)
{
    if (SMO_MotorPare.Ls <= 0.0f || SMO_MotorPare.Ts <= 0.0f) {
        SMO_Pare_init();
    }

    // 1. 根据离散电流模型更新估计电流。
    //    这里的输入量均位于 alpha-beta 静止坐标系。
    Angle_SMOPare.EstIalpha = SMO_MotorPare.Fsmopos * Angle_SMOPare.EstIalpha +
                              SMO_MotorPare.Gsmopos *
                              (u_alpha - Angle_SMOPare.Ealpha - Angle_SMOPare.Zalpha);
    Angle_SMOPare.EstIbeta = SMO_MotorPare.Fsmopos * Angle_SMOPare.EstIbeta +
                             SMO_MotorPare.Gsmopos *
                             (u_beta - Angle_SMOPare.Ebeta - Angle_SMOPare.Zbeta);

    // 2. 估计电流与采样电流作差，得到滑模面误差。
    Angle_SMOPare.IalphaError = Angle_SMOPare.EstIalpha - i_alpha;
    Angle_SMOPare.IbetaError = Angle_SMOPare.EstIbeta - i_beta;

    // 3. 通过饱和函数生成滑模开关量。
    //    这是 SMO 的核心校正项，负责快速抑制电流模型误差。
    Angle_SMOPare.Zalpha = Angle_SMOPare.Kslide *
                           smo_sat(Angle_SMOPare.IalphaError, Angle_SMOPare.E0);
    Angle_SMOPare.Zbeta = Angle_SMOPare.Kslide *
                          smo_sat(Angle_SMOPare.IbetaError, Angle_SMOPare.E0);

    // 4. 对开关量进行一阶低通，提取平滑反电动势。
    Angle_SMOPare.Ealpha += Angle_SMOPare.Kslf_emf *
                            (Angle_SMOPare.Zalpha - Angle_SMOPare.Ealpha);
    Angle_SMOPare.Ebeta += Angle_SMOPare.Kslf_emf *
                           (Angle_SMOPare.Zbeta - Angle_SMOPare.Ebeta);

    // 5. 用反电动势进入 PLL，得到角度和速度估计。
    Pll_Compute(&Angle_SMOPare, Angle_SMOPare.Ealpha, Angle_SMOPare.Ebeta);
    Angle_SMOPare.Theta_pre = angle_normalize(Angle_SMOPare.Theta +
                                              SMO_ANGLE_COMPENSATION);

    return Angle_SMOPare.Theta_pre;
}

float32_t SMO_bemf_angle(AlphaBetaTypeDef *alpha_beta)
{
    // 保留当前 alpha-beta 电压到调试变量中，便于日志观察。
    alpha_beta->alpha_v = alpha_beta->alpha;
    alpha_beta->beta_v = alpha_beta->beta;

    return SMO_bemf_angle_from_voltage_current(alpha_beta->alpha_v,
                                               alpha_beta->beta_v,
                                               alpha_beta->alpha_i,
                                               alpha_beta->beta_i);
}

#if HFI_ENABLE

/* ---------------------HFI 观测器---------------------- */
HFI_Observer_t g_hfi_obs;
HFI_PLL_t g_hfi_pll;

/**
 * @brief HFI 观测器初始化
 * @param obs HFI 观测器结构体指针
 * @param u_inject 注入电压幅值，单位 V
 */
void hfi_observer_init(HFI_Observer_t *obs, float u_inject)
{
    obs->u_inject = u_inject;
    obs->u_inject_d = 0.0f;
    obs->inject_flag = 0;
    obs->init_done = 0;
    
    obs->i_alpha_last[0] = 0.0f;
    obs->i_alpha_last[1] = 0.0f;
    obs->i_beta_last[0] = 0.0f;
    obs->i_beta_last[1] = 0.0f;
    
    obs->i_alpha_h = 0.0f;
    obs->i_beta_h = 0.0f;
    obs->i_alpha_h_last = 0.0f;
    obs->i_beta_h_last = 0.0f;
    
    obs->i_env_alpha = 0.0f;
    obs->i_env_beta = 0.0f;
    
    obs->i_d_fund = 0.0f;
    obs->i_q_fund = 0.0f;
    
    obs->nsd_count = 0;
    obs->sum_pos = 0.0f;
    obs->sum_neg = 0.0f;
    obs->polarity_determined = 0;
}

/**
 * @brief HFI 观测器更新，提取高频电流分量
 * @param obs HFI 观测器结构体指针
 * @param i_alpha alpha 轴电流
 * @param i_beta beta 轴电流
 * @note 建议在 PWM 周期开始处调用
 */
void hfi_observer_update(HFI_Observer_t *obs, float i_alpha, float i_beta)
{
    // 1. 提取高频电流分量。
    obs->i_alpha_h = (i_alpha - 2.0f * obs->i_alpha_last[0] + obs->i_alpha_last[1]) * 0.25f;
    obs->i_beta_h  = (i_beta  - 2.0f * obs->i_beta_last[0]  + obs->i_beta_last[1])  * 0.25f;
    
    // 2. 更新历史采样值。
    obs->i_alpha_last[1] = obs->i_alpha_last[0];
    obs->i_alpha_last[0] = i_alpha;
    obs->i_beta_last[1] = obs->i_beta_last[0];
    obs->i_beta_last[0] = i_beta;
    
    // 3. 计算注入极性。这里的 sign 对应上一拍的注入方向。
    float sign = (obs->inject_flag == 0) ? 1.0f : -1.0f;
    
    // 4. 解调得到电流包络。
    obs->i_env_alpha = (obs->i_alpha_h - obs->i_alpha_h_last) * sign;
    obs->i_env_beta  = (obs->i_beta_h - obs->i_beta_h_last) * sign;

    // 5. 更新高频历史量。
    obs->i_alpha_h_last = obs->i_alpha_h;
    obs->i_beta_h_last = obs->i_beta_h;

    // 6. 切换下一拍注入极性。
    obs->inject_flag ^= 1;
}

/**
 * @brief 鎻愬彇鍩洪?戠數娴佸垎閲忥紙鐢ㄤ簬鐢垫祦鐜?鍙嶉?堬級
 * @param obs HFI瑙傛祴鍣ㄧ粨鏋勪綋鎸囬拡
 * @param i_d d杞寸數娴侊紙鍚?楂橀?戯級
 * @param i_q q杞寸數娴侊紙鍚?楂橀?戯級
 * @param i_d_fund 杈撳嚭鐨刣杞村熀棰戠數娴佹寚閽?
 * @param i_q_fund 杈撳嚭鐨剄杞村熀棰戠數娴佹寚閽?
 * @note 浣跨敤婊戝姩骞冲潎婊ゆ尝鎻愬彇鍩洪?戝垎閲?
 */
void hfi_extract_fundamental(HFI_Observer_t *obs, float i_d, float i_q, 
                              float *i_d_fund, float *i_q_fund)
{
    // 浣跨敤涓夌偣婊戝姩骞冲潎鎻愬彇鍩洪?戝垎閲?
    // i_fund = (i[n] + 2*i[n-1] + i[n-2]) / 4
    static float i_d_last[2] = {0.0f, 0.0f};
    static float i_q_last[2] = {0.0f, 0.0f};
    
    *i_d_fund = (i_d + 2.0f * i_d_last[0] + i_d_last[1]) * 0.25f;
    *i_q_fund = (i_q + 2.0f * i_q_last[0] + i_q_last[1]) * 0.25f;
    
    // 淇濆瓨鍘嗗彶鍊?
    i_d_last[1] = i_d_last[0];
    i_d_last[0] = i_d;
    i_q_last[1] = i_q_last[0];
    i_q_last[0] = i_q;
    
    // 鍚屾椂鏇存柊鍒扮粨鏋勪綋
    obs->i_d_fund = *i_d_fund;
    obs->i_q_fund = *i_q_fund;
}

/**
 * @brief 鑾峰彇褰撳墠搴旀敞鍏ョ殑鐢靛帇
 * @param obs HFI瑙傛祴鍣ㄧ粨鏋勪綋鎸囬拡
 * @return 褰撳墠d杞存敞鍏ョ數鍘嬶紙鍚?鏋佹�э級
 */
float hfi_get_inject_voltage(HFI_Observer_t *obs)
{
    // 鏂规尝娉ㄥ叆锛氭牴鎹甶nject_flag鍐冲畾鏋佹�?
    obs->u_inject_d = (obs->inject_flag == 0) ? obs->u_inject : -obs->u_inject;
    return obs->u_inject_d;
}

/**
 * @brief HFI-PLL鍒濆?嬪寲
 * @param pll PLL缁撴瀯浣撴寚閽?
 * @param kp 姣斾緥澧炵泭锛堝缓璁?2.0~3.0锛?
 * @param ki 绉?鍒嗗?炵泭锛堝缓璁?15~25锛?
 */
void hfi_pll_init(HFI_PLL_t *pll, float kp, float ki)
{
    pll->kp = kp;
    pll->ki = ki;
    pll->theta_est = 0.0f;
    pll->omega_est = 0.0f;
    pll->p_term = 0.0f;
    pll->i_term = 0.0f;
    pll->error = 0.0f;
}

/**
 * @brief HFI-PLL鏇存柊 - 璺熻釜杞?瀛愪綅缃?
 * @param pll PLL缁撴瀯浣撴寚閽?
 * @param obs HFI瑙傛祴鍣ㄧ粨鏋勪綋鎸囬拡锛堜娇鐢ㄥ叾涓?鐨刬_env_alpha/beta锛?
 * @param Ts 閲囨牱鍛ㄦ湡(s)
 * @note 鍩轰簬鐢垫祦鍖呯粶鐨勫弽姝ｅ垏杩涜?屼綅缃?璺熻釜
 */
void hfi_pll_update(HFI_PLL_t *pll, HFI_Observer_t *obs, float Ts)
{
    static ButterworthLPF_t omega_filt;
    static uint8_t filt_inited = 0;
    
    // 鍒濆?嬪寲婊ゆ尝鍣?
    if (!filt_inited) {
        butterworth_init(&omega_filt, 10.0f, PWM_FREQ);
        filt_inited = 1;
    }
    
    float sin_theta, cos_theta;
    
    // ================== 1. 瑙掑害褰掍竴鍖栵紙-蟺 ~ 蟺锛?==================
    pll->theta_est = angle_normalize_pi(pll->theta_est);

    // ================== 2. 璁＄畻浼拌?¤?掑害鐨? sin/cos ==================
    float angle_deg = pll->theta_est * RAD_TO_DEG;
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);
    
    // ================== 3. 鑾峰彇瑙ｈ皟鍚庣殑鐢垫祦鍖呯粶 ==================
    float i_alpha = obs->i_env_alpha;
    float i_beta = obs->i_env_beta;
    
    // ================== 4. 姝ｄ氦瑙ｈ皟璁＄畻璇?宸? ==================
    pll->error = -i_beta * cos_theta + i_alpha * sin_theta; // 杩欓噷鍙栧弽锛岃?傛祴鍑轰负鐢佃?掑害锛?90搴?
    
    // ================== 5. PI 鎺у埗鍣? ==================
    pll->p_term = pll->kp * pll->error;
    pll->i_term += pll->ki * pll->error;
    
    // ================== 6. 绉?鍒嗛檺骞? ==================
    const float INT_LIMIT = 500.0f;
    if (pll->i_term > INT_LIMIT) pll->i_term = INT_LIMIT;
    if (pll->i_term < -INT_LIMIT) pll->i_term = -INT_LIMIT;
    
    // ================== 7. 瑙掗�熷害璁＄畻锛堝師濮嬪�硷級==================
    pll->omega_est = pll->p_term + pll->i_term;

    // ================== 8. 宸寸壒娌冩柉浣庨�氭护娉?锛堝?归�熷害婊ゆ尝锛?==================
    pll->omega_est_filt = butterworth_filter(&omega_filt, pll->omega_est);
    
    // ================== 9. 瑙掑害绉?鍒嗭紙浣跨敤婊ゆ尝鍚庣殑閫熷害锛?==================
    pll->theta_est += pll->omega_est * Ts;
}

/**
 * @brief NSD(姝ｈ礋鑴夊啿妫�娴?)鏋佹�ц鲸璇?
 * @param obs HFI瑙傛祴鍣ㄧ粨鏋勪綋鎸囬拡
 * @param pll PLL缁撴瀯浣撴寚閽堬紙鐢ㄤ簬淇?姝ｅ垵濮嬭?掑害锛?
 * @param i_d d杞寸數娴侊紙鍚?楂橀?戯級
 * @param i_q q杞寸數娴侊紙鍚?楂橀?戯級
 * @note 鍦ㄧ數鏈哄惎鍔ㄥ墠鎵ц?岋紝閫氳繃姣旇緝姝ｈ礋鑴夊啿鍝嶅簲纭?瀹氭瀬鎬?
 */
void hfi_nsd_check(HFI_Observer_t *obs, HFI_PLL_t *pll, float i_d, float i_q)
{ 
    if (obs->polarity_determined) return;
    obs->nsd_count++;

    #if HFI_NSD_ENABLE
    // 鎻愬彇d杞撮珮棰戠數娴?
    static float i_d_last[2] = {0};
    float i_d_h = (i_d - 2.0f * i_d_last[0] + i_d_last[1]) * 0.25f;
    i_d_last[1] = i_d_last[0];
    i_d_last[0] = i_d;

    // 鏍囧噯NSD搴忓垪锛?+Id -> 0 -> -Id -> 0 -> 姣旇緝
    // 闃舵??1: 0 - 200 (10ms) +1.5A 20khz
    if (obs->nsd_count < 200) {
        obs->nsd_target_d = 0.2f;
    }
    // 闃舵??2: 200 - 250 (10ms) 閲囨牱姝?
    else if (obs->nsd_count < 250) {
        obs->sum_pos += fabsf(i_d_h);
        obs->nsd_target_d = 0.2f;
    }
    // 闃舵??3: 250 - 350 (10ms) 鏂藉姞 -1.5A
    else if (obs->nsd_count < 350) {
        obs->nsd_target_d = -0.2f;
    }
    // 闃舵??4: 350 - 400 (10ms) 閲囨牱璐?
    else if (obs->nsd_count < 400) {
        obs->sum_neg += fabsf(i_d_h);
        obs->nsd_target_d = -0.2f;
    }
    // 闃舵??5锛氬垽鏂?涓庝慨姝?
    else {
        debug_log("%.4f, %.4f", obs->sum_pos, obs->sum_neg);
        if (obs->sum_neg > obs->sum_pos) {
            pll->theta_est += PI;  // 鍙嶈浆180搴?
            pll->theta_est = angle_normalize_pi(pll->theta_est);
        }
        obs->polarity_determined = 1;
        obs->nsd_target_d = 0.0f;
    }
    #else
    // 寮烘媺鍒癲杞达紝鐒跺悗鐩存帴纭?瀹氭瀬鎬?
    if (obs->nsd_count < 2000)
    {
        obs->nsd_target_d = 1.0f;
    }
    else
    {
        obs->nsd_target_d = 0.0f;
        // pll->theta_est += PI;
        pll->theta_est = angle_normalize_pi(pll->theta_est);
        obs->polarity_determined = 1;
    }
    #endif
}

#endif
