#include "foc_conversion.h"
#include "foc_sensorless.h"
#include <string.h>

/* ---------------------·ÇÏßÐÔ´ÅÁ´¹Û²âÆ÷---------------------- */

FluxObserver_t g_flux_obs;

/**
 * @brief ´ÅÁ´¹Û²âÆ÷³õÊ¼»¯
 * @param obs ¹Û²âÆ÷×´Ì¬½á¹¹ÌåÖ¸Õë
 */
void flux_observer_init(FluxObserver_t *obs)
{
    obs->x1 = 0.0f;
    obs->x2 = 0.0f;
    obs->cos_theta = 1.0f;
    obs->sin_theta = 0.0f;
}

/**
 * @brief ´ÅÁ´¹Û²âÆ÷¸üÐÂº¯Êý
 * @param obs ¹Û²âÆ÷×´Ì¬½á¹¹ÌåÖ¸Õë
 * @param u_alpha ¦ÁÖáµçÑ¹ÊäÈë
 * @param u_beta ¦ÂÖáµçÑ¹ÊäÈë
 * @param i_alpha ¦ÁÖáµçÁ÷ÊäÈë
 * @param i_beta ¦ÂÖáµçÁ÷ÊäÈë
 * @param Ts ²ÉÑùÖÜÆÚ£¬µ¥Î» s
 */
void flux_observer_update(FluxObserver_t *obs, float u_alpha, float u_beta,
                          float i_alpha, float i_beta, float Ts)
{
    static ButterworthLPF_t omega_flux_filt;
    static uint8_t filt_inited = 0;
    
    // Ê×´Î½øÈëÊ±³õÊ¼»¯½ÇËÙ¶ÈµÍÍ¨ÂË²¨Æ÷¡£
    if (!filt_inited) {
        butterworth_init(&omega_flux_filt, 10.0f, PWM_FREQ);
        filt_inited = 1;
    }

    // ===== 1. ·ÇÏßÐÔ´ÅÁ´¹Û²â =====
    float y1 = u_alpha - FLUX_R_S * i_alpha;
    float y2 = u_beta  - FLUX_R_S * i_beta;
    
    // eta = x - Ls * i£¬±íÊ¾¹À¼Æ´ÅÁ´ÓëµçÁ÷Ä£ÐÍ´ÅÁ´Ö®¼äµÄÆ«²î¡£
    obs->eta1 = obs->x1 - MOTOR_INDUCTANCE_Ld * i_alpha;
    obs->eta2 = obs->x2 - MOTOR_INDUCTANCE_Lq * i_beta;
    
    // Îó²îÏòÁ¿Ä£³¤Æ½·½£¬ÓÃÓÚ¹¹Ôì·ÇÏßÐÔÐ£ÕýÏî¡£
    float eta_norm_sq = obs->eta1 * obs->eta1 + obs->eta2 * obs->eta2;

    // ·ÇÏßÐÔÐ£ÕýÏî£ºgamma / 2 * eta * (phi_m^2 - ||eta||^2)
    float error_term = FLUX_GAMMA_K * (FLUX_PHI_M_2 - eta_norm_sq);
    
    // ²ÉÓÃÅ·À­»ý·Ö¸üÐÂ¹Û²âÆ÷ÄÚ²¿×´Ì¬¡£
    obs->x1 += Ts * (y1 + error_term * obs->eta1);
    obs->x2 += Ts * (y2 + error_term * obs->eta2);
    
    // ===== 2. PLL ÌáÈ¡½Ç¶È =====
    // ÊäÈë£ºeta = phi_m * [cos(theta), sin(theta)]
    // Êä³ö£ºtheta_hat¡¢cos(theta_hat)¡¢sin(theta_hat)
    
    // ¼ÆËã½Ç¶ÈÎó²î£º
    // sin(theta - theta_hat) ¡Ö sin(theta) * cos(theta_hat) - cos(theta) * sin(theta_hat)
    float sin_theta_real = obs->eta2 * _1_FLUX_PHI_M;
    float cos_theta_real = obs->eta1 * _1_FLUX_PHI_M;
  
    // ²ÉÓÃÐ¡½Ç¶È½üËÆ×÷Îª PLL µÄÏàÎ»Îó²îÊäÈë¡£
    float angle_error = sin_theta_real * obs->cos_theta - cos_theta_real * obs->sin_theta;
    
    // PLL µÄ PI µ÷½ÚÆ÷£¬»ý·ÖÏî×öÏÞ·ù±ÜÃâ windup¡£
    obs->omega_integ += PLL_KI * angle_error * Ts;
    if (obs->omega_integ > MAX_SPEED) obs->omega_integ = MAX_SPEED;
    if (obs->omega_integ < -MAX_SPEED) obs->omega_integ = -MAX_SPEED;

    float omega_pll = obs->omega_integ + SMO_PLL_KP * angle_error;

    // ¶Ô PLL µÄÔ­Ê¼½ÇËÙ¶È¹À¼Æ×öµÍÍ¨£¬¼õÐ¡ËÙ¶È¶¶¶¯¡£
    obs->omega_filt = butterworth_filter(&omega_flux_filt, omega_pll);

    // ¶Ô½ÇËÙ¶È»ý·ÖµÃµ½¹Û²â½Ç¶È¡£
    obs->theta_hat += omega_pll * Ts;
    
    // ½Ç¶È¹éÒ»»¯µ½ -pi ~ pi¡£
    obs->theta_hat = angle_normalize_pi(obs->theta_hat);

    obs->theta = angle_normalize(obs->theta_hat);

    // ¹éÒ»»¯µ½ 0~360¡ã£¬ÓÃÓÚ arm_sin_cos_f32¡£
    float theta_deg = angle_normalize_360(rad2deg(obs->theta_hat));

    // ¸üÐÂÊä³öÕýÓàÏÒ¡£
    arm_sin_cos_f32(theta_deg, &obs->sin_theta, &obs->cos_theta);
}

/* ---------------------SMO »¬Ä£¹Û²âÆ÷---------------------- */

SMO_Observer_t g_smo_obs;

static float32_t smo_sat(float32_t err, float32_t band)
{
    // ±ß½ç²ã±¥ºÍº¯Êý£º
    // Îó²î½ÏÐ¡Ê±±£ÁôÏßÐÔÇø£¬½µµÍ¶¶Õñ£»
    // Îó²î½Ï´óÊ±ÍË»¯³É·ûºÅº¯Êý£¬¼Ó¿ìÊÕÁ²ËÙ¶È¡£
    if (band <= 1e-6f) {
        if (err > 0.0f) return 1.0f;
        if (err < 0.0f) return -1.0f;
        return 0.0f;
    }

    if (err > band) return 1.0f;
    if (err < -band) return -1.0f;
    return err / band;
}

static void smo_pll_update(SMO_Observer_t *obs)
{
    // ÕâÀï½«¹À¼Æ·´µç¶¯ÊÆÊ¸Á¿ËÍÈë PLL£¬ÌáÈ¡µç½Ç¶ÈºÍµç½ÇËÙ¶È¡£
    float cos_value = arm_cos_f32(obs->theta);
    float sin_value = arm_sin_f32(obs->theta);

    obs->err = obs->e_alpha * sin_value - obs->e_beta * cos_value;
    obs->omega_integ += obs->err * SMO_PLL_KI;

    if (obs->omega_integ > SMO_PLL_INT_LIMIT) obs->omega_integ = SMO_PLL_INT_LIMIT;
    if (obs->omega_integ < -SMO_PLL_INT_LIMIT) obs->omega_integ = -SMO_PLL_INT_LIMIT;

    obs->omega = obs->err * SMO_PLL_KP + obs->omega_integ;

    if (obs->omega > SMO_PLL_INT_LIMIT) obs->omega = SMO_PLL_INT_LIMIT;
    if (obs->omega < -SMO_PLL_INT_LIMIT) obs->omega = -SMO_PLL_INT_LIMIT;

    obs->theta += obs->omega * obs->ts;
    obs->theta = angle_normalize(obs->theta);
}

void SMO_Pare_init(void)
{
    memset(&g_smo_obs, 0, sizeof(g_smo_obs));

    // ´Ó¹¤³ÌÅäÖÃÖÐ¶ÁÈ¡µç»ú²ÎÊý£¬²¢ÀëÉ¢»¯µçÁ÷Ä£ÐÍ£º
    //   di/dt = (u - e - R*i - z) / L
    //   i(k+1) = F * i(k) + G * (u - e - z)
    g_smo_obs.rs = MOTOR_RESISTANCE;
    g_smo_obs.ls = MOTOR_INDUCTANCE;
    g_smo_obs.ts = PWM_PERIOD_S;

    g_smo_obs.fsmopos = 1.0f - (g_smo_obs.rs * g_smo_obs.ts / g_smo_obs.ls);
    g_smo_obs.gsmopos = g_smo_obs.ts / g_smo_obs.ls;

    // »¬Ä£ÔöÒæ Kslide ¾ö¶¨¹Û²âÆ÷¡°À­»ØÎó²î¡±µÄÁ¦¶È¡£
    // ¹ýÐ¡»á¸ú×ÙÎÞÁ¦£¬¹ý´óÔòÈÝÒ×´øÀ´¶¶ÕñºÍ·´µç¶¯ÊÆÏ÷¶¥¡£
    g_smo_obs.kslide = g_smo_obs.rs * SMO_SLIDE_GAIN_FACTOR;
    // ·´µç¶¯ÊÆµÍÍ¨ÏµÊý£¬ÓÃÓÚ´Ó¿ª¹ØÁ¿ z ÖÐÌáÈ¡Æ½»¬µÄ e_alpha/e_beta¡£
    g_smo_obs.kslf_emf = SMO_EMF_FILTER_COEFF;
    // ±ß½ç²ã¿í¶È£¬Ó°Ïì±¥ºÍº¯ÊýµÄÏßÐÔÇø´óÐ¡¡£
    g_smo_obs.e0 = SMO_CURRENT_ERR_BAND;
}

float32_t SMO_bemf_angle(AlphaBetaTypeDef *alpha_beta)
{
    // 1. ¸ù¾ÝÀëÉ¢µçÁ÷Ä£ÐÍ¸üÐÂ¹À¼ÆµçÁ÷¡£
    //    ÕâÀïµÄÊäÈëÁ¿¾ùÎ»ÓÚ alpha-beta ¾²Ö¹×ø±êÏµ¡£
    g_smo_obs.est_i_alpha = g_smo_obs.fsmopos * g_smo_obs.est_i_alpha +
                            g_smo_obs.gsmopos *
                            (alpha_beta->alpha_v - g_smo_obs.e_alpha - g_smo_obs.z_alpha);
    g_smo_obs.est_i_beta = g_smo_obs.fsmopos * g_smo_obs.est_i_beta +
                           g_smo_obs.gsmopos *
                           (alpha_beta->beta_v - g_smo_obs.e_beta - g_smo_obs.z_beta);

    // 2. ¹À¼ÆµçÁ÷Óë²ÉÑùµçÁ÷×÷²î£¬µÃµ½»¬Ä£ÃæÎó²î¡£
    g_smo_obs.i_alpha_error = g_smo_obs.est_i_alpha - alpha_beta->alpha_i;
    g_smo_obs.i_beta_error = g_smo_obs.est_i_beta - alpha_beta->beta_i;

    // 3. Í¨¹ý±¥ºÍº¯ÊýÉú³É»¬Ä£¿ª¹ØÁ¿¡£
    //    ÕâÊÇ SMO µÄºËÐÄÐ£ÕýÏî£¬¸ºÔð¿ìËÙÒÖÖÆµçÁ÷Ä£ÐÍÎó²î¡£
    g_smo_obs.z_alpha = g_smo_obs.kslide *
                        smo_sat(g_smo_obs.i_alpha_error, g_smo_obs.e0);
    g_smo_obs.z_beta = g_smo_obs.kslide *
                       smo_sat(g_smo_obs.i_beta_error, g_smo_obs.e0);

    // 4. ¶Ô¿ª¹ØÁ¿½øÐÐÒ»½×µÍÍ¨£¬ÌáÈ¡Æ½»¬·´µç¶¯ÊÆ¡£
    g_smo_obs.e_alpha += g_smo_obs.kslf_emf *
                         (g_smo_obs.z_alpha - g_smo_obs.e_alpha);
    g_smo_obs.e_beta += g_smo_obs.kslf_emf *
                        (g_smo_obs.z_beta - g_smo_obs.e_beta);

    // 5. ÓÃ·´µç¶¯ÊÆ½øÈë PLL£¬µÃµ½½Ç¶ÈºÍËÙ¶È¹À¼Æ¡£
    smo_pll_update(&g_smo_obs);
    g_smo_obs.theta_comp = angle_normalize(g_smo_obs.theta +
                                           SMO_ANGLE_COMPENSATION);

    return g_smo_obs.theta_comp;
}

#if HFI_ENABLE

/* ---------------------HFI ¹Û²âÆ÷---------------------- */
HFI_Observer_t g_hfi_obs;
HFI_PLL_t g_hfi_pll;

/**
 * @brief HFI ¹Û²âÆ÷³õÊ¼»¯
 * @param obs HFI ¹Û²âÆ÷½á¹¹ÌåÖ¸Õë
 * @param u_inject ×¢ÈëµçÑ¹·ùÖµ£¬µ¥Î» V
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
 * @brief HFI ¹Û²âÆ÷¸üÐÂ£¬ÌáÈ¡¸ßÆµµçÁ÷·ÖÁ¿
 * @param obs HFI ¹Û²âÆ÷½á¹¹ÌåÖ¸Õë
 * @param i_alpha alpha ÖáµçÁ÷
 * @param i_beta beta ÖáµçÁ÷
 * @note ½¨ÒéÔÚ PWM ÖÜÆÚ¿ªÊ¼´¦µ÷ÓÃ
 */
void hfi_observer_update(HFI_Observer_t *obs, float i_alpha, float i_beta)
{
    // 1. ÌáÈ¡¸ßÆµµçÁ÷·ÖÁ¿¡£
    obs->i_alpha_h = (i_alpha - 2.0f * obs->i_alpha_last[0] + obs->i_alpha_last[1]) * 0.25f;
    obs->i_beta_h  = (i_beta  - 2.0f * obs->i_beta_last[0]  + obs->i_beta_last[1])  * 0.25f;
    
    // 2. ¸üÐÂÀúÊ·²ÉÑùÖµ¡£
    obs->i_alpha_last[1] = obs->i_alpha_last[0];
    obs->i_alpha_last[0] = i_alpha;
    obs->i_beta_last[1] = obs->i_beta_last[0];
    obs->i_beta_last[0] = i_beta;
    
    // 3. ¼ÆËã×¢Èë¼«ÐÔ¡£ÕâÀïµÄ sign ¶ÔÓ¦ÉÏÒ»ÅÄµÄ×¢Èë·½Ïò¡£
    float sign = (obs->inject_flag == 0) ? 1.0f : -1.0f;
    
    // 4. ½âµ÷µÃµ½µçÁ÷°üÂç¡£
    obs->i_env_alpha = (obs->i_alpha_h - obs->i_alpha_h_last) * sign;
    obs->i_env_beta  = (obs->i_beta_h - obs->i_beta_h_last) * sign;

    // 5. ¸üÐÂ¸ßÆµÀúÊ·Á¿¡£
    obs->i_alpha_h_last = obs->i_alpha_h;
    obs->i_beta_h_last = obs->i_beta_h;

    // 6. ÇÐ»»ÏÂÒ»ÅÄ×¢Èë¼«ÐÔ¡£
    obs->inject_flag ^= 1;
}

/**
 * @brief æå–åŸºé?‘ç”µæµåˆ†é‡ï¼ˆç”¨äºŽç”µæµçŽ?åé?ˆï¼‰
 * @param obs HFIè§‚æµ‹å™¨ç»“æž„ä½“æŒ‡é’ˆ
 * @param i_d dè½´ç”µæµï¼ˆå?é«˜é?‘ï¼‰
 * @param i_q qè½´ç”µæµï¼ˆå?é«˜é?‘ï¼‰
 * @param i_d_fund è¾“å‡ºçš„dè½´åŸºé¢‘ç”µæµæŒ‡é’?
 * @param i_q_fund è¾“å‡ºçš„qè½´åŸºé¢‘ç”µæµæŒ‡é’?
 * @note ä½¿ç”¨æ»‘åŠ¨å¹³å‡æ»¤æ³¢æå–åŸºé?‘åˆ†é‡?
 */
void hfi_extract_fundamental(HFI_Observer_t *obs, float i_d, float i_q, 
                              float *i_d_fund, float *i_q_fund)
{
    // ä½¿ç”¨ä¸‰ç‚¹æ»‘åŠ¨å¹³å‡æå–åŸºé?‘åˆ†é‡?
    // i_fund = (i[n] + 2*i[n-1] + i[n-2]) / 4
    static float i_d_last[2] = {0.0f, 0.0f};
    static float i_q_last[2] = {0.0f, 0.0f};
    
    *i_d_fund = (i_d + 2.0f * i_d_last[0] + i_d_last[1]) * 0.25f;
    *i_q_fund = (i_q + 2.0f * i_q_last[0] + i_q_last[1]) * 0.25f;
    
    // ä¿å­˜åŽ†å²å€?
    i_d_last[1] = i_d_last[0];
    i_d_last[0] = i_d;
    i_q_last[1] = i_q_last[0];
    i_q_last[0] = i_q;
    
    // åŒæ—¶æ›´æ–°åˆ°ç»“æž„ä½“
    obs->i_d_fund = *i_d_fund;
    obs->i_q_fund = *i_q_fund;
}

/**
 * @brief èŽ·å–å½“å‰åº”æ³¨å…¥çš„ç”µåŽ‹
 * @param obs HFIè§‚æµ‹å™¨ç»“æž„ä½“æŒ‡é’ˆ
 * @return å½“å‰dè½´æ³¨å…¥ç”µåŽ‹ï¼ˆå?æžæ€§ï¼‰
 */
float hfi_get_inject_voltage(HFI_Observer_t *obs)
{
    // æ–¹æ³¢æ³¨å…¥ï¼šæ ¹æ®inject_flagå†³å®šæžæ€?
    obs->u_inject_d = (obs->inject_flag == 0) ? obs->u_inject : -obs->u_inject;
    return obs->u_inject_d;
}

/**
 * @brief HFI-PLLåˆå?‹åŒ–
 * @param pll PLLç»“æž„ä½“æŒ‡é’?
 * @param kp æ¯”ä¾‹å¢žç›Šï¼ˆå»ºè®?2.0~3.0ï¼?
 * @param ki ç§?åˆ†å?žç›Šï¼ˆå»ºè®?15~25ï¼?
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
 * @brief HFI-PLLæ›´æ–° - è·Ÿè¸ªè½?å­ä½ç½?
 * @param pll PLLç»“æž„ä½“æŒ‡é’?
 * @param obs HFIè§‚æµ‹å™¨ç»“æž„ä½“æŒ‡é’ˆï¼ˆä½¿ç”¨å…¶ä¸?çš„i_env_alpha/betaï¼?
 * @param Ts é‡‡æ ·å‘¨æœŸ(s)
 * @note åŸºäºŽç”µæµåŒ…ç»œçš„åæ­£åˆ‡è¿›è?Œä½ç½?è·Ÿè¸ª
 */
void hfi_pll_update(HFI_PLL_t *pll, HFI_Observer_t *obs, float Ts)
{
    static ButterworthLPF_t omega_filt;
    static uint8_t filt_inited = 0;
    
    // åˆå?‹åŒ–æ»¤æ³¢å™?
    if (!filt_inited) {
        butterworth_init(&omega_filt, 10.0f, PWM_FREQ);
        filt_inited = 1;
    }
    
    float sin_theta, cos_theta;
    
    // ================== 1. è§’åº¦å½’ä¸€åŒ–ï¼ˆ-Ï€ ~ Ï€ï¼?==================
    pll->theta_est = angle_normalize_pi(pll->theta_est);

    // ================== 2. è®¡ç®—ä¼°è?¡è?’åº¦çš? sin/cos ==================
    float angle_deg = pll->theta_est * RAD_TO_DEG;
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);
    
    // ================== 3. èŽ·å–è§£è°ƒåŽçš„ç”µæµåŒ…ç»œ ==================
    float i_alpha = obs->i_env_alpha;
    float i_beta = obs->i_env_beta;
    
    // ================== 4. æ­£äº¤è§£è°ƒè®¡ç®—è¯?å·? ==================
    pll->error = -i_beta * cos_theta + i_alpha * sin_theta; // è¿™é‡Œå–åï¼Œè?‚æµ‹å‡ºä¸ºç”µè?’åº¦ï¼?90åº?
    
    // ================== 5. PI æŽ§åˆ¶å™? ==================
    pll->p_term = pll->kp * pll->error;
    pll->i_term += pll->ki * pll->error;
    
    // ================== 6. ç§?åˆ†é™å¹? ==================
    const float INT_LIMIT = 500.0f;
    if (pll->i_term > INT_LIMIT) pll->i_term = INT_LIMIT;
    if (pll->i_term < -INT_LIMIT) pll->i_term = -INT_LIMIT;
    
    // ================== 7. è§’é€Ÿåº¦è®¡ç®—ï¼ˆåŽŸå§‹å€¼ï¼‰==================
    pll->omega_est = pll->p_term + pll->i_term;

    // ================== 8. å·´ç‰¹æ²ƒæ–¯ä½Žé€šæ»¤æ³?ï¼ˆå?¹é€Ÿåº¦æ»¤æ³¢ï¼?==================
    pll->omega_est_filt = butterworth_filter(&omega_filt, pll->omega_est);
    
    // ================== 9. è§’åº¦ç§?åˆ†ï¼ˆä½¿ç”¨æ»¤æ³¢åŽçš„é€Ÿåº¦ï¼?==================
    pll->theta_est += pll->omega_est * Ts;
}

/**
 * @brief NSD(æ­£è´Ÿè„‰å†²æ£€æµ?)æžæ€§è¾¨è¯?
 * @param obs HFIè§‚æµ‹å™¨ç»“æž„ä½“æŒ‡é’ˆ
 * @param pll PLLç»“æž„ä½“æŒ‡é’ˆï¼ˆç”¨äºŽä¿?æ­£åˆå§‹è?’åº¦ï¼?
 * @param i_d dè½´ç”µæµï¼ˆå?é«˜é?‘ï¼‰
 * @param i_q qè½´ç”µæµï¼ˆå?é«˜é?‘ï¼‰
 * @note åœ¨ç”µæœºå¯åŠ¨å‰æ‰§è?Œï¼Œé€šè¿‡æ¯”è¾ƒæ­£è´Ÿè„‰å†²å“åº”ç¡?å®šæžæ€?
 */
void hfi_nsd_check(HFI_Observer_t *obs, HFI_PLL_t *pll, float i_d, float i_q)
{ 
    if (obs->polarity_determined) return;
    obs->nsd_count++;

    #if HFI_NSD_ENABLE
    // æå–dè½´é«˜é¢‘ç”µæµ?
    static float i_d_last[2] = {0};
    float i_d_h = (i_d - 2.0f * i_d_last[0] + i_d_last[1]) * 0.25f;
    i_d_last[1] = i_d_last[0];
    i_d_last[0] = i_d;

    // æ ‡å‡†NSDåºåˆ—ï¼?+Id -> 0 -> -Id -> 0 -> æ¯”è¾ƒ
    // é˜¶æ??1: 0 - 200 (10ms) +1.5A 20khz
    if (obs->nsd_count < 200) {
        obs->nsd_target_d = 0.2f;
    }
    // é˜¶æ??2: 200 - 250 (10ms) é‡‡æ ·æ­?
    else if (obs->nsd_count < 250) {
        obs->sum_pos += fabsf(i_d_h);
        obs->nsd_target_d = 0.2f;
    }
    // é˜¶æ??3: 250 - 350 (10ms) æ–½åŠ  -1.5A
    else if (obs->nsd_count < 350) {
        obs->nsd_target_d = -0.2f;
    }
    // é˜¶æ??4: 350 - 400 (10ms) é‡‡æ ·è´?
    else if (obs->nsd_count < 400) {
        obs->sum_neg += fabsf(i_d_h);
        obs->nsd_target_d = -0.2f;
    }
    // é˜¶æ??5ï¼šåˆ¤æ–?ä¸Žä¿®æ­?
    else {
        debug_log("%.4f, %.4f", obs->sum_pos, obs->sum_neg);
        if (obs->sum_neg > obs->sum_pos) {
            pll->theta_est += PI;  // åè½¬180åº?
            pll->theta_est = angle_normalize_pi(pll->theta_est);
        }
        obs->polarity_determined = 1;
        obs->nsd_target_d = 0.0f;
    }
    #else
    // å¼ºæ‹‰åˆ°dè½´ï¼Œç„¶åŽç›´æŽ¥ç¡?å®šæžæ€?
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


