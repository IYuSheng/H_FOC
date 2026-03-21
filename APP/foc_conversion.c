#include "foc_conversion.h"

foc_control_t foc_ctrl;
SVPWM_t svpwm;
AlphaBetaTypeDef alpha_beta;
pi_t iq_pid;
pi_t id_pid;
pi_t speed_pid;
pi_t position_pid;

/* ===================== 巴特沃斯滤波器 ===================== */

/**
 * @brief 二阶巴特沃斯低通滤波器初始化
 * @param filt 滤波器结构体指针
 * @param cutoff_freq 截止频率(Hz)
 * @param sample_freq 采样频率(Hz)
 * 
 * 传递函数：H(s) = ωc? / (s? + √2·ωc·s + ωc?)
 */
void butterworth_init(ButterworthLPF_t *filt, float cutoff_freq, float sample_freq)
{
    filt->cutoff_freq = cutoff_freq;
    filt->sample_freq = sample_freq;
    
    // 预扭曲频率（双线性变换）
    float wc = 2.0f * PI * cutoff_freq;
    float T = 1.0f / sample_freq;
    float wa = (2.0f / T) * tanf(wc * T / 2.0f);  // 预扭曲后的角频率
    
    // 二阶巴特沃斯系数计算
    float sqrt2 = 1.41421356f;
    float Q = wa * wa;
    float D = 4.0f + 2.0f * sqrt2 * wa * T + Q * T * T;
    
    // 分子系数（归一化，使直流增益为1）
    filt->b0 = Q * T * T / D;
    filt->b1 = 2.0f * Q * T * T / D;
    filt->b2 = Q * T * T / D;
    
    // 分母系数（a0归一化为1，所以这里是-a1/D, -a2/D）
    filt->a1 = (2.0f * Q * T * T - 8.0f) / D;
    filt->a2 = (4.0f - 2.0f * sqrt2 * wa * T + Q * T * T) / D;
    
    // 清零历史值
    filt->x[0] = filt->x[1] = 0.0f;
    filt->y[0] = filt->y[1] = 0.0f;
}

/**
 * @brief 二阶巴特沃斯滤波器更新
 * @param filt 滤波器结构体指针
 * @param input 当前输入值
 * @return 滤波后的输出值
 * 
 * 差分方程：y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
 */
float butterworth_filter(ButterworthLPF_t *filt, float input)
{
    // 计算当前输出
    float output = filt->b0 * input 
                 + filt->b1 * filt->x[0] 
                 + filt->b2 * filt->x[1]
                 - filt->a1 * filt->y[0] 
                 - filt->a2 * filt->y[1];
    
    // 更新历史值（移位）
    filt->x[1] = filt->x[0];
    filt->x[0] = input;
    filt->y[1] = filt->y[0];
    filt->y[0] = output;
    
    return output;
}

/**
 * @brief 电角度归一化（映射到0~2π范围）
 * @param angle 输入电角度（rad，范围无限制）
 * @return 归一化后电角度（rad，0~2π）
 */
float32_t angle_normalize(float32_t angle)
{
    angle = fmodf(angle, _2PI);
    if (angle < 0.0f) angle += _2PI;
    return angle;
}

/**
 * @brief 电角度归一化（映射到-π~π范围）
 * @param angle 输入电角度（rad，范围无限制）
 * @return 归一化后电角度（rad，-π~π）
 */
float32_t angle_normalize_pi(float32_t angle)
{
    while (angle < -_PI)   angle += _2PI;
    while (angle >= _PI)  angle -= _2PI;
    return angle;
}

/**
 * @brief 电角度归一化（映射到0~360范围）
 * @param angle 输入电角度（°，范围无限制）
 * @return 归一化后电角度（°，0~360）
 */
float32_t angle_normalize_360(float32_t angle)
{
    while (angle < 0.0f)   angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

/**
 * @brief 角度制转弧度制
 * @param deg 输入角度（°）
 * @return 转换后弧度值（rad）
 */
float deg2rad(float deg)
{
    return deg * 0.017453292519943295f;  // π / 180
}

/**
 * @brief 弧度制转角度制
 * @param rad 输入弧度（rad）
 * @return 转换后角度值（°）
 */
float rad2deg(float rad)
{
    return rad * 57.29577951308232f;  // 180 / π
}

/**
 * @brief SVPWM通用扇区判断函数
 * @param u_alpha α轴目标电压（V）
 * @param u_beta  β轴目标电压（V）
 * @return 扇区编号（1~6，对应0~60°~360°）
 */
uint8_t svpwm_sector_calc(AlphaBetaTypeDef *alpha_beta)
{
    uint8_t sector, pos1, pos2, pos3 = 0;
    
    float32_t X = alpha_beta->beta;
    float32_t Y = (_SQRT3  * alpha_beta->alpha - alpha_beta->beta);
    float32_t Z = (-_SQRT3 * alpha_beta->alpha - alpha_beta->beta);
    
    pos1 = (X > 1e-6f) ? 1 : 0;
    pos2 = (Y > 1e-6f) ? 1 : 0;
    pos3 = (Z > 1e-6f) ? 1 : 0;

    sector = pos1 * 1 + pos2 * 2 + pos3 * 4;

    switch (sector)
    {
        case 3:  return 1;
        case 1:  return 2;
        case 5:  return 3;
        case 4:  return 4;
        case 6:  return 5;
        case 2:  return 6;
        default: return 1;
    }
}

/**
 * @brief SVPWM基本矢量作用时间计算
 * @param sector 当前扇区（1~6）
 * @param u_alpha α轴目标电压（V）
 * @param u_beta  β轴目标电压（V）
 * @param vdc 母线电压（V）
 * @param T1 第一个基本矢量作用时间（s）
 * @param T2 第二个基本矢量作用时间（s）
 * @param T0 零矢量总作用时间（s）
 */
void svpwm_calc_times(AlphaBetaTypeDef *alpha_beta, SVPWM_t *svpwm, float32_t vdc)
{
    // 1) 计算幅值平方
    float32_t alpha = alpha_beta->alpha;
    float32_t beta  = alpha_beta->beta;
    float32_t u_mag_sq = alpha * alpha + beta * beta;
    const float32_t u_max = _1_SQRT3 * vdc;
    const float32_t u_max_sq = u_max * u_max;

    // 2) 过调制处理
    if (u_mag_sq > u_max_sq && u_mag_sq > 1e-12f)
    {
        float32_t u_mag = sqrtf(u_mag_sq);
        float32_t scale = u_max / u_mag;
        alpha *= scale;
        beta  *= scale;
        alpha_beta->alpha = alpha;
        alpha_beta->beta  = beta;
    }

    // 3) 中间变量计算
    float32_t factor = FACTOR / vdc;
    float32_t half_factor = factor * 0.5f;
    float32_t X = beta * factor;
    float32_t Y = ( _SQRT3 * alpha + beta) * half_factor;
    float32_t Z = (beta -_SQRT3 * alpha) * half_factor;
    
    // 4. 按扇区计算T1和T2（基于X/Y/Z）
    switch (svpwm->sector)
    {
        case 1:  // 扇区1：V4(100) + V6(110) → T1=Y, T2=X 3
            svpwm->T1 = -Z;
            svpwm->T2 = X;
            break;
        case 2:  // 扇区2：V6(110) + V2(010) → T1=X, T2=-Z 1
            svpwm->T1 = Z;
            svpwm->T2 = Y;
            break;
        case 3:  // 扇区3：V2(010) + V3(011) → T1=-Z, T2=-Y 5
            svpwm->T1 = X;
            svpwm->T2 = -Y;
            break;
        case 4:  // 扇区4：V3(011) + V1(001) → T1=-Y, T2=-X 4
            svpwm->T1 = -X;
            svpwm->T2 = Z;
            break;
        case 5:  // 扇区5：V1(001) + V5(101) → T1=-X, T2=Z 6
            svpwm->T1 = -Y;
            svpwm->T2 = -Z;
            break;
        case 6:  // 扇区6：V5(101) + V4(100) → T1=Z, T2=Y 2
            svpwm->T1 = Y;
            svpwm->T2 = -X;
            break;
        default:
            svpwm->T1 = 0.0f;
            svpwm->T2 = 0.0f;
            break;
    }
    
    // 6. 计算零矢量时间（确保T0≥0，避免负数）
    svpwm->T0 = PWM_PERIOD_S - svpwm->T1 - svpwm->T2;
    if (svpwm->T0 < 0.0f) svpwm->T0 = 0.0f;
}

/**
 * @brief SVPWM计算三相导通时间并转换为比较值
 * @param sector 当前扇区 (1-6)
 * @param T1 基本矢量1作用时间
 * @param T2 基本矢量2作用时间
 * @param T0 零矢量作用时间
 * @param pwm_a A相比较值指针
 * @param pwm_b B相比较值指针
 * @param pwm_c C相比较值指针
 */
void svpwm_duty_calc(SVPWM_t *svpwm)
{
    float32_t Ta, Tb, Tc;                 // 三相桥臂导通时间（s）
    float32_t T0_half = svpwm->T0 / 2.0f;

    // 按扇区计算三相导通时间
    switch (svpwm->sector)
    {
        case 1:  // 扇区1
            Ta = T0_half + svpwm->T1 + svpwm->T2;
            Tb = T0_half + svpwm->T2;
            Tc = T0_half;
            break;
        case 2:  // 扇区2
            Ta = T0_half + svpwm->T2;
            Tb = T0_half + svpwm->T1 + svpwm->T2;
            Tc = T0_half;
            break;
        case 3:  // 扇区3
            Ta = T0_half;
            Tb = T0_half + svpwm->T1 + svpwm->T2;
            Tc = T0_half + svpwm->T2;
            break;
        case 4:  // 扇区4
            Ta = T0_half;
            Tb = T0_half + svpwm->T2;
            Tc = T0_half + svpwm->T1 + svpwm->T2;
            break;
        case 5:  // 扇区5
            Ta = T0_half + svpwm->T2;
            Tb = T0_half;
            Tc = T0_half + svpwm->T1 + svpwm->T2;
            break;
        case 6:  // 扇区6
            Ta = T0_half + svpwm->T1 + svpwm->T2;
            Tb = T0_half;
            Tc = T0_half + svpwm->T2;
            break;
        default:  // 异常扇区
            Ta = T0_half;
            Tb = T0_half;
            Tc = T0_half;
            break;
    }
    // 导通时间→比较值
    const float32_t k = PWM_FREQ_PERIOD;
    svpwm->pwm_a = Ta * k;
    svpwm->pwm_b = Tb * k;
    svpwm->pwm_c = Tc * k;
}

/**
 * @brief 反Park变换
 * @param d_ptr D轴输入指针
 * @param q_ptr Q轴输入指针
 * @param alpha_ptr Alpha轴输出指针
 * @param beta_ptr Beta轴输出指针
 * @param angle 电角度(角度, 0-360度)
 */
void inv_park_transform_f32(foc_control_t *foc_ctrl, AlphaBetaTypeDef *alpha_beta, float32_t sin_theta, float32_t cos_theta)
{
    // 反Park变换：将旋转DQ坐标系电压转换为静止αβ坐标系电压
    alpha_beta->alpha = foc_ctrl->out_d * cos_theta - foc_ctrl->out_q * sin_theta;
    alpha_beta->beta = foc_ctrl->out_d * sin_theta + foc_ctrl->out_q * cos_theta;
}

/**
 * @brief 将三相电压电流转换为αβ坐标系下的值
 * @param abc_i 三相电流指针
 * @param abc_v 三相电压指针
 * @param alpha_beta 输出的αβ轴值
 */
void clark_transform(void *abc_i, void *abc_v, AlphaBetaTypeDef *alpha_beta)
{
    foc_data_i *current_abc = (foc_data_i *)abc_i;
    
    // 标准幅值不变性Clarke变换
    // α = (2/3)*ia + (-1/3)*ib + (-1/3)*ic
    // β = 0*ia + (1/√3)*ib + (-1/√3)*ic
    const float32_t TWO_THIRD = 2.0f / 3.0f;
    const float32_t ONE_THIRD = 1.0f / 3.0f;
    const float32_t ONE_SQRT3 = 1.0f / sqrtf(3.0f);

    // 电流Clarke变换（a/b/c → α/β）
    alpha_beta->alpha_i = TWO_THIRD * current_abc->ia - ONE_THIRD * current_abc->ib - ONE_THIRD * current_abc->ic;
    alpha_beta->beta_i = ONE_SQRT3 * current_abc->ib - ONE_SQRT3 * current_abc->ic;
}

// ... existing code ...

/**
 * @brief 将αβ转换为dq坐标系下的电流值
 * @param current_abc αβ电流值
 * @param current_αβ 输出的dq轴电流值
 */
void park_transform(AlphaBetaTypeDef *alpha_beta, foc_control_t *foc_ctrl, float32_t sin_theta, float32_t cos_theta)
{
    // Park变换将两相静止坐标系转换为两相旋转坐标系
    arm_park_f32(alpha_beta->alpha_i, alpha_beta->beta_i, &foc_ctrl->abc_dq.current_d, &foc_ctrl->abc_dq.current_q, sin_theta, cos_theta);
}

/**
 * @brief 将三相电流转换为dq坐标系下的电流值
 * @param current_abc 三相电流值 (ia, ib, ic)
 * @param current_dq 输出的dq轴电流值 (id, iq)
 * @param angle 电角度(弧度)
 */
void abc_to_dq_current(void *current_abc_ptr, foc_control_t *foc_ctrl, float angle)
{
    foc_data_i *current_abc = (foc_data_i *)current_abc_ptr;

    float32_t alpha, beta;
    float32_t sin_val, cos_val;
    
    // 计算角度的正余弦值（需要将弧度转换为角度）
    float32_t angle_deg = angle * RAD_TO_DEG;
    arm_sin_cos_f32(angle_deg, &sin_val, &cos_val);
    
    // 使用DSP库的Clarke变换将三相电流转换为两相静止坐标系
    arm_clarke_f32(current_abc->ia, current_abc->ib, &alpha, &beta);
    
    // 使用DSP库的Park变换将两相静止坐标系转换为两相旋转坐标系
    arm_park_f32(alpha, beta, &foc_ctrl->abc_dq.current_d, &foc_ctrl->abc_dq.current_q, sin_val, cos_val);
}

/**
 * @brief D轴电流环PID计算
 * @param target_id 目标D轴电流
 * @param actual_id 实际D轴电流
 * @return D轴电压输出
 */
float32_t foc_id_pid_calculate(float32_t target_id, float32_t actual_id)
{
    float32_t error = target_id - actual_id;
    float32_t p_term = id_pid.kp * error;
    float32_t id_pid_output;
    
    // 积分项计算与限幅
    id_pid.integral += id_pid.ki * error;

    if (id_pid.integral > id_pid.integral_limit) {
        id_pid.integral = id_pid.integral_limit;
    } else if (id_pid.integral < -id_pid.integral_limit) {
        id_pid.integral = -id_pid.integral_limit;
    }

    id_pid_output = p_term + id_pid.integral;
    
    return id_pid_output;
}

/**
 * @brief Q轴电流环PID计算
 * @param target_iq 目标Q轴电流
 * @param actual_iq 实际Q轴电流
 * @return Q轴电压输出
 */
float32_t foc_iq_pid_calculate(float32_t target_iq, float32_t actual_iq)
{
    float32_t error = target_iq - actual_iq;
    float32_t p_term = iq_pid.kp * error;
    float32_t iq_pid_output;
    
    // 积分项计算与限幅
    iq_pid.integral += iq_pid.ki * error;

    if (iq_pid.integral > iq_pid.integral_limit) {
        iq_pid.integral = iq_pid.integral_limit;
    } else if (iq_pid.integral < -iq_pid.integral_limit) {
        iq_pid.integral = -iq_pid.integral_limit;
    }

    iq_pid_output = p_term + iq_pid.integral;
    
    return iq_pid_output;
}

/**
 * @brief 速度环PID计算
 * @param target_speed 目标速度(RPM)
 * @param actual_speed 实际速度(RPM)
 * @return 输出值(Q轴电流)
 */
float32_t foc_speed_pid_calculate(float32_t target_speed, float32_t actual_speed)
{
    float32_t error = target_speed - actual_speed;
    float32_t p_term = speed_pid.kp * error;
    
    // 积分项计算与限幅
    speed_pid.integral += speed_pid.ki * error;
    if (speed_pid.integral > speed_pid.integral_limit) {
        speed_pid.integral = speed_pid.integral_limit;
    } else if (speed_pid.integral < -speed_pid.integral_limit) {
        speed_pid.integral = -speed_pid.integral_limit;
    }
    
    return p_term + speed_pid.integral;
}

/**
 * @brief 位置环PI控制器计算
 * @param target_position 目标位置（弧度）
 * @param current_position 当前位置（弧度）
 * @return PI控制器输出（速度指令，rad/s）
 */
float32_t foc_position_pid_calculate(float32_t target_position, float32_t current_position)
{
    float32_t error, p_term;
    
    position_pid.target = target_position;
    position_pid.current = current_position;
    
    // 计算误差（目标 - 当前）
    error = target_position - current_position;
    if(fabsf(error) < 0.01f)
    {
        // 在死区内，清零输出和积分
        position_pid.integral = 0.0f;
        error = 0.0f; // 死区处理
    }
    position_pid.error = error;
    
    // 比例项
    p_term = position_pid.kp * error;
    
    // 积分项累加
    position_pid.integral += position_pid.ki * error;
    
    // 积分限幅
    if (position_pid.integral > position_pid.integral_limit) {
        position_pid.integral = position_pid.integral_limit;
    } else if (position_pid.integral < -position_pid.integral_limit) {
        position_pid.integral = -position_pid.integral_limit;
    }
    
    // 输出 = 比例项 + 积分项
    position_pid.output = p_term + position_pid.integral;
    
    return position_pid.output;
}
