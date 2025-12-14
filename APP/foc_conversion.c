#include "foc_conversion.h"

pi_t iq_pid;
pi_t id_pid;
pi_t speed_pid;
pi_t position_pid;

/**
 * @brief 电角度归一化（映射到0~2π范围）
 * @param angle 输入电角度（rad，范围无限制）
 * @return 归一化后电角度（rad，0~2π）
 */
inline float32_t angle_normalize(float32_t angle)
{
    angle = fmodf(angle, _2PI);
    if (angle < 0.0f) angle += _2PI;
    return angle;
}

/**
 * @brief SVPWM通用扇区判断函数
 * @param u_alpha α轴目标电压（V）
 * @param u_beta  β轴目标电压（V）
 * @return 扇区编号（1~6，对应0~60°~360°）
 */
inline uint8_t svpwm_sector_calc(AlphaBetaTypeDef *alpha_beta)
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
inline void svpwm_calc_times(AlphaBetaTypeDef *alpha_beta, SVPWM_t *svpwm, float32_t vdc)
{
    // 1. 计算目标电压矢量幅值（避免过调制）
    float32_t u_mag;
    float32_t alpha_sq, beta_sq, sum_sq;
    const float32_t u_max = _1_SQRT3 * vdc; // SVPWM最大输出相电压幅值
    
    // 使用DSP函数计算平方和开方
    arm_mult_f32(&alpha_beta->alpha, &alpha_beta->alpha, &alpha_sq, 1);
    arm_mult_f32(&alpha_beta->beta, &alpha_beta->beta, &beta_sq, 1);
    arm_add_f32(&alpha_sq, &beta_sq, &sum_sq, 1);
    arm_sqrt_f32(sum_sq, &u_mag);
    
    // 2. 过调制处理
    if (u_mag > u_max && u_mag > 1e-6f)
    {
        float32_t scale = u_max / u_mag;
        arm_scale_f32(&alpha_beta->alpha, scale, &alpha_beta->alpha, 1);
        arm_scale_f32(&alpha_beta->beta, scale, &alpha_beta->beta, 1);
        u_mag = u_max;
    }
    
    // 3. 计算与扇区判断完全一致的中间变量
    float32_t factor = FACTOR / vdc;
    float32_t half_factor = factor * 0.5f;
    float32_t X, Y, Z;
    
    // X
    arm_scale_f32(&alpha_beta->beta, factor, &X, 1);
    // Y
    float32_t temp1, temp2;
    float32_t sqrt3_const = _SQRT3;
    arm_scale_f32(&sqrt3_const, alpha_beta->alpha, &temp1, 1);
    arm_add_f32(&temp1, &alpha_beta->beta, &temp2, 1);
    arm_scale_f32(&temp2, half_factor, &Y, 1);
    // Z
    arm_scale_f32(&sqrt3_const, alpha_beta->alpha, &temp1, 1);
    arm_sub_f32(&temp1, &alpha_beta->beta, &temp2, 1);
    arm_scale_f32(&temp2, half_factor, &Z, 1);
    arm_negate_f32(&Z, &Z, 1); // 取负值
    
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
inline void svpwm_duty_calc(SVPWM_t *svpwm)
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
    svpwm->pwm_a = Ta * PWM_FREQ_PERIOD;
    svpwm->pwm_b = Tb * PWM_FREQ_PERIOD;
    svpwm->pwm_c = Tc * PWM_FREQ_PERIOD;
}

/**
 * @brief 反Park变换
 * @param d_ptr D轴输入指针
 * @param q_ptr Q轴输入指针
 * @param alpha_ptr Alpha轴输出指针
 * @param beta_ptr Beta轴输出指针
 * @param angle 电角度(弧度)
 */
inline void inv_park_transform_f32(foc_control_t *foc_ctrl, AlphaBetaTypeDef *alpha_beta, float32_t angle)
{
    float32_t sin_theta, cos_theta;
    
    // 将电角度从弧度转换为角度
    float32_t angle_deg;
    arm_scale_f32(&angle, RAD_TO_DEG, &angle_deg, 1);
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

    // 反Park变换：将旋转DQ坐标系电压转换为静止αβ坐标系电压
    alpha_beta->alpha = foc_ctrl->out_d * cos_theta - foc_ctrl->out_q * sin_theta;
    alpha_beta->beta = foc_ctrl->out_d * sin_theta + foc_ctrl->out_q * cos_theta;
}

// ... existing code ...

/**
 * @brief 将三相电压电流转换为αβ坐标系下的值
 * @param abc_i 三相电流指针
 * @param abc_v 三相电压指针
 * @param alpha_beta 输出的αβ轴值
 */
inline void clark_transform(void *abc_i, void *abc_v, AlphaBetaTypeDef *alpha_beta)
{
    foc_data_i *current_abc = (foc_data_i *)abc_i;
    foc_data_v *voltage_abc = (foc_data_v *)abc_v;
    
    // 标准幅值不变性Clarke变换
    // α = (2/3)*ia + (-1/3)*ib + (-1/3)*ic
    // β = 0*ia + (1/√3)*ib + (-1/√3)*ic
    const float32_t TWO_THIRD = 2.0f / 3.0f;
    const float32_t ONE_THIRD = 1.0f / 3.0f;
    const float32_t ONE_SQRT3 = 1.0f / sqrtf(3.0f);

    // 电流Clarke变换（a/b/c → α/β）
    alpha_beta->alpha_i = TWO_THIRD * current_abc->ia - ONE_THIRD * current_abc->ib - ONE_THIRD * current_abc->ic;
    alpha_beta->beta_i = ONE_SQRT3 * current_abc->ib - ONE_SQRT3 * current_abc->ic;
    
    // 电压Clarke变换（a/b/c → α/β）
    alpha_beta->alpha_v = TWO_THIRD * voltage_abc->va - ONE_THIRD * voltage_abc->vb - ONE_THIRD * voltage_abc->vc;
    alpha_beta->beta_v = ONE_SQRT3 * voltage_abc->vb - ONE_SQRT3 * voltage_abc->vc;
}

// ... existing code ...

/**
 * @brief 将αβ转换为dq坐标系下的电流值
 * @param current_abc αβ电流值
 * @param current_αβ 输出的dq轴电流值
 */
inline void park_transform(AlphaBetaTypeDef *alpha_beta, foc_control_t *foc_ctrl)
{
    float32_t sin_val, cos_val;
    // 计算角度的正余弦值（需要将弧度转换为角度）
    float32_t angle_deg = foc_ctrl->angle * RAD_TO_DEG;
    arm_sin_cos_f32(angle_deg, &sin_val, &cos_val);

    // 使用DSP库的Park变换将两相静止坐标系转换为两相旋转坐标系
    arm_park_f32(alpha_beta->alpha_i, alpha_beta->beta_i, &foc_ctrl->abc_dq.current_d, &foc_ctrl->abc_dq.current_q, sin_val, cos_val);
}

/**
 * @brief 将三相电流转换为dq坐标系下的电流值
 * @param current_abc 三相电流值 (ia, ib, ic)
 * @param current_dq 输出的dq轴电流值 (id, iq)
 * @param angle 电角度(弧度)
 */
inline void abc_to_dq_current(void *current_abc_ptr, foc_control_t *foc_ctrl, float angle)
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
inline float32_t foc_id_pid_calculate(float32_t target_id, float32_t actual_id)
{
    float32_t error = target_id - actual_id;
    float32_t p_term = id_pid.kp * error;
    
    // 积分项计算与限幅
    id_pid.integral += id_pid.ki * error;
    if (id_pid.integral > id_pid.integral_limit) {
        id_pid.integral = id_pid.integral_limit;
    } else if (id_pid.integral < -id_pid.integral_limit) {
        id_pid.integral = -id_pid.integral_limit;
    }
    
    return p_term + id_pid.integral;
}

/**
 * @brief Q轴电流环PID计算
 * @param target_iq 目标Q轴电流
 * @param actual_iq 实际Q轴电流
 * @return Q轴电压输出
 */
inline float32_t foc_iq_pid_calculate(float32_t target_iq, float32_t actual_iq)
{
    float32_t error = target_iq - actual_iq;
    float32_t p_term = iq_pid.kp * error;
    
    // 积分项计算与限幅
    iq_pid.integral += iq_pid.ki * error;
    if (iq_pid.integral > iq_pid.integral_limit) {
        iq_pid.integral = iq_pid.integral_limit;
    } else if (iq_pid.integral < -iq_pid.integral_limit) {
        iq_pid.integral = -iq_pid.integral_limit;
    }
    
    return p_term + iq_pid.integral;
}

/**
 * @brief 速度环PID计算
 * @param target_speed 目标速度(RPM)
 * @param actual_speed 实际速度(RPM)
 * @return 输出值(Q轴电流)
 */
inline float32_t foc_speed_pid_calculate(float32_t target_speed, float32_t actual_speed)
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
inline float32_t foc_position_pid_calculate(float32_t target_position, float32_t current_position)
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

/*----------------------------------------------------无感部分-------------------------------------------------------*/

// 全局变量声明
SMO_MotorPare_t SMO_MotorPare;
Ppll_obj_t Angle_SMOPare;

// 电机参数与控制参数初始化（同步删除冗余参数的初始化）
void SMO_Pare_init(void)  
{
    // 1. 电机基础参数初始化（保留）
    SMO_MotorPare.Rs = MOTOR_RESISTANCE;
    SMO_MotorPare.Ls = MOTOR_INDUCTANCE;
    SMO_MotorPare.Ts = PWM_PERIOD_S;
    SMO_MotorPare.POLES = MOTOR_POLE_PAIRS;

    // 2. 滑模观测器精确离散化系数计算
    SMO_MotorPare.Fsmopos = 1.0f - (SMO_MotorPare.Rs * SMO_MotorPare.Ts / SMO_MotorPare.Ls);
    // 精确离散化的Gsmopos公式
    SMO_MotorPare.Gsmopos = SMO_MotorPare.Ts / SMO_MotorPare.Ls;

    // 3. SMO控制参数初始化
    Angle_SMOPare.Kslide = SMO_MotorPare.Rs * 1.5f;      // 滑模增益
    Angle_SMOPare.Kslf_emf = 0.1f;    // 反电动势滤波系数
    Angle_SMOPare.E0 = 0.5f;          // 电流误差饱和阈值
    // 初始化估算电流、反电动势、控制量为0
    Angle_SMOPare.EstIalpha = 0.0f;
    Angle_SMOPare.EstIbeta = 0.0f;
    Angle_SMOPare.Ealpha = 0.0f;
    Angle_SMOPare.Ebeta = 0.0f;
    Angle_SMOPare.Zalpha = 0.0f;
    Angle_SMOPare.Zbeta = 0.0f;

    // 4. PLL参数初始化（保留）
    Angle_SMOPare.tPll.Kp = 150.0f;
    Angle_SMOPare.tPll.Ki = 100.0f;
    Angle_SMOPare.tPll.Speed_coeff = (60.0f)/(2*SMO_MotorPare.POLES*PI);
    Angle_SMOPare.tPll.Kslf = 0.5f;
}

/**
 * @brief PLL角度和速度计算
 * @param ptHandle PLL对象指针
 * @param Coff_Sine 反电动势α轴分量
 * @param Coff_Cos 反电动势β轴分量
 */
static void Pll_Compute(Ppll_obj_t *ptHandle, float Coff_Sine, float Coff_Cos)
{
    // 1. 计算估算角度的正余弦
    float Cos_Value = arm_cos_f32(ptHandle->Theta);
    float Sin_Value = arm_sin_f32(ptHandle->Theta);
    
    // 2. 计算相位误差
    ptHandle->Err = Coff_Sine * Sin_Value - Coff_Cos * Cos_Value;

    // // 4. PI积分项累加
    ptHandle->Interg += ptHandle->Err * ptHandle->tPll.Ki;
  
    // 5. PI控制器输出（电角速度Ui）
    ptHandle->Ui = ptHandle->Err * ptHandle->tPll.Kp + ptHandle->Interg;

    // 6. 电角度更新（累加角度增量）
    ptHandle->Theta += ptHandle->Ui * SMO_MotorPare.Ts;
    // 积分过零重置
    if(ptHandle->Theta > _2PI || ptHandle->Theta < -_2PI)
    {
        ptHandle->Interg = 0.0f;
    }

    // 7. 角度归一化（确保在0~2π范围）
    if(ptHandle->Theta < 0) {
        ptHandle->Theta += _2PI;
    } else if(ptHandle->Theta >= _2PI) {
        ptHandle->Theta -= _2PI;
    }
    
    // 8. 转速计算（机械转速RPM）
    ptHandle->Speed_Rpm = ptHandle->tPll.Speed_coeff * ptHandle->Ui;
 
    // 9. 转速低通滤波（平滑转速输出）
    ptHandle->SpeedLpf_Rpm = ptHandle->SpeedLpf_Rpm + ptHandle->tPll.Kslf * (ptHandle->Speed_Rpm - ptHandle->SpeedLpf_Rpm);
}

/**
 * @brief 使用反正切函数提取电角度（不使用PLL）
 * @param Ealpha 反电动势α轴分量
 * @param Ebeta 反电动势β轴分量
 * @return 估算的电角度(弧度，范围0~2π)
 */
inline float32_t atan_bemf_angle(float32_t Ealpha, float32_t Ebeta)
{
    float32_t angle;
    
    // 使用atan2函数计算角度，结果范围为[-π, π]
    angle = atan2f(Ebeta, Ealpha);
    
    // 将角度范围转换为[0, 2π]
    if (angle < 0.0f) {
        angle += _2PI;
    }
    
    return angle;
}

/**
 * @brief SMO扩展反电动势估算角度计算
 * @param alpha_beta alpha_beta坐标系下的电压电流值指针
 * @return 估算的电角度(弧度)
 */
inline float32_t SMO_bemf_angle(AlphaBetaTypeDef *alpha_beta)
{
    // -------------------------- 滑模核心逻辑 --------------------------
    // 1. 精确离散化电流估计
    Angle_SMOPare.EstIalpha = SMO_MotorPare.Fsmopos * Angle_SMOPare.EstIalpha +
                              SMO_MotorPare.Gsmopos * (alpha_beta->alpha_v - Angle_SMOPare.Ealpha - Angle_SMOPare.Zalpha);
    Angle_SMOPare.EstIbeta = SMO_MotorPare.Fsmopos * Angle_SMOPare.EstIbeta +
                             SMO_MotorPare.Gsmopos * (alpha_beta->beta_v - Angle_SMOPare.Ebeta - Angle_SMOPare.Zbeta);

    // 2. 电流误差计算（估算电流 - 实际电流）
    Angle_SMOPare.IalphaError = Angle_SMOPare.EstIalpha - alpha_beta->alpha_i;
    Angle_SMOPare.IbetaError = Angle_SMOPare.EstIbeta - alpha_beta->beta_i;

    // 3. 改进的滑模控制量Z计算（使用更平滑的符号函数）
    // 使用连续符号函数替代离散符号函数，减少抖振
    float32_t alpha_err_abs = fabsf(Angle_SMOPare.IalphaError);
    float32_t beta_err_abs = fabsf(Angle_SMOPare.IbetaError);
    
    // 避免除零情况
    if (alpha_err_abs < 1e-6f) alpha_err_abs = 1e-6f;
    if (beta_err_abs < 1e-6f) beta_err_abs = 1e-6f;
    
    Angle_SMOPare.Zalpha = Angle_SMOPare.Kslide * (Angle_SMOPare.IalphaError / alpha_err_abs);
    Angle_SMOPare.Zbeta = Angle_SMOPare.Kslide * (Angle_SMOPare.IbetaError / beta_err_abs);

    // 4. 改进的反电动势估算（增加滤波效果）
    // 使用更平滑的低通滤波
    Angle_SMOPare.Ealpha = Angle_SMOPare.Ealpha + Angle_SMOPare.Kslf_emf * (Angle_SMOPare.Zalpha - Angle_SMOPare.Ealpha);
    Angle_SMOPare.Ebeta = Angle_SMOPare.Ebeta + Angle_SMOPare.Kslf_emf * (Angle_SMOPare.Zbeta - Angle_SMOPare.Ebeta);

    // 4. 反电动势估算
    Angle_SMOPare.Ealpha += Angle_SMOPare.Kslf_emf * (Angle_SMOPare.Zalpha - Angle_SMOPare.Ealpha);
    Angle_SMOPare.Ebeta += Angle_SMOPare.Kslf_emf * (Angle_SMOPare.Zbeta - Angle_SMOPare.Ebeta);
    // -----------------------------------------------------------------------------------

    // 5. 角度和速度估算
    Pll_Compute(&Angle_SMOPare, Angle_SMOPare.Ealpha, Angle_SMOPare.Ebeta);
    // Angle_SMOPare.Theta = atan_bemf_angle(Angle_SMOPare.Ealpha, Angle_SMOPare.Ebeta);

    // 6. 添加角度补偿
    #define ANGLE_COMPENSATION 0.0f  // 根据实际测试调整补偿值（弧度）
    Angle_SMOPare.Theta_pre = Angle_SMOPare.Theta + ANGLE_COMPENSATION;
    
    // 7. 角度归一化
    if(Angle_SMOPare.Theta_pre < 0) {
        Angle_SMOPare.Theta_pre += _2PI;
    } else if(Angle_SMOPare.Theta_pre >= _2PI) {
        Angle_SMOPare.Theta_pre -= _2PI;
    }

    return Angle_SMOPare.Theta_pre;
}

/*----------------------------------------------------电机参数辨识部分-------------------------------------------------------*/

// 电机参数辨识结构体
typedef struct {
    // 辨识结果
    float32_t Rs;                // 相电阻 (Ω)
    float32_t Ls;                // 相电感 (H) - 取Ld和Lq的平均值
    float32_t Ld;                // D轴电感 (H)
    float32_t Lq;                // Q轴电感 (H)
    
    // 辨识状态
    ParamIdentState_t state;     // 当前辨识状态
    
    // 测试参数
    float32_t dc_test_voltage;   // 直流测试电压 (V)
    float32_t hf_freq;           // 高频测试频率 (Hz) - 建议500Hz
    float32_t hf_voltage;        // 高频测试电压幅值 (V)
    
    // 采样数据
    uint16_t sample_idx;         // 采样索引
    uint16_t sample_count;       // 总采样数（2个周期=40点）
    
    // 时间控制
    uint32_t start_tick;         // 当前测试开始时间（单位：中断计数）
    uint32_t elapsed_ticks;      // 已过去时间（单位：中断计数）
    
    // 统计信息
    float32_t v_sum_sq;          // 电压平方和（用于RMS计算）
    float32_t i_sum_sq;          // 电流平方和（用于RMS计算）
    float32_t v_max;             // 电压最大值（绝对值）
    float32_t i_max;             // 电流最大值（绝对值）
} MotorParamIdent_t;

MotorParamIdent_t g_param_id;

/**
 * @brief 初始化电机参数辨识
 * @param dc_voltage 直流测试电压(V)
 * @param hf_freq 高频测试频率(Hz)
 * @param hf_voltage 高频测试电压幅值(V)
 */
static void MotorParamIdent_Init(float32_t dc_voltage, float32_t hf_freq, float32_t hf_voltage)
{
    g_param_id.Rs = 0.0f;
    g_param_id.Ls = 0.0f;
    g_param_id.Ld = 0.0f;
    g_param_id.Lq = 0.0f;
    g_param_id.state = PARAM_ID_IDLE;
    
    g_param_id.dc_test_voltage = dc_voltage;
    g_param_id.hf_freq = hf_freq;
    g_param_id.hf_voltage = hf_voltage;
    
    g_param_id.sample_idx = 0;
    g_param_id.sample_count = (uint16_t)(PWM_FREQ / hf_freq * 5.0f); // 5个周期的采样点数
    
    // 初始化统计信息
    g_param_id.v_sum_sq = 0.0f;
    g_param_id.i_sum_sq = 0.0f;
    g_param_id.v_max = 0.0f;
    g_param_id.i_max = 0.0f;
}

/**
 * @brief 电机参数辨识
 */
void StartMotorParameterIdentification(void)
{
    // 初始化辨识参数
    // 直流电压：1V（确保电流不过大）
    // 高频频率：600Hz（感抗占主导）
    // 高频电压：0.5V（获得可测量的电流）
    MotorParamIdent_Init(1.0f, 600.0f, 0.5f);
}

/**
 * @brief 执行一步电机参数辨识
 * @param current_tick 当前中断计数（20kHz递增）
 */
ParamIdentState_t MotorParamIdent_Step(uint32_t current_tick, AlphaBetaTypeDef *alpha_beta, foc_control_t *foc_ctrl)
{
    // 先将采集到的电流电压转换为alpha-beta坐标系
    clark_transform(&foc_datai,&foc_datav, alpha_beta);
    park_transform(alpha_beta, foc_ctrl);
    
    switch(g_param_id.state)
    {
        case PARAM_ID_IDLE:
            g_param_id.state = PARAM_ID_RS_DC_TEST;
            g_param_id.start_tick = current_tick;
            g_param_id.sample_idx = 0;
            break;
            
        case PARAM_ID_RS_DC_TEST:   //辨识电阻
        {
            // 1. 注入直流电压到D轴
            foc_ctrl->angle = 0.0f;           // 角度设为0，D轴对齐α轴
            foc_ctrl->out_d = g_param_id.dc_test_voltage;  // D轴注入直流电压
            foc_ctrl->out_q = 0.0f;           // Q轴电压为0
            
            // 2. 等待电流稳定（2000个中断=100ms）
            g_param_id.elapsed_ticks = current_tick - g_param_id.start_tick;
            
            if(g_param_id.elapsed_ticks > PWM_FREQ / 2)  // 500ms稳定时间
            {
                // 采样阶段：采集100个样本
                if (g_param_id.sample_idx < 100)
                {
                    // 采集电流值
                    clark_transform(&foc_datai,&foc_datav, alpha_beta);
                    float32_t current_i = alpha_beta->alpha_i;
                    
                    // 累积采样值
                    g_param_id.i_sum_sq += current_i;
                    g_param_id.sample_idx++;
                }
                else
                {
                    // 采样完成，计算平均电流和电阻
                    float32_t avg_current = g_param_id.i_sum_sq / 100.0f;
                    
                    // 计算电阻：R = V / I
                    g_param_id.Rs = g_param_id.dc_test_voltage / avg_current;
                    
                    // 进入下一个状态
                    g_param_id.state = PARAM_ID_LD_HF_TEST;
                    g_param_id.start_tick = current_tick;
                    g_param_id.elapsed_ticks = 0;
                    g_param_id.sample_idx = 0;
                    g_param_id.v_sum_sq = 0.0f;
                    g_param_id.i_sum_sq = 0.0f;
                    g_param_id.v_max = 0.0f;
                    g_param_id.i_max = 0.0f;
                }
            }
        }
        break;
        
        case PARAM_ID_LD_HF_TEST:   // 辨识Ld
        {
            // 1. 注入高频正弦电压到D轴
            foc_ctrl->angle = 0.0f;           // D轴对齐α轴
            float32_t omega = _2PI * g_param_id.hf_freq;
            uint32_t hf_ticks = current_tick - g_param_id.start_tick;
            float32_t hf_time = (float32_t)hf_ticks / PWM_FREQ;
            
            // 生成高频正弦信号
            float32_t v_d = g_param_id.hf_voltage * sinf(omega * hf_time);
            foc_ctrl->out_d = v_d;
            foc_ctrl->out_q = 0.0f;
            
            // 2. 7个周期数据，跳过前2个周期,一共采集5个周期
            uint32_t skip_ticks = (uint32_t)(PWM_FREQ / g_param_id.hf_freq * 2.0f); // 跳过前2个周期
            if(hf_ticks >= skip_ticks && (hf_ticks - skip_ticks) < g_param_id.sample_count)
            {
                // 采样电压和电流（d轴）
                float32_t v_sample = v_d;
                float32_t i_sample = foc_ctrl->abc_dq.current_d;
                
                // 更新统计信息
                g_param_id.v_sum_sq += v_sample * v_sample;
                g_param_id.i_sum_sq += i_sample * i_sample;
                
                float32_t v_abs = fabsf(v_sample);
                float32_t i_abs = fabsf(i_sample);
                if(v_abs > g_param_id.v_max) g_param_id.v_max = v_abs;
                if(i_abs > g_param_id.i_max) g_param_id.i_max = i_abs;
                
                g_param_id.sample_idx++;
            }
            else if(hf_ticks >= skip_ticks && (hf_ticks - skip_ticks) >= g_param_id.sample_count)
            {
                // 采样完成，计算D轴电感
                if(g_param_id.sample_idx > 0)
                {
                    // 计算RMS值
                    float32_t v_rms = sqrtf(g_param_id.v_sum_sq / g_param_id.sample_idx);
                    float32_t i_rms = sqrtf(g_param_id.i_sum_sq / g_param_id.sample_idx);
                    
                    float32_t Z = v_rms / i_rms;           // 阻抗幅值
                    float32_t omega_L = _2PI * g_param_id.hf_freq;
                    
                    // L = sqrt(Z^2 - R^2) / ω
                    float32_t Z_sq = Z * Z;
                    float32_t R_sq = g_param_id.Rs * g_param_id.Rs;
                        
                    g_param_id.Ld = sqrtf(Z_sq - R_sq) / omega_L;
                }
                
                // 进入下一个状态
                g_param_id.state = PARAM_ID_LQ_HF_TEST;
                g_param_id.start_tick = current_tick;
                g_param_id.sample_idx = 0;
                g_param_id.v_sum_sq = 0.0f;
                g_param_id.i_sum_sq = 0.0f;
                g_param_id.v_max = 0.0f;
                g_param_id.i_max = 0.0f;
            }
        }
        break;
        
        case PARAM_ID_LQ_HF_TEST:   // 辨识Lq
        {
            // 1. 注入高频正弦电压到Q轴
            foc_ctrl->angle = _PI_2;      // Q轴对齐β轴
            float32_t omega = _2PI * g_param_id.hf_freq;
            uint32_t hf_ticks = current_tick - g_param_id.start_tick;
            float32_t hf_time = (float32_t)hf_ticks / PWM_FREQ;
            
            // 生成高频正弦信号
            float32_t v_q = g_param_id.hf_voltage * sinf(omega * hf_time);
            foc_ctrl->out_d = 0.0f;
            foc_ctrl->out_q = v_q;
            
            // 2. 7个周期数据，跳过前2个周期,一共采集5个周期
            uint32_t skip_ticks = (uint32_t)(PWM_FREQ / g_param_id.hf_freq * 2.0f); // 跳过前2个周期
            if(hf_ticks >= skip_ticks && (hf_ticks - skip_ticks) < g_param_id.sample_count)
            {
                // 采样电压和电流（β轴）
                float32_t v_sample = v_q;
                float32_t i_sample = foc_ctrl->abc_dq.current_q;
                
                // 更新统计信息
                g_param_id.v_sum_sq += v_sample * v_sample;
                g_param_id.i_sum_sq += i_sample * i_sample;
                
                float32_t v_abs = fabsf(v_sample);
                float32_t i_abs = fabsf(i_sample);
                if(v_abs > g_param_id.v_max) g_param_id.v_max = v_abs;
                if(i_abs > g_param_id.i_max) g_param_id.i_max = i_abs;
                
                g_param_id.sample_idx++;
            }
            else if(hf_ticks >= skip_ticks && (hf_ticks - skip_ticks) >= g_param_id.sample_count)
            {
                // 采样完成，计算Q轴电感
                if(g_param_id.sample_idx > 0)
                {
                    // 计算RMS值
                    float32_t v_rms = sqrtf(g_param_id.v_sum_sq / g_param_id.sample_idx);
                    float32_t i_rms = sqrtf(g_param_id.i_sum_sq / g_param_id.sample_idx);
                    
                    float32_t Z = v_rms / i_rms;           // 阻抗幅值
                    float32_t omega_L = _2PI * g_param_id.hf_freq;
                        
                    // L = sqrt(Z^2 - R^2) / ω
                    float32_t Z_sq = Z * Z;
                    float32_t R_sq = g_param_id.Rs * g_param_id.Rs;
                        
                    g_param_id.Lq = sqrtf(Z_sq - R_sq) / omega_L;
                }
                
                // 计算平均电感
                g_param_id.Ls = (g_param_id.Ld + g_param_id.Lq) * 0.5f;
                
                // 完成辨识
                g_param_id.state = PARAM_ID_COMPLETED;
                
                // 打印最终结果
                debug_log("Rs=%.6f Ld=%.6f Lq=%.6f Ls=%.6f", 
                         g_param_id.Rs, g_param_id.Ld, g_param_id.Lq, g_param_id.Ls);
            }
        }
        break;
        
        case PARAM_ID_COMPLETED:
            // 辨识完成，停止注入电压
            foc_ctrl->out_d = 0.0f;
            foc_ctrl->out_q = 0.0f;
            break;
            
        default:
            break;
    }
    
    return g_param_id.state;
}
