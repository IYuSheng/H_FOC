#include "foc_conversion.h"

position_pid_t position_pid;
position_pid_t id_pid;
position_pid_t iq_pid;
position_pid_t speed_pid;

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
inline uint8_t svpwm_sector_calc(float32_t u_alpha, float32_t u_beta)
{
    uint8_t sector, pos1, pos2, pos3 = 0;
    
    float32_t X = u_beta;
    float32_t Y = (_SQRT3  * u_alpha - u_beta);
    float32_t Z = (-_SQRT3 * u_alpha - u_beta);
    
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
inline void svpwm_calc_times(int32_t sector, float32_t u_alpha, float32_t u_beta, 
                                    float32_t vdc, float32_t* T1, float32_t* T2, float32_t* T0)
{
    // 1. 计算目标电压矢量幅值（避免过调制）
    float32_t u_mag;
    float32_t alpha_sq, beta_sq, sum_sq;
    
    // 使用DSP函数计算平方和开方
    arm_mult_f32(&u_alpha, &u_alpha, &alpha_sq, 1);
    arm_mult_f32(&u_beta, &u_beta, &beta_sq, 1);
    arm_add_f32(&alpha_sq, &beta_sq, &sum_sq, 1);
    arm_sqrt_f32(sum_sq, &u_mag);
    
    const float32_t u_max = vdc * _1_SQRT3; // SVPWM最大输出电压幅值
    
    // 2. 过调制处理
    if (u_mag > u_max && u_mag > 1e-6f)
    {
        float32_t scale = u_max / u_mag;
        arm_scale_f32(&u_alpha, scale, &u_alpha, 1);
        arm_scale_f32(&u_beta, scale, &u_beta, 1);
        u_mag = u_max;
    }
    
    // 3. 计算与扇区判断完全一致的中间变量
    float32_t factor = FACTOR / vdc;
    float32_t half_factor = factor * 0.5f;
    float32_t X, Y, Z;
    
    // X
    arm_scale_f32(&u_beta, factor, &X, 1);
    // Y
    float32_t temp1, temp2;
    float32_t sqrt3_const = _SQRT3;
    arm_scale_f32(&sqrt3_const, u_alpha, &temp1, 1);
    arm_add_f32(&temp1, &u_beta, &temp2, 1);
    arm_scale_f32(&temp2, half_factor, &Y, 1);
    // Z
    arm_scale_f32(&sqrt3_const, u_alpha, &temp1, 1);
    arm_sub_f32(&temp1, &u_beta, &temp2, 1);
    arm_scale_f32(&temp2, half_factor, &Z, 1);
    arm_negate_f32(&Z, &Z, 1); // 取负值
    
    // 4. 按扇区计算T1和T2（基于X/Y/Z）
    switch (sector)
    {
        case 1:  // 扇区1：V4(100) + V6(110) → T1=Y, T2=X 3
            *T1 = -Z;
            *T2 = X;
            break;
        case 2:  // 扇区2：V6(110) + V2(010) → T1=X, T2=-Z 1
            *T1 = Z;
            *T2 = Y;
            break;
        case 3:  // 扇区3：V2(010) + V3(011) → T1=-Z, T2=-Y 5
            *T1 = X;
            *T2 = -Y;
            break;
        case 4:  // 扇区4：V3(011) + V1(001) → T1=-Y, T2=-X 4
            *T1 = -X;
            *T2 = Z;
            break;
        case 5:  // 扇区5：V1(001) + V5(101) → T1=-X, T2=Z 6
            *T1 = -Y;
            *T2 = -Z;
            break;
        case 6:  // 扇区6：V5(101) + V4(100) → T1=Z, T2=Y 2
            *T1 = Y;
            *T2 = -X;
            break;
        default:
            *T1 = 0.0f;
            *T2 = 0.0f;
            break;
    }
    
    // 6. 计算零矢量时间（确保T0≥0，避免负数）
    float32_t t1_t2_sum;
    arm_add_f32(T1, T2, &t1_t2_sum, 1);
    *T0 = PWM_PERIOD_S - t1_t2_sum;
}

/**
 * @brief SVPWM计算三相导通时间并转换为占空比
 * @param sector 当前扇区 (1-6)
 * @param T1 基本矢量1作用时间
 * @param T2 基本矢量2作用时间
 * @param T0 零矢量作用时间
 * @param duty_a A相占空比指针
 * @param duty_b B相占空比指针
 * @param duty_c C相占空比指针
 */
inline void svpwm_duty_calc(int32_t sector, float32_t T1, float32_t T2, float32_t T0, 
                     float32_t* duty_a, float32_t* duty_b, float32_t* duty_c)
{
    float32_t Ta, Tb, Tc;                 // 三相桥臂导通时间（s）
    float32_t T0_half = T0 / 2.0f;

    // 按扇区计算三相导通时间
    switch (sector)
    {
        case 1:  // 扇区1
            Ta = T0_half + T1 + T2;
            Tb = T0_half + T2;
            Tc = T0_half;
            break;
        case 2:  // 扇区2
            Ta = T0_half + T2;
            Tb = T0_half + T1 + T2;
            Tc = T0_half;
            break;
        case 3:  // 扇区3
            Ta = T0_half;
            Tb = T0_half + T1 + T2;
            Tc = T0_half + T2;
            break;
        case 4:  // 扇区4
            Ta = T0_half;
            Tb = T0_half + T2;
            Tc = T0_half + T1 + T2;
            break;
        case 5:  // 扇区5
            Ta = T0_half + T2;
            Tb = T0_half;
            Tc = T0_half + T1 + T2;
            break;
        case 6:  // 扇区6
            Ta = T0_half + T1 + T2;
            Tb = T0_half;
            Tc = T0_half + T2;
            break;
        default:  // 异常扇区
            Ta = T0_half;
            Tb = T0_half;
            Tc = T0_half;
            break;
    }
    // 导通时间→占空比(此处需要将占空比除以2，因为定时器是中心对齐模式,算出来的Ta,Tb,Tc是两个PWM周期的时间下的有效时间)
    *duty_a = Ta * HALF_PWM_FREQ;
    *duty_b = Tb * HALF_PWM_FREQ;
    *duty_c = Tc * HALF_PWM_FREQ;
}

/**
 * @brief 反Park变换
 * @param d_ptr D轴输入指针
 * @param q_ptr Q轴输入指针
 * @param alpha_ptr Alpha轴输出指针
 * @param beta_ptr Beta轴输出指针
 * @param angle 电角度(弧度)
 */
inline void inv_park_transform_f32(float32_t* d_ptr, float32_t* q_ptr, 
                                          float32_t* alpha_ptr, float32_t* beta_ptr, 
                                          float32_t angle)
{
    float32_t d = *d_ptr;
    float32_t q = *q_ptr;
    float32_t sin_theta, cos_theta;
    
    // 将电角度从弧度转换为角度
    float32_t angle_deg;
    arm_scale_f32(&angle, RAD_TO_DEG, &angle_deg, 1);
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

    // 反Park变换：将旋转DQ坐标系电压转换为静止αβ坐标系电压
    // 反Park变换公式
    *alpha_ptr = d * cos_theta - q * sin_theta;
    *beta_ptr = d * sin_theta + q * cos_theta;
}

/**
 * @brief 将三相电流转换为dq坐标系下的电流值
 * @param current_abc 三相电流值 (ia, ib, ic)
 * @param current_dq 输出的dq轴电流值 (id, iq)
 * @param angle 电角度(弧度)
 */
inline void abc_to_dq_current(void *current_abc_ptr, DQTypeDef *current_dq, float angle)
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
    arm_park_f32(alpha, beta, &current_dq->d, &current_dq->q, sin_val, cos_val);
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

    // if(fabs(error) < 0.5f)
    // {
    //     speed_pid.integral = 0.0f;
    // }
    
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
