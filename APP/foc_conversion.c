#include "foc_conversion.h"

/**
 * @brief 电角度归一化（映射到0~2π范围）
 * @param angle 输入电角度（rad，范围无限制）
 * @return 归一化后电角度（rad，0~2π）
 */
inline float32_t angle_normalize(float32_t angle)
{
    // 使用fmodf函数优化角度归一化
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
    uint8_t sector = 0;
    
    float32_t v_ref1 = u_beta;
    float32_t v_ref2 = (_SQRT3  * u_alpha - u_beta) / 2.0f;
    float32_t v_ref3 = (-_SQRT3 * u_alpha - u_beta) / 2.0f;
    
    uint8_t pos1 = (v_ref1 > 1e-6f) ? 1 : 0;
    uint8_t pos2 = (v_ref2 > 1e-6f) ? 1 : 0;
    uint8_t pos3 = (v_ref3 > 1e-6f) ? 1 : 0;

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
    float32_t factor = _SQRT3 * PWM_PERIOD_S / vdc;
    float32_t v_ref1, v_ref2, v_ref3;
    
    // 使用DSP函数进行计算
    arm_scale_f32(&u_beta, factor, &v_ref1, 1);
    
    float32_t temp1, temp2;
    float32_t sqrt3_const = _SQRT3;
    arm_scale_f32(&sqrt3_const, u_alpha, &temp1, 1);
    arm_add_f32(&temp1, &u_beta, &temp2, 1);
    float32_t half_factor = factor / 2.0f;
    arm_scale_f32(&temp2, half_factor, &v_ref2, 1);
    
    arm_scale_f32(&sqrt3_const, u_alpha, &temp1, 1);
    arm_sub_f32(&temp1, &u_beta, &temp2, 1);
    arm_scale_f32(&temp2, half_factor, &v_ref3, 1);
    arm_negate_f32(&v_ref3, &v_ref3, 1); // 取负值
    
    // 4. 按扇区计算T1和T2（基于v_ref1/v_ref2/v_ref3）
    switch (sector)
    {
        case 1:  // 扇区1：V4(100) + V6(110) → T1=v_ref2, T2=v_ref1 3
            *T1 = -v_ref3;
            *T2 = v_ref1;
            break;
        case 2:  // 扇区2：V6(110) + V2(010) → T1=v_ref1, T2=-v_ref3 1
            *T1 = v_ref3;
            *T2 = v_ref2;
            break;
        case 3:  // 扇区3：V2(010) + V3(011) → T1=-v_ref3, T2=-v_ref2 5
            *T1 = v_ref1;
            *T2 = -v_ref2;
            break;
        case 4:  // 扇区4：V3(011) + V1(001) → T1=-v_ref2, T2=-v_ref1 4
            *T1 = -v_ref1;
            *T2 = v_ref3;
            break;
        case 5:  // 扇区5：V1(001) + V5(101) → T1=-v_ref1, T2=v_ref3 6
            *T1 = -v_ref2;
            *T2 = -v_ref3;
            break;
        case 6:  // 扇区6：V5(101) + V4(100) → T1=v_ref3, T2=v_ref2 2
            *T1 = v_ref2;
            *T2 = -v_ref1;
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
    if (*T0 < 0.0f) *T0 = 0.0f; // 增加保护，防止T1+T2超出周期
}

/**
 * @brief Clark变换 (从三相坐标系转换到两相静止坐标系)
 * @param abc 输入的三相值 (a, b, c)
 * @param alpha_beta 输出的两相静止坐标系值 (alpha, beta)
 */
void clark_transform(ABCTypeDef *abc, AlphaBetaTypeDef *alpha_beta)
{
  // Clarke变换 - 等幅变换 (功率不变)
  // alpha = (2*a - b - c) / 3
  // beta  = (b - c) / sqrt(3)
  alpha_beta->alpha = (2.0f * abc->a - abc->b - abc->c) * 0.33333333333f;
  alpha_beta->beta  = (abc->b - abc->c) * _1_SQRT3;
}

/**
 * @brief 反Clark变换 (从两相静止坐标系转换到三相坐标系)
 * @param alpha_beta 输入的两相静止坐标系值 (alpha, beta)
 * @param abc 输出的三相值 (a, b, c)
 */
void inv_clark_transform(AlphaBetaTypeDef *alpha_beta, ABCTypeDef *abc)
{
  // 反Clarke变换 - 等幅变换 (功率不变)
  // a = alpha
  // b = -0.5 * alpha + sqrt(3)/2 * beta
  // c = -0.5 * alpha - sqrt(3)/2 * beta
  abc->a = alpha_beta->alpha;
  abc->b = -0.5f * alpha_beta->alpha + _1_SQRT3 * alpha_beta->beta;
  abc->c = -0.5f * alpha_beta->alpha - _1_SQRT3 * alpha_beta->beta;
  
  // 确保三相和为零 (可选)
  // float zero_seq = (abc->a + abc->b + abc->c) / 3.0f;
  // abc->a -= zero_seq;
  // abc->b -= zero_seq;
  // abc->c -= zero_seq;
}

/**
 * @brief Park变换 (从两相静止坐标系转换到两相旋转坐标系)
 * @param alpha_beta 输入的两相静止坐标系值 (alpha, beta)
 * @param dq 输出的两相旋转坐标系值 (d, q)
 * @param angle 旋转角度(电角度，弧度)
 */
void park_transform(AlphaBetaTypeDef *alpha_beta, DQTypeDef *dq, float angle)
{
  float sin_val = sinf(angle);
  float cos_val = cosf(angle);

  // Park变换
  // d = alpha * cos + beta * sin
  // q = -alpha * sin + beta * cos
  dq->d = alpha_beta->alpha * cos_val + alpha_beta->beta * sin_val;
  dq->q = -alpha_beta->alpha * sin_val + alpha_beta->beta * cos_val;
}

/**
 * @brief 反Park变换 (从两相旋转坐标系转换到两相静止坐标系)
 * @param dq 输入的两相旋转坐标系值 (d, q)
 * @param alpha_beta 输出的两相静止坐标系值 (alpha, beta)
 * @param angle 旋转角度(电角度，弧度)
 */
void inv_park_transform(DQTypeDef *dq, AlphaBetaTypeDef *alpha_beta, float angle)
{
  float sin_val = sinf(angle);
  float cos_val = cosf(angle);

  // 反Park变换
  // alpha = d * cos - q * sin
  // beta  = d * sin + q * cos
  alpha_beta->alpha = dq->d * cos_val - dq->q * sin_val;
  alpha_beta->beta  = dq->d * sin_val + dq->q * cos_val;
}
