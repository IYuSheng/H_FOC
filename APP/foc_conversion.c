#include "foc_conversion.h"

#define SQRT3_INV 0.57735026919f  // 1/sqrt(3)
#define SQRT3    1.73205080757f   // sqrt(3)
#define SQRT3_DIV_2 0.86602540378f // sqrt(3)/2

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
  alpha_beta->beta  = (abc->b - abc->c) * SQRT3_INV;
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
  abc->b = -0.5f * alpha_beta->alpha + SQRT3_DIV_2 * alpha_beta->beta;
  abc->c = -0.5f * alpha_beta->alpha - SQRT3_DIV_2 * alpha_beta->beta;
  
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
