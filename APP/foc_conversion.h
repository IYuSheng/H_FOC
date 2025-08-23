#ifndef __FOC_CONVERSION_H
#define __FOC_CONVERSION_H

#include "stm32f4xx.h"
#include "freertos.h"
#include "task.h"
#include "math.h"
#include "arm_math.h"
#include "Config.h"

// 三相坐标系结构体
typedef struct
{
  float a;
  float b;
  float c;
} ABCTypeDef;

// 两相静止坐标系结构体 (Alpha-Beta)
typedef struct
{
  float alpha;
  float beta;
} AlphaBetaTypeDef;

// 两相旋转坐标系结构体 (D-Q)
typedef struct
{
  float d;
  float q;
} DQTypeDef;

// FOC变换相关函数声明
extern inline float32_t angle_normalize(float32_t angle);
extern inline uint8_t svpwm_sector_calc(float32_t u_alpha, float32_t u_beta);
extern inline void svpwm_calc_times(int32_t sector, float32_t u_alpha, float32_t u_beta, float32_t vdc, float32_t* T1, float32_t* T2, float32_t* T0);
void clark_transform(ABCTypeDef *abc, AlphaBetaTypeDef *alpha_beta);
void inv_clark_transform(AlphaBetaTypeDef *alpha_beta, ABCTypeDef *abc);
void park_transform(AlphaBetaTypeDef *alpha_beta, DQTypeDef *dq, float angle);
void inv_park_transform(DQTypeDef *dq, AlphaBetaTypeDef *alpha_beta, float angle);

#endif /* __FOC_CONVERSION_H */
