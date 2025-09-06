#ifndef __FOC_CONVERSION_H
#define __FOC_CONVERSION_H

#include "stm32f4xx.h"
#include "freertos.h"
#include "task.h"
#include "math.h"
#include "arm_math.h"
#include "Config.h"
#include "foc_debug.h"

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

// 位置环PI控制器结构体
typedef struct {
    float32_t kp;           // 比例增益
    float32_t ki;           // 积分增益
    float32_t integral;     // 积分项
    float32_t integral_limit; // 积分限幅
    float32_t error;        // 当前误差
    float32_t output;       // 输出值
    float32_t target;       // 目标位置
    float32_t current;      // 当前位置
} position_pid_t;

// FOC变换相关函数声明
void foc_position_pid_init(float32_t kp, float32_t ki, float32_t integral_limit);
float32_t foc_position_pid_calculate(float32_t target_position, float32_t current_position);
extern inline float32_t angle_normalize(float32_t angle);
extern inline uint8_t svpwm_sector_calc(float32_t u_alpha, float32_t u_beta);
extern inline void svpwm_calc_times(int32_t sector, float32_t u_alpha, float32_t u_beta, float32_t vdc, float32_t* T1, float32_t* T2, float32_t* T0);
void clark_transform(ABCTypeDef *abc, AlphaBetaTypeDef *alpha_beta);
void inv_clark_transform(AlphaBetaTypeDef *alpha_beta, ABCTypeDef *abc);
void park_transform(AlphaBetaTypeDef *alpha_beta, DQTypeDef *dq, float angle);
void inv_park_transform(DQTypeDef *dq, AlphaBetaTypeDef *alpha_beta, float angle);

#endif /* __FOC_CONVERSION_H */
