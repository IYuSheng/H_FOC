#ifndef __FOC_CONVERSION_H
#define __FOC_CONVERSION_H

#include "stm32f4xx.h"
#include "Config.h"
#include "bsp_adc.h"

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

extern position_pid_t position_pid;

// FOC变换相关函数声明
extern inline float32_t angle_normalize(float32_t angle);
// SVPWM相关函数声明
extern inline uint8_t svpwm_sector_calc(float32_t u_alpha, float32_t u_beta);
extern inline void svpwm_calc_times(int32_t sector, float32_t u_alpha, float32_t u_beta, float32_t vdc, float32_t* T1, float32_t* T2, float32_t* T0);
extern inline void svpwm_duty_calc(int32_t sector, float32_t T1, float32_t T2, float32_t T0, float32_t* duty_a, float32_t* duty_b, float32_t* duty_c);
extern inline void inv_park_transform_f32(float32_t* d_ptr, float32_t* q_ptr, float32_t* alpha_ptr, float32_t* beta_ptr, float32_t angle);
// Clarke+Park变换
extern inline void abc_to_dq_current(void *current_abc_ptr, DQTypeDef *current_dq, float angle);
// 电流环PID控制器计算
extern inline float32_t foc_id_pid_calculate(float32_t target_id, float32_t actual_id);
extern inline float32_t foc_iq_pid_calculate(float32_t target_iq, float32_t actual_iq);
// 速度环PID控制器计算
extern inline float32_t foc_speed_pid_calculate(float32_t target_speed, float32_t actual_speed);
// 位置环PID控制器计算
extern inline float32_t foc_position_pid_calculate(float32_t target_position, float32_t current_position);

#endif /* __FOC_CONVERSION_H */
