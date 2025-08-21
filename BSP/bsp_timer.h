#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include "stm32f4xx.h"
#include "Config.h"
#include "foc_gather.h"
#include "foc_debug.h"
#include "foc_control.h"

extern uint32_t TIM1_Clock; // 84MHz

// 基于TIM1时钟计算PWM周期（中心对齐模式下，频率 = TIM1_Clock/(TIM_CKD_DIV2*(PWM_PERIOD+1))）
#define PWM_FREQUENCY     10000 // PWM频率 10K (Hz)
#define PWM_PERIOD        ((TIM1_Clock / (2 * PWM_FREQUENCY)) - 1)  // 4199

// 死区时间计算     死区时间基准频率 = TIM1_Clock/TIM_CKD_DIV2 = 84MHz/2 = 42MHz  死区时间步长 = 1/42MHz = 23.81ns
#define DEAD_TIME_US      1     // 实际死区时间，单位us
#define DEAD_TIME         ((uint16_t)((TIM1_Clock / (2 * 1000000)) * DEAD_TIME_US))     // 死区时间 1us = DEAD_TIME(42) * 23.81ns(死区时间步长) DEAD_TIME实际代表死区步数

// 高端PWM引脚定义 (TIM1 CH1, CH2, CH3)
#define PWM_HA_PIN        GPIO_Pin_8
#define PWM_HB_PIN        GPIO_Pin_9
#define PWM_HC_PIN        GPIO_Pin_10
#define PWM_H_PORT        GPIOA

// 低端PWM引脚定义 (TIM1 CH1N, CH2N, CH3N)
#define PWM_LA_PIN        GPIO_Pin_13
#define PWM_LB_PIN        GPIO_Pin_14
#define PWM_LC_PIN        GPIO_Pin_15
#define PWM_L_PORT        GPIOB

// 函数声明
void bsp_timer_init(void);
void bsp_pwm_set_duty(uint16_t ha_duty, uint16_t hb_duty, uint16_t hc_duty);
void bsp_pwm_enable(void);
void bsp_pwm_disable(void);

#endif /* __BSP_TIMER_H */
