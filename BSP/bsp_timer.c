#include "bsp_timer.h"
#include "Config.h"

#define FOC_CONTROL_MODE FOC_MODE

// 时间戳变量
static volatile uint32_t timestamp_ms = 0;
static volatile uint32_t timestamp_us = 0;

/**
 * @brief 初始化TIM1高级定时器输出互补PWM波
 */
void bsp_timer_init(void)
{
  // 先获取APB2时钟，计算TIM1实际时钟
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  // 使能相关时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /*--------------------------------------GPIO配置--------------------------------------------------------*/

  // 配置高端PWM引脚 (PA8, PA9, PA10)
  GPIO_InitStructure.GPIO_Pin = PWM_HA_PIN | PWM_HB_PIN | PWM_HC_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // 复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // 输出速度
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    // 无上下拉
  GPIO_Init(PWM_H_PORT, &GPIO_InitStructure);

  // 配置低端PWM引脚 (PB13, PB14, PB15)
  GPIO_InitStructure.GPIO_Pin = PWM_LA_PIN | PWM_LB_PIN | PWM_LC_PIN;
  GPIO_Init(PWM_L_PORT, &GPIO_InitStructure);

  // 配置引脚复用功能
  GPIO_PinAFConfig(PWM_H_PORT, GPIO_PinSource8, GPIO_AF_TIM1);  // PA8 -> TIM1_CH1
  GPIO_PinAFConfig(PWM_H_PORT, GPIO_PinSource9, GPIO_AF_TIM1);  // PA9 -> TIM1_CH2
  GPIO_PinAFConfig(PWM_H_PORT, GPIO_PinSource10, GPIO_AF_TIM1); // PA10 -> TIM1_CH3
  GPIO_PinAFConfig(PWM_L_PORT, GPIO_PinSource13, GPIO_AF_TIM1); // PB13 -> TIM1_CH1N
  GPIO_PinAFConfig(PWM_L_PORT, GPIO_PinSource14, GPIO_AF_TIM1); // PB14 -> TIM1_CH2N
  GPIO_PinAFConfig(PWM_L_PORT, GPIO_PinSource15, GPIO_AF_TIM1); // PB15 -> TIM1_CH3N

  /*------------------------------------------定时器配置----------------------------------------------------*/

  // TIM1基本定时器配置
  TIM_TimeBaseStructure.TIM_Prescaler = 0;                                // 预分频器
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1; // 中心对齐计数模式,计数器先向上计数到ARR，再向下计数到0
  TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;                          // 自动重载值  4199   ARR
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;                 // 时钟分频,和死区时间计算有关
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;                        // 重复计数器
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  // PWM输出配置
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;           // PWM模式1, 计数器 < CCR时：有效电平
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;      // 主输出使能
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;    // 互补输出使能
  TIM_OCInitStructure.TIM_Pulse = 0;                              // CCR
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;    // 主要输出极性高
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High; // 互补输出极性高
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;       // 空闲状态
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;     // 互补空闲状态

  // 配置通道1, 2, 3
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);

  // 使能预装载寄存器
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

 /*------------------------------------------TRGO配置----------------------------------------------------*/
  // 配置TIM1 TRGO信号源为_OC4REF，用于触发ADC注入组转换
  // 这样可以精确控制触发时机，在三相PWM都为低电平时触发
  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC4Ref);
  
  // 配置通道4作为触发源，不输出PWM，仅用于触发ADC
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;           // PWM模式2，确保在特定时刻触发
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; // 不输出PWM波形
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; // 不输出互补PWM波形
  TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD - 20;            // 确保零相位触发，避开开关噪声
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

  // 使能TIM1主输出
  TIM_CtrlPWMOutputs(TIM1, ENABLE);

  // 使能TIM1
  TIM_Cmd(TIM1, ENABLE);
}

void bsp_pwm_stop(void)
{
  TIM_Cmd(TIM1, DISABLE);
  TIM_CtrlPWMOutputs(TIM1, DISABLE);
}

void bsp_pwm_start(void)
{
  TIM_Cmd(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

/**
 * @brief 设置PWM占空比:210(5%)-4199(99.98%),分辨率：0.024%
 * @param ha_duty A相高端占空比 (0-PWM_PERIOD)
 * @param hb_duty B相高端占空比 (0-PWM_PERIOD)
 * @param hc_duty C相高端占空比 (0-PWM_PERIOD)
 */
void bsp_pwm_set_duty(uint16_t ha_duty, uint16_t hb_duty, uint16_t hc_duty)
{
  TIM_SetCompare1(TIM1, ha_duty);
  TIM_SetCompare2(TIM1, hb_duty);
  TIM_SetCompare3(TIM1, hc_duty);
}

/**
 * @brief 初始化TIM2 1ms中断
 */
void bsp_timestamp_init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 168MHz主频，APB1=42MHz，TIM2时钟=84MHz
    // 目标：1ms = 1kHz
    
    // 方案：分两步
    // Prescaler: 84MHz / 84 = 1MHz
    // Period: 1MHz / 1000 = 1kHz = 1ms
    
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;        // 84MHz / 84 = 1MHz
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;         // 1MHz / 1000 = 1kHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief TIM2中断服务函数
 * @note  1ms触发一次
 */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        // FOC外环控制
        foc_control_out();
    }
}
