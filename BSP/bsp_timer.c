#include "bsp_timer.h"

uint32_t TIM1_Clock;

/**
 * @brief 初始化TIM1高级定时器输出互补PWM波
 */
void bsp_timer_init(void)
{
  // 先获取APB2时钟，计算TIM1实际时钟
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  // 若APB2预分频 > 1，则TIM1时钟 = 2*APB2时钟；否则等于APB2时钟
  if (RCC_Clocks.PCLK2_Frequency < SystemCoreClock / 2)
    {
      TIM1_Clock = 2 * RCC_Clocks.PCLK2_Frequency;
    }
  else
    {
      TIM1_Clock = RCC_Clocks.PCLK2_Frequency;
    }

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
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

  // 配置刹车引脚（使用PB12作为刹车输入）
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // 复用刹车功能（TIM1_BKIN）
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_TIM1);

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

  // 自动输出使能、断路功能、死区设置
  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
  TIM_BDTRInitStructure.TIM_DeadTime = DEAD_TIME;             // DEAD_TIME = (TIM1时钟 / 2MHz) × 设置时间(μs)
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;        // 断路功能开启
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

  /*------------------------------------------中断配置----------------------------------------------------*/

  // 使能更新中断,配置为中心对齐模式1，每个PWM周期将会产生两次更新中断
  // 第一次为向下计数开始点(ARR至ARR-1)对应PWM波形峰值点
  // 第二次为向上计数开始点(0至1)对应PWM波形最低点,此时应进行电流采样
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  // 配置NVIC
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 高优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // 使能刹车中断
  TIM_ITConfig(TIM1, TIM_IT_Break, ENABLE);

  // 使能TIM1主输出
  TIM_CtrlPWMOutputs(TIM1, ENABLE);

  // 使能TIM1
  TIM_Cmd(TIM1, ENABLE);
  TIM_SetCompare1(TIM1, 1000);
  TIM_SetCompare2(TIM1, 1000);
  TIM_SetCompare3(TIM1, 1000);
}

/**
 * @brief 设置PWM占空比:210(5%)-4199(99.98%),分辨率：0.024%
 * @param ha_duty A相高端占空比 (0-PWM_PERIOD)
 * @param hb_duty B相高端占空比 (0-PWM_PERIOD)
 * @param hc_duty C相高端占空比 (0-PWM_PERIOD)
 */
void bsp_pwm_set_duty(uint16_t ha_duty, uint16_t hb_duty, uint16_t hc_duty)
{
  const uint16_t MIN_DUTY = PWM_PERIOD / 20; // 最小5%占空比
  // 限制最大和最小占空比
  ha_duty = (ha_duty < MIN_DUTY) ? MIN_DUTY : (ha_duty > PWM_PERIOD ? PWM_PERIOD : ha_duty);
  hb_duty = (hb_duty < MIN_DUTY) ? MIN_DUTY : (hb_duty > PWM_PERIOD ? PWM_PERIOD : hb_duty);
  hc_duty = (hc_duty < MIN_DUTY) ? MIN_DUTY : (hc_duty > PWM_PERIOD ? PWM_PERIOD : hc_duty);

  TIM_SetCompare1(TIM1, ha_duty);
  TIM_SetCompare2(TIM1, hb_duty);
  TIM_SetCompare3(TIM1, hc_duty);
}

/**
 * @brief 使能PWM输出
 */
void bsp_pwm_enable(void)
{
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

/**
 * @brief 禁用PWM输出
 */
void bsp_pwm_disable(void)
{
  TIM_CtrlPWMOutputs(TIM1, DISABLE);
  TIM_SetCompare1(TIM1, 0);
  TIM_SetCompare2(TIM1, 0);
  TIM_SetCompare3(TIM1, 0);
}

// 中断服务程序
void TIM1_UP_TIM10_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
      TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
      // 调用FOC核心控制函数

    }
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM1, TIM_IT_Break) != RESET)
    {
      TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
      bsp_pwm_disable(); // 紧急关闭PWM
      // 可添加故障标志位设置、日志记录等
    }
}
