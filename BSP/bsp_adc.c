#include "bsp_adc.h"

// 配置参数：根据硬件特性调整
#define ADC_DMA_BUFFER_SIZE 4        // 单个缓冲区大小（4个电压通道）
#define ADC_CALIBRATION_DELAY 1000 // ADC启动校准等待延迟
#define INJ_CHANNELS 3              // 注入组通道数（3个电流通道）
#define REG_CHANNELS 4              // 规则组通道数（4个电压通道）

// ADC DMA双缓冲区（循环模式，规则组电压通道）
static uint16_t adc_dma_buffer[2][ADC_DMA_BUFFER_SIZE];  // DMA传输为16位

// 数据标志与存储
volatile uint8_t adc_data_ready = 0;  // 0:无新数据, 1:缓冲区0就绪, 2:缓冲区1就绪
foc_data_t foc_raw_data = {0};        // 原始数据
foc_data_v foc_datav = {0};           // 处理后电压数据
foc_data_i foc_datai = {0};           // 处理后电流数据

// 电流转换系数 = 3.3 / 4096 / Ω / 增益倍速
float32_t I_tran = ADC_REF_VOLTAGE / ADC_MAX_VALUE / R_Current / INA240_GAIN;

// 校准参数（需实际校准获取）
typedef struct
{
  q15_t ia_offset;  // 电流A相零点偏移（q15_t对应int16_t）
  q15_t ib_offset;  // 电流B相零点偏移
  q15_t ic_offset;  // 电流C相零点偏移
  float32_t v_gain; // 相电压增益（浮点）
} adc_calib_t;

// 电压转换系数 = 3.3V / 4095 * (2.2+56)kΩ / 2.2kΩ
static adc_calib_t adc_calib = {0, 0, 0, ADC_REF_VOLTAGE / ADC_MAX_VALUE * (R_Voaltage_1 + R_Voaltage_2) / R_Voaltage_2};

/**
 * @brief DMA中断服务函数（处理缓冲区切换）
 */
void DMA2_Stream0_IRQHandler(void)
{
  // 半传输完成（缓冲区0满）
  if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0))
  {
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
    // 读取缓冲区0数据（电压，uint16_t转q15_t）
    foc_raw_data.va = (q15_t)adc_dma_buffer[0][0];
    foc_raw_data.vb = (q15_t)adc_dma_buffer[0][1];
    foc_raw_data.vc = (q15_t)adc_dma_buffer[0][2];
    foc_raw_data.vbus = (q15_t)adc_dma_buffer[0][3];
    adc_data_ready = 1;  // 标记缓冲区0数据就绪
  }

  // 传输完成（缓冲区1满）
  if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
  {
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
    // 读取缓冲区1数据（电压，uint16_t转q15_t）
    foc_raw_data.va = (q15_t)adc_dma_buffer[1][0];
    foc_raw_data.vb = (q15_t)adc_dma_buffer[1][1];
    foc_raw_data.vc = (q15_t)adc_dma_buffer[1][2];
    foc_raw_data.vbus = (q15_t)adc_dma_buffer[1][3];
    adc_data_ready = 2;  // 标记缓冲区1数据就绪
  }
}

/**
 * @brief ADC注入组中断服务函数（处理电流数据）
 */
void ADC_IRQHandler(void)
{
  // GPIO_ToggleBits(GPIOA, GPIO_Pin_5);

  if (ADC_GetITStatus(ADC1, ADC_IT_JEOC))
  {
    ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    
    foc_raw_data.ia = (q15_t)ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
    foc_raw_data.ib = (q15_t)ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
    foc_raw_data.ic = (q15_t)ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);

    // 手动校准零点漂移
    foc_datai.ia = (2042 - foc_raw_data.ia);
    foc_datai.ib = (2046 - foc_raw_data.ib);
    foc_datai.ic = (2046 - foc_raw_data.ic);

    // 3. 使用DSP库函数执行乘法：最终值 = 校准值 * 转换系数
    arm_mult_f32(&foc_datai.ia, &I_tran, &foc_datai.ia, 1);
    arm_mult_f32(&foc_datai.ib, &I_tran, &foc_datai.ib, 1);
    arm_mult_f32(&foc_datai.ic, &I_tran, &foc_datai.ic, 1);

    // if (bsp_adc_process_data())
    //  {
    //    // 获取校准后的电压ADC数据
    //    foc_datav = bsp_adc_get_calib_data();
    //  }

    // 先计算核心值
    // float temp_value = (foc_datai.ic / 3.3f * 4095.0f) + 2048.0f;

    // // 限制在有效范围内
    // if(temp_value > 4095.0f) temp_value = 4095.0f;
    // if(temp_value < 0.0f) temp_value = 0.0f;

    // // 转换为整数并输出
    // uint16_t dac_value = (uint16_t)temp_value;
    // DAC_SetChannel2Data(DAC_Align_12b_R, dac_value);

    // 执行FOC内环控制
    foc_control();
  }
}

/**
 * @brief 初始化FOC电流和电压采样ADC
 */
void bsp_adc_init(void)
{
  // 初始化ADC
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // 使能GPIOA、GPIOC、ADC和DMA时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  // 配置PC0, PC1, PC2, PC3为模拟输入模式（电流和母线电压）
  GPIO_InitStructure.GPIO_Pin = FOC_CURRENT_IA_PIN | FOC_CURRENT_IB_PIN |
                                FOC_CURRENT_IC_PIN | FOC_VOLTAGE_VBUS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(FOC_CURRENT_GPIO_PORT, &GPIO_InitStructure);

  // 配置PA0, PA1, PA2为模拟输入模式（相电压）
  GPIO_InitStructure.GPIO_Pin = FOC_VOLTAGE_VA_PIN | FOC_VOLTAGE_VB_PIN | FOC_VOLTAGE_VC_PIN;
  GPIO_Init(FOC_VOLTAGE_GPIO_PORT_A, &GPIO_InitStructure);

  // ADC通用配置
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;  // 优化分频
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;  // 增加通道切换延迟
  ADC_CommonInit(&ADC_CommonInitStructure);

  // -------------------------- 规则组配置（连续转换） --------------------------
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  // 外部触发模式
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigInjecConv_T1_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = REG_CHANNELS;  // 4个电压通道
  ADC1->CR2 |= ADC_CR2_DDS;
  ADC_Init(ADC1, &ADC_InitStructure);

  // 配置规则组通道（电压）
  ADC_RegularChannelConfig(ADC1, FOC_VOLTAGE_VA_CHANNEL, 1, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, FOC_VOLTAGE_VB_CHANNEL, 2, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, FOC_VOLTAGE_VC_CHANNEL, 3, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, FOC_VOLTAGE_VBUS_CHANNEL, 4, ADC_SampleTime_56Cycles);

  // -------------------------- 注入组配置（同步触发） --------------------------
  // 注入组通道数：3个电流通道
  ADC_InjectedSequencerLengthConfig(ADC1, INJ_CHANNELS);
  // 配置注入组通道（电流）
  ADC_InjectedChannelConfig(ADC1, FOC_CURRENT_IA_CHANNEL, 1, ADC_SampleTime_28Cycles);
  ADC_InjectedChannelConfig(ADC1, FOC_CURRENT_IB_CHANNEL, 2, ADC_SampleTime_28Cycles);
  ADC_InjectedChannelConfig(ADC1, FOC_CURRENT_IC_CHANNEL, 3, ADC_SampleTime_28Cycles);
  // 注入组触发源：TIM1 TRGO（更新事件）
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
  ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Rising);  // 上升沿触发

  // 使能注入组转换完成中断（优先级高于规则组）
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // 最高优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // DMA 初始化配置（用于规则组电压采集）
  DMA_DeInit(DMA2_Stream0);
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adc_dma_buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = ADC_DMA_BUFFER_SIZE * 2;  // 总大小=2个缓冲区
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);

  // 使能DMA半传输和传输完成中断
  DMA_ITConfig(DMA2_Stream0, DMA_IT_HT | DMA_IT_TC, ENABLE);

  // NVIC配置（DMA中断）
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // 清除DMA传输完成标志
  DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0);

  // 使能 ADC 的 DMA 请求
  ADC_DMACmd(ADC1, ENABLE);

  // 使能ADC1
  ADC_Cmd(ADC1, ENABLE);

  // 等待ADC稳定
  for(volatile int i=0; i<ADC_CALIBRATION_DELAY; i++);

  // 使能 DMA 通道
  DMA_Cmd(DMA2_Stream0, ENABLE);

  // 启动规则组连续转换
  ADC_SoftwareStartConv(ADC1);

  // 测试注入组ADC采样时机
  // 配置PA5为调试引脚
  // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  // GPIO_Init(GPIOA, &GPIO_InitStructure);

  // 配置PA5为DAC输出
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  // 使能DAC时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  
  // DAC配置
  DAC_InitTypeDef DAC_InitStructure;
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);
  
  // 使能DAC通道2
  DAC_Cmd(DAC_Channel_2, ENABLE);
}

/**
 * @brief 处理新的电压ADC数据
 * @return 1:数据更新完成, 0:无新数据
 */
uint8_t bsp_adc_process_data(void)
{
  if (adc_data_ready == 0)
    {
      return 0;  // 无新数据
    }

  float32_t raw_va = (float32_t)foc_raw_data.va;
  float32_t raw_vb = (float32_t)foc_raw_data.vb;
  float32_t raw_vc = (float32_t)foc_raw_data.vc;
  float32_t raw_vbus = (float32_t)foc_raw_data.vbus;
  
  arm_mult_f32(&raw_va, &adc_calib.v_gain, &foc_datav.va, 1);
  arm_mult_f32(&raw_vb, &adc_calib.v_gain, &foc_datav.vb, 1);
  arm_mult_f32(&raw_vc, &adc_calib.v_gain, &foc_datav.vc, 1);
  arm_mult_f32(&raw_vbus, &adc_calib.v_gain, &foc_datav.vbus, 1);

  // 清除标志
  adc_data_ready = 0;
  return 1;
}

/**
 * @brief 获取FOC采样数据（原始数据）
 * @return FOC数据结构体
 */
foc_data_t bsp_adc_get_RAW_Data(void)
{
  return foc_raw_data;
}

/**
 * @brief 获取校准后的FOC电压采样数据(需手动采样)
 * @return 处理后的FOC电压数据（带滤波）
 */
foc_data_v bsp_adc_get_calib_data(void)
{
  return foc_datav;
}
