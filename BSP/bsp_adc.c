#include "bsp_adc.h"

// 配置参数：根据硬件特性调整
#define ADC_DMA_BUFFER_SIZE 4        // 单个缓冲区大小（4个电压通道）
#define SAMPLE_AVG_COUNT 4          // 滑动平均次数（抑制随机噪声）
#define ADC_CALIBRATION_DELAY 10000 // ADC启动校准等待延迟
#define INJ_CHANNELS 3              // 注入组通道数（3个电流通道）
#define REG_CHANNELS 4              // 规则组通道数（4个电压通道）
#define LPF_ALPHA 0.4f              // 低通滤波系数(0 < LPF_ALPHA < 1)

// ADC DMA双缓冲区（循环模式，规则组电压通道）
static uint16_t adc_dma_buffer[2][ADC_DMA_BUFFER_SIZE];  // DMA传输为16位

// 数据标志与存储
volatile uint8_t adc_data_ready = 0;  // 0:无新数据, 1:缓冲区0就绪, 2:缓冲区1就绪
foc_data_t foc_raw_data = {0};        // 原始数据
foc_data_v foc_datav = {0};           // 处理后电压数据
foc_data_i foc_datai = {0};           // 处理后电流数据

// 电流转换系数 = 3.3 / 4096 / 1mΩ / 20
float32_t I_tran = ADC_REF_VOLTAGE / ADC_MAX_VALUE / R_Current / INA240_GAIN;

// 滑动平均值缓冲区(电压)
static q31_t va_avg_buf[SAMPLE_AVG_COUNT] = {0};
static q31_t vb_avg_buf[SAMPLE_AVG_COUNT] = {0};
static q31_t vc_avg_buf[SAMPLE_AVG_COUNT] = {0};
static q31_t vbus_avg_buf[SAMPLE_AVG_COUNT] = {0};
static uint8_t avg_idx = 0;  // 滑动平均索引

static uint8_t is_filter_initialized_i = 0;  // 电流滤波器初始化标志
static uint8_t is_filter_initialized_v = 0;  // 电压滤波器初始化标志

// 低通滤波状态变量(电流) - 使用DSP浮点类型
static float32_t ia_lpf = 0.0f, ib_lpf = 0.0f, ic_lpf = 0.0f;

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
 * @brief 电压电流采样通用滤波器（DSP库优化）
 */
static inline void adc_apply_filter(foc_mode mode)
{
  // 电流滤波（一阶低通滤波）
  if(mode == i_mode)
  {
    // 初始化时填充初始值
    if (!is_filter_initialized_i) {
      arm_fill_f32((float32_t)foc_raw_data.ia, &ia_lpf, 1);
      arm_fill_f32((float32_t)foc_raw_data.ib, &ib_lpf, 1);
      arm_fill_f32((float32_t)foc_raw_data.ic, &ic_lpf, 1);
      is_filter_initialized_i = 1;
    }

    // 应用一阶低通滤波器：y = α*x + (1-α)*y_prev
    float32_t x_ia = (float32_t)foc_raw_data.ia;
    float32_t x_ib = (float32_t)foc_raw_data.ib;
    float32_t x_ic = (float32_t)foc_raw_data.ic;
    float32_t coeff1 = LPF_ALPHA;
    float32_t coeff2 = 1.0f - LPF_ALPHA;
    
    // 用DSP库函数计算滤波值（优化浮点运算）
    float32_t temp1, temp2;
    arm_mult_f32(&coeff1, &x_ia, &temp1, 1);  // α*x
    arm_mult_f32(&coeff2, &ia_lpf, &temp2, 1); // (1-α)*y_prev
    arm_add_f32(&temp1, &temp2, &ia_lpf, 1);   // 求和
    
    arm_mult_f32(&coeff1, &x_ib, &temp1, 1);
    arm_mult_f32(&coeff2, &ib_lpf, &temp2, 1);
    arm_add_f32(&temp1, &temp2, &ib_lpf, 1);
    
    arm_mult_f32(&coeff1, &x_ic, &temp1, 1);
    arm_mult_f32(&coeff2, &ic_lpf, &temp2, 1);
    arm_add_f32(&temp1, &temp2, &ic_lpf, 1);

    // 保存滤波后的数据
    foc_datai.ia = ia_lpf;
    foc_datai.ib = ib_lpf;
    foc_datai.ic = ic_lpf;
  }
  // 电压滤波（滑动平均值滤波）
  else if(mode == v_mode)
  {
    q31_t va_mean, vb_mean, vc_mean, vbus_mean;  // 均值结果（32位整数）
    
    // 初始化时填充缓冲区
    if (!is_filter_initialized_v) {
      arm_fill_q31((q31_t)foc_raw_data.va, va_avg_buf, SAMPLE_AVG_COUNT);
      arm_fill_q31((q31_t)foc_raw_data.vb, vb_avg_buf, SAMPLE_AVG_COUNT);
      arm_fill_q31((q31_t)foc_raw_data.vc, vc_avg_buf, SAMPLE_AVG_COUNT);
      arm_fill_q31((q31_t)foc_raw_data.vbus, vbus_avg_buf, SAMPLE_AVG_COUNT);
      is_filter_initialized_v = 1;
    }

    // 更新滑动窗口数据（q15_t转q31_t存储，避免溢出）
    va_avg_buf[avg_idx] = (q31_t)foc_raw_data.va;
    vb_avg_buf[avg_idx] = (q31_t)foc_raw_data.vb;
    vc_avg_buf[avg_idx] = (q31_t)foc_raw_data.vc;
    vbus_avg_buf[avg_idx] = (q31_t)foc_raw_data.vbus;

    // 用DSP库计算平均值（替代手动求和）
    arm_mean_q31(va_avg_buf, SAMPLE_AVG_COUNT, &va_mean);
    arm_mean_q31(vb_avg_buf, SAMPLE_AVG_COUNT, &vb_mean);
    arm_mean_q31(vc_avg_buf, SAMPLE_AVG_COUNT, &vc_mean);
    arm_mean_q31(vbus_avg_buf, SAMPLE_AVG_COUNT, &vbus_mean);

    // 转换为最终电压数据（均值×转换系数）
    foc_datav.va = ((float32_t)va_mean) * adc_calib.v_gain;
    foc_datav.vb = ((float32_t)vb_mean) * adc_calib.v_gain;
    foc_datav.vc = ((float32_t)vc_mean) * adc_calib.v_gain;
    foc_datav.vbus = ((float32_t)vbus_mean) * adc_calib.v_gain;
    
    // 更新平均索引
    avg_idx = (avg_idx + 1) % SAMPLE_AVG_COUNT;
  }
}

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
 * @brief ADC注入组中断服务函数（处理电流数据）- 完全使用DSP库函数
 */
void ADC_IRQHandler(void)
{
  if (ADC_GetITStatus(ADC1, ADC_IT_JEOC))
  {
    ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);  // 清除中断标志

    // 读取注入组转换结果（电流，uint16_t转q15_t）
    foc_raw_data.ia = (q15_t)ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
    foc_raw_data.ib = (q15_t)ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
    foc_raw_data.ic = (q15_t)ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);

    // debug_log("%d, %d, %d", foc_raw_data.ia, foc_raw_data.ib, foc_raw_data.ic);

    // 应用电流滤波
    adc_apply_filter(i_mode);

    // -------------------------- 完全使用DSP库函数处理 --------------------------
    // 1. 准备偏移量的浮点格式
    float32_t ia_offset = (float32_t)adc_calib.ia_offset;
    float32_t ib_offset = (float32_t)adc_calib.ib_offset;
    float32_t ic_offset = (float32_t)adc_calib.ic_offset;
    // debug_log("%.2f,%.2f,%.2f", ia_offset, ib_offset, ic_offset);
    
    // 2. 使用DSP库函数执行减法：校准值 = 偏移量 - 滤波值(由硬件决定，采集到的电流方向应取反，最终采集值为流向电机的电流)
    arm_sub_f32(&ia_offset, &foc_datai.ia, &foc_datai.ia, 1);
    arm_sub_f32(&ib_offset, &foc_datai.ib, &foc_datai.ib, 1);
    arm_sub_f32(&ic_offset, &foc_datai.ic, &foc_datai.ic, 1);
    // debug_log("%.2f,%.2f,%.2f", foc_datai.ia, foc_datai.ia, foc_datai.ia);

    // 3. 使用DSP库函数执行乘法：最终值 = 校准值 * 转换系数
    arm_mult_f32(&foc_datai.ia, &I_tran, &foc_datai.ia, 1);
    arm_mult_f32(&foc_datai.ib, &I_tran, &foc_datai.ib, 1);
    arm_mult_f32(&foc_datai.ic, &I_tran, &foc_datai.ic, 1);
  }
}

/**
 * @brief 初始化ADC规则组用于电流零点校准
 * @note  此函数仅用于校准阶段，校准完成后需调用正常的bsp_adc_init()
 */
void bsp_adc_init_calibration(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  // 使能GPIOA、GPIOC和ADC时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  // 配置PC0, PC1, PC2为模拟输入模式（电流通道）
  GPIO_InitStructure.GPIO_Pin = FOC_CURRENT_IA_PIN | FOC_CURRENT_IB_PIN | FOC_CURRENT_IC_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(FOC_CURRENT_GPIO_PORT, &GPIO_InitStructure);
  
  // ADC通用配置
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);
  
  // 规则组配置（仅用于校准）
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 3;  // 仅3个电流通道
  ADC_Init(ADC1, &ADC_InitStructure);
  
  // 配置规则组通道（电流）
  ADC_RegularChannelConfig(ADC1, FOC_CURRENT_IA_CHANNEL, 1, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, FOC_CURRENT_IB_CHANNEL, 2, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, FOC_CURRENT_IC_CHANNEL, 3, ADC_SampleTime_56Cycles);
  
  // 禁用注入组（确保不会干扰）
  ADC_InjectedSequencerLengthConfig(ADC1, 0);
  
  // 使能ADC1
  ADC_Cmd(ADC1, ENABLE);
  
  // 等待ADC稳定
  for(volatile int i=0; i<ADC_CALIBRATION_DELAY; i++);
  
  // 启动规则组转换
  ADC_SoftwareStartConv(ADC1);
}

/**
 * @brief 初始化FOC电流和电压采样ADC
 */
void bsp_adc_init(void)
{
  // 初始化电流采集ADC为规则组模式（仅用于校准）
  bsp_adc_init_calibration();
  
  // 校准电流零点
  bsp_adc_calib_current_offset();

  // 先停止当前ADC转换
  ADC_Cmd(ADC1, DISABLE);

  // 开始正式初始化ADC相关
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
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  // 规则组连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;  // 无外部触发（自动连续）
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = REG_CHANNELS;  // 4个电压通道
  ADC1->CR2 |= ADC_CR2_DDS;
  ADC_Init(ADC1, &ADC_InitStructure);

  // 配置规则组通道（电压）
  ADC_RegularChannelConfig(ADC1, FOC_VOLTAGE_VA_CHANNEL, 1, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, FOC_VOLTAGE_VB_CHANNEL, 2, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, FOC_VOLTAGE_VC_CHANNEL, 3, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, FOC_VOLTAGE_VBUS_CHANNEL, 4, ADC_SampleTime_144Cycles);

  // -------------------------- 注入组配置（同步触发） --------------------------
  // 注入组通道数：3个电流通道
  ADC_InjectedSequencerLengthConfig(ADC1, INJ_CHANNELS);
  // 配置注入组通道（电流）
  ADC_InjectedChannelConfig(ADC1, FOC_CURRENT_IA_CHANNEL, 1, ADC_SampleTime_56Cycles);
  ADC_InjectedChannelConfig(ADC1, FOC_CURRENT_IB_CHANNEL, 2, ADC_SampleTime_56Cycles);
  ADC_InjectedChannelConfig(ADC1, FOC_CURRENT_IC_CHANNEL, 3, ADC_SampleTime_56Cycles);
  // 注入组触发源：TIM1 TRGO（更新事件）
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
  ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Rising);  // 上升沿触发

  // 使能注入组转换完成中断（优先级高于规则组）
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // 最高优先级
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

  // 应用电压采集滤波
  adc_apply_filter(v_mode);
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

/**
 * @brief 使用规则组校准电流零点（电机静止时调用，我这里在ADC初始化时处理）
 */
void bsp_adc_calib_current_offset(void)
{
  // 采样多次取平均作为零点偏移
  int32_t ia_sum = 0, ib_sum = 0, ic_sum = 0;
  const uint16_t calib_count = 1000;

  // 廞弃前几个采样值让系统稳定
  for (uint16_t i = 0; i < 100; i++)
  {
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    ADC_GetConversionValue(ADC1);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    ADC_GetConversionValue(ADC1);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    ADC_GetConversionValue(ADC1);
  }
  
  for (uint16_t i = 0; i < calib_count; i++)
  {
    // 等待转换完成
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    // 读取规则组转换结果
    // 注意：由于配置了3个通道，需要连续读取3次
    ia_sum += ADC_GetConversionValue(ADC1);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    ib_sum += ADC_GetConversionValue(ADC1);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    ic_sum += ADC_GetConversionValue(ADC1);
  }
  // 如果采集的数值异常，过大说明电机此时并没有停止或者读取异常，放弃零点校准，采用默认值
  if(fabs(ia_sum - A_REF * calib_count) > 5 * calib_count && fabs(ib_sum - A_REF * calib_count) > 5 * calib_count && fabs(ic_sum - A_REF * calib_count) > 5 * calib_count)
  {
    adc_calib.ia_offset = A_REF;
    adc_calib.ib_offset = A_REF;
    adc_calib.ic_offset = A_REF;
    debug_log("\r\ncalib_current_offset failed : %d %d %d",ia_sum,ib_sum,ic_sum);
  }
  else
  {
    adc_calib.ia_offset = ia_sum / calib_count;
    adc_calib.ib_offset = ib_sum / calib_count;
    adc_calib.ic_offset = ic_sum / calib_count;
    debug_log("\r\ncalib_current_offset success : %d %d %d",adc_calib.ia_offset,adc_calib.ib_offset,adc_calib.ic_offset);
  }
  
}
