#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "stm32f4xx.h"
#include "foc_debug.h"
#include "Config.h"
#include "foc_gather.h"
#include "math.h"
#include "arm_math.h"

// FOC电流采样ADC通道定义
#define FOC_CURRENT_IA_CHANNEL    ADC_Channel_10  // PC0
#define FOC_CURRENT_IB_CHANNEL    ADC_Channel_11  // PC1
#define FOC_CURRENT_IC_CHANNEL    ADC_Channel_12  // PC2

// FOC电压采样ADC通道定义
#define FOC_VOLTAGE_VA_CHANNEL    ADC_Channel_0   // PA0
#define FOC_VOLTAGE_VB_CHANNEL    ADC_Channel_1   // PA1
#define FOC_VOLTAGE_VC_CHANNEL    ADC_Channel_2   // PA2
#define FOC_VOLTAGE_VBUS_CHANNEL  ADC_Channel_13  // PC3

// FOC电流采样GPIO引脚定义
#define FOC_CURRENT_IA_PIN        GPIO_Pin_0
#define FOC_CURRENT_IB_PIN        GPIO_Pin_1
#define FOC_CURRENT_IC_PIN        GPIO_Pin_2
#define FOC_CURRENT_GPIO_PORT     GPIOC

// FOC电压采样GPIO引脚定义
#define FOC_VOLTAGE_VA_PIN        GPIO_Pin_0
#define FOC_VOLTAGE_VB_PIN        GPIO_Pin_1
#define FOC_VOLTAGE_VC_PIN        GPIO_Pin_2
#define FOC_VOLTAGE_VBUS_PIN      GPIO_Pin_3
#define FOC_VOLTAGE_GPIO_PORT_A   GPIOA
#define FOC_VOLTAGE_GPIO_PORT_C   GPIOC

// 注入组（电流通道，高优先级）：IA、IB、IC
#define INJ_CHANNELS 3
#define INJ_IA_CHANNEL ADC_Channel_0  // 示例：IA对应通道0
#define INJ_IB_CHANNEL ADC_Channel_1  // IB对应通道1
#define INJ_IC_CHANNEL ADC_Channel_2  // IC对应通道2

// 规则组（电压通道，连续转换）：VA、VB、VC、VBUS
#define REG_CHANNELS 4
#define REG_VA_CHANNEL ADC_Channel_3
#define REG_VB_CHANNEL ADC_Channel_4
#define REG_VC_CHANNEL ADC_Channel_5
#define REG_VBUS_CHANNEL ADC_Channel_6

// FOC电流采样结构体（使用DSP库整数类型）
typedef struct
{
  q15_t ia;    // A相电流（16位整数，对应int16_t）
  q15_t ib;    // B相电流
  q15_t ic;    // C相电流
  q15_t va;    // A相电压
  q15_t vb;    // B相电压
  q15_t vc;    // C相电压
  q15_t vbus;  // 母线电压
} foc_data_t;

// FOC电流转换结构体（使用DSP库浮点类型）
typedef struct
{
  float32_t ia;    // A相电流（32位浮点）
  float32_t ib;    // B相电流
  float32_t ic;    // C相电流
} foc_data_i;

// FOC电压转换结构体（使用DSP库浮点类型）
typedef struct
{
  float32_t va;    // A相电压
  float32_t vb;    // B相电压
  float32_t vc;    // C相电压
  float32_t vbus;  // 母线电压
} foc_data_v;

// FOC采集模式
typedef enum
{
  v_mode = 0,
  i_mode = 1
} foc_mode;

// 采集电压及母线电压结构体
extern foc_data_v foc_datav;

// 函数声明
void bsp_adc_init(void);
uint8_t bsp_adc_process_data(void);
foc_data_t bsp_adc_get_RAW_Data(void);
foc_data_v bsp_adc_get_calib_data(void);
void bsp_adc_calib_current_offset(void);

#endif /* __BSP_ADC_H */
