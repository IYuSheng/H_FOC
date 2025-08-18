#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "stm32f4xx.h"
#include "foc_debug.h"

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

// FOC电流采样结构体
typedef struct
{
  uint16_t ia;    // A相电流
  uint16_t ib;    // B相电流
  uint16_t ic;    // C相电流
  uint16_t va;    // A相电压
  uint16_t vb;    // B相电压
  uint16_t vc;    // C相电压
  uint16_t vbus;  // 母线电压
} foc_data_t;

// 函数声明
void bsp_adc_init(void);
void bsp_adc_start_conversion(void);
foc_data_t bsp_adc_get_RAW_Data(void);

#endif /* __BSP_ADC_H */
