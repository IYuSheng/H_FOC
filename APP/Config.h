#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm32f4xx.h"
#include <arm_math.h>

// --------------------------- 模式选项 --------------------------
#define DEBUG_MODE  0

// -------------------------- 常用常量定义 --------------------------
#define _PI         PI
#define _2PI        (2.0f * PI)
#define _60_angle   2.0f * PI / 6.0f  // 60度对应的弧度
#define _SQRT3      1.732050807568877f
#define _SQRT3_2    0.866025403784439f  // sqrt(3)/2
#define _1_SQRT3    0.577350269189626f  // 1/sqrt(3)
#define _2_SQRT3    1.154700538379252f  // 2/sqrt(3)
#define SPEED_FACTOR (_2PI / 60.0f * MOTOR_POLE_PAIRS) // 速度转换系数 (机械转速rpm → 电角速度rad/s)
#define RAD_TO_DEG  (180.0f / PI)
#define FACTOR       _SQRT3 / PWM_FREQ  // SVPWM时间中间计算系数

// -------------------------- 电源与硬件限制 --------------------------
#define VOLTAGE_LIMIT        3.3f    // 直流母线电压限制 (V)
#define CURRENT_LIMIT        1.0f    // 最大相电流限制 (A)
#define OVER_CURRENT_THRESH  0.6f    // 过流保护阈值 (A)，需大于CURRENT_LIMIT
#define OVER_VOLTAGE_THRESH  3.6f    // 过压保护阈值 (V)
#define UNDER_VOLTAGE_THRESH 2.8f   // 欠压保护阈值 (V)
#define VOLTAGE_AMPLITUDE_LIMIT    0.6f   // 输出电压限制(%),这里设置输出电压不能高于母线电压的60%

// -------------------------- 电机参数 (根据实际电机填写) --------------------------
#define MOTOR_POLE_PAIRS    15       // 电机极对数 (例如：14极电机为7对)
#define MOTOR_RESISTANCE    0.5f    // 相电阻 (Ohm)，需实测或手册值
#define MOTOR_INDUCTANCE    0.002f  // 相电感 (H)，需实测或手册值
#define MAX_SPEED_RPM       3000    // 最大转速限制 (RPM)
#define ENCODER_RESOLUTION  1024    // 编码器分辨率 (线数)

// -------------------------- FOC控制参数 --------------------------
#define PWM_FREQ            10000.0f   // PWM频率 (Hz)，需与定时器配置匹配
#define HALF_PWM_FREQ       PWM_FREQ * 0.5f   // PWM频率 (Hz)，需与定时器配置匹配
#define PWM_PERIOD_S        1.0f / PWM_FREQ  // PWM周期（单位：s），与PWM_FREQ对应：T = 1/F
#define SVPWM_VOLT_COEF   (2.0f / sqrtf(3.0f)) // SVPWM基本矢量幅值系数（母线电压相关，固定值）

// PI调节器参数 (初始值，需根据电机调试优化)
// 速度环PI
#define SPEED_P_GAIN        5.0f    // 比例增益
#define SPEED_I_GAIN        0.2f    // 积分增益
#define SPEED_I_LIMIT       1.0f    // 积分限幅

// d轴电流环PI (励磁分量)
#define ID_P_GAIN           8.0f
#define ID_I_GAIN           0.5f
#define ID_I_LIMIT          0.8f

// q轴电流环PI (转矩分量)
#define IQ_P_GAIN           8.0f
#define IQ_I_GAIN           0.5f
#define IQ_I_LIMIT          0.8f

// -------------------------- 电流采样配置 --------------------------
#define ADC_REF_VOLTAGE     3.3f    // ADC参考电压 (V)
#define ADC_MAX_VALUE       4096.0f    // ADC分辨率 (例如：12位ADC为4096)
#define INA240_GAIN         20.0f   // 电流采样运放增益
#define V_REF               1.65f    // REF1 引脚电压3.3V，单位：V
#define A_REF               2048
#define R_Current           0.001f    // 采样电阻阻值 (1mΩ)

// -------------------------- 电压采样配置 --------------------------
#define R_Voaltage_1  56.0f   // 56KΩ
#define R_Voaltage_2  2.2f    // 2.2KΩ

// -------------------------- 控制模式定义 --------------------------
typedef enum
{
  FOC_MODE_TORQUE = 0,    // 转矩模式 (直接控制IQ)
  FOC_MODE_SPEED,         // 速度模式 (闭环控制转速)
  FOC_MODE_POSITION       // 位置模式 (闭环控制角度)
} FOC_ModeTypeDef;

#define DEFAULT_CONTROL_MODE FOC_MODE_SPEED  // 默认控制模式

// -------------------------- 故障状态定义 --------------------------
typedef enum
{
  FAULT_NONE = 0,         // 无故障
  FAULT_OVER_CURRENT,     // 过流
  FAULT_OVER_VOLTAGE,     // 过压
  FAULT_UNDER_VOLTAGE,    // 欠压
  FAULT_BREAK,            // 刹车触发
  FAULT_ENCODER           // 编码器故障
} FaultTypeDef;

#endif /* __CONFIG_H */
