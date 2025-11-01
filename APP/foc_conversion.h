#ifndef __FOC_CONVERSION_H
#define __FOC_CONVERSION_H

#include "stm32f4xx.h"
#include "Config.h"
#include "bsp_adc.h"
#include "foc_control.h"

// 两相静止坐标系结构体 (Alpha-Beta)
typedef struct
{
  // 输出dq转换
  float alpha;
  float beta;
  // 输入abc转换
  float alpha_i;
  float beta_i;
  // 输入abc转换
  float alpha_v;
  float beta_v;
} AlphaBetaTypeDef;

// SVPWM结构体参数定义
typedef struct
{
  float32_t T0;
  float32_t T1;
  float32_t T2;
  uint8_t sector;
  uint16_t pwm_a;
  uint16_t pwm_b;
  uint16_t pwm_c;
} SVPWM_t;

// FOC控制参数结构体
typedef struct
{
  float32_t out_d;            // 输出D轴电压
  float32_t out_q;            // 输出Q轴电压
  float32_t target_d;         // 目标D轴电流
  float32_t target_q;         // 目标Q轴电流
  float32_t angle;            // 电角度(弧度)
  float32_t speed;            // 速度（rad/s）
  float32_t speed_rpm;        // 速度（rpm）
  float32_t target_speed;     // 目标速度(rpm)
  float32_t target_position;  // 目标位置（弧度）
  struct
  {
    float32_t current_d;     // 实际D轴电流
    float32_t current_q;     // 实际Q轴电流
  }abc_dq;  // 经采集变换后的实际dq轴电流
} foc_control_t;

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

// 无感SMO电机参数结构体
typedef struct {
    float Rs;          // 电机定子电阻（计算Gsmopos、电流估计用）
    float Ls;          // 电机定子电感（计算Fsmopos用）
    float Ib;          // 基准电流（计算Gsmopos用）
    float Vb;          // 基准电压（计算Gsmopos用）
    float Ts;          // 控制周期（电流估计、角度更新用）
    uint16_t POLES;    // 电机极对数（转速计算、角度更新用）
    float Fsmopos;     // 状态转移矩阵（电流估计用）
    float Gsmopos;     // 观测器增益（电流估计用）
} SMO_MotorPare_t;

// PLL与SMO控制参数结构体
typedef struct {
    // 滑模观测器核心变量
    float EstIalpha;    // α轴估算电流
    float EstIbeta;     // β轴估算电流
    float IalphaError;  // α轴电流误差
    float IbetaError;   // β轴电流误差
    float E0;           // 电流误差饱和阈值
    float Zalpha;       // α轴滑模控制量
    float Zbeta;        // β轴滑模控制量
    float Ealpha;       // α轴反电动势
    float Ebeta;        // β轴反电动势

    // PLL与控制参数
    float Theta;        // 当前电角度
    float Err;          // PLL相位误差
    float Interg;       // PLL积分项
    float Ui;           // PLL输出电角速度
    float Speed_Rpm;    // 未滤波转速
    float SpeedLpf_Rpm; // 滤波后转速
    float Kslide;       // 滑模增益
    float Kslf_emf;     // 反电动势滤波系数
    struct {
        float Kp;       // PLL比例增益
        float Ki;       // PLL积分增益
        float Speed_coeff; // 转速转换系数
        float Kslf;     // PLL转速滤波系数
    } tPll;
} Ppll_obj_t;

extern Ppll_obj_t Angle_SMOPare;
extern position_pid_t position_pid;

// FOC变换相关函数声明
extern inline float32_t angle_normalize(float32_t angle);
// SVPWM相关函数声明
extern inline uint8_t svpwm_sector_calc(AlphaBetaTypeDef *alpha_beta);
extern inline void svpwm_calc_times(AlphaBetaTypeDef *alpha_beta, SVPWM_t *svpwm, float32_t vdc);
extern inline void svpwm_duty_calc(SVPWM_t *svpwm);
extern inline void inv_park_transform_f32(foc_control_t *foc_ctrl, AlphaBetaTypeDef *alpha_beta, float32_t angle);

// Clarke变换(精确)
extern inline void clark_transform(void *abc_i, void *abc_v, AlphaBetaTypeDef *alpha_beta);
// park变换(DSP库快速)
extern inline void park_transform(AlphaBetaTypeDef *alpha_beta, foc_control_t *foc_ctrl);
// Clarke+Park变换(DSP库快速)
extern inline void abc_to_dq_current(void *current_abc_ptr, foc_control_t *foc_ctrl, float angle);
// 电流环PID控制器计算
extern inline float32_t foc_id_pid_calculate(float32_t target_id, float32_t actual_id);
extern inline float32_t foc_iq_pid_calculate(float32_t target_iq, float32_t actual_iq);
// 速度环PID控制器计算
extern inline float32_t foc_speed_pid_calculate(float32_t target_speed, float32_t actual_speed);
// 位置环PID控制器计算
extern inline float32_t foc_position_pid_calculate(float32_t target_position, float32_t current_position);

/*无感部分*/
void  SMO_Pare_init(void);
extern inline float32_t SMO_bemf_angle(AlphaBetaTypeDef *alpha_beta);
void bsp_adc_get_bemf(float32_t* alpha, float32_t* beta);

#endif /* __FOC_CONVERSION_H */
