#ifndef __FOC_CONVERSION_H
#define __FOC_CONVERSION_H

#include "stm32f4xx.h"
#include "Config.h"
#include "bsp_adc.h"
#include "foc_control.h"
#include "arm_math.h"

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
  float32_t omega;            // 电角速度
  float32_t target_speed;     // 目标速度(rpm)
  float32_t target_position;  // 目标位置（弧度）
  struct
  {
    float32_t current_d;     // 实际D轴电流
    float32_t current_q;     // 实际Q轴电流
    float32_t current_d_filt;
    float32_t current_q_filt;
  }abc_dq;  // 经采集变换后的实际dq轴电流
} foc_control_t;

// PI控制器结构体
typedef struct {
    float32_t kp;           // 比例增益
    float32_t ki;           // 积分增益
    float32_t integral;     // 积分项
    float32_t integral_limit; // 积分限幅
    float32_t error;        // 当前误差
    float32_t output;       // 输出值
    float32_t target;       // 目标位置
    float32_t current;      // 当前位置
} pi_t;

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
    float Ealpha;       // α轴扩展反电动势
    float Ebeta;        // β轴扩展反电动势

    // PLL与控制参数
    float Theta;        // 当前电角度
    float Theta_pre;    // 补偿后电角度
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

typedef struct {
    // 二阶巴特沃斯滤波器系数
    float b0, b1, b2;  // 分子系数
    float a1, a2;      // 分母系数（a0归一化为1）
    
    // 历史值
    float x[2];        // 输入历史 [n-1, n-2]
    float y[2];        // 输出历史 [n-1, n-2]
    
    float cutoff_freq; // 截止频率(Hz)
    float sample_freq; // 采样频率(Hz)
} ButterworthLPF_t;

extern foc_control_t foc_ctrl;
extern SVPWM_t svpwm;
extern AlphaBetaTypeDef alpha_beta;
extern Ppll_obj_t Angle_SMOPare;
extern SMO_MotorPare_t SMO_MotorPare;

// FOC变换相关函数声明

/**
 * @brief 二阶巴特沃斯低通滤波器初始化
 * @param filt 滤波器结构体指针
 * @param cutoff_freq 截止频率(Hz)
 * @param sample_freq 采样频率(Hz)
 * 
 * 传递函数：H(s) = ωc? / (s? + √2·ωc·s + ωc?)
 */
void butterworth_init(ButterworthLPF_t *filt, float cutoff_freq, float sample_freq);

/**
 * @brief 二阶巴特沃斯滤波器更新
 * @param filt 滤波器结构体指针
 * @param input 当前输入值
 * @return 滤波后的输出值
 * 
 * 差分方程：y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
 */
float butterworth_filter(ButterworthLPF_t *filt, float input);

/**
 * @brief 电角度归一化（映射到0~2π范围）
 * @param angle 输入电角度（rad，范围无限制）
 * @return 归一化后电角度（rad，0~2π）
 */
extern inline float32_t angle_normalize(float32_t angle);

/**
 * @brief 电角度归一化（映射到-π~π范围）
 * @param angle 输入电角度（rad，范围无限制）
 * @return 归一化后电角度（rad，-π~π）
 */
extern inline float32_t angle_normalize_pi(float32_t angle);

/**
 * @brief 电角度归一化（映射到0~360范围）
 * @param angle 输入电角度（°，范围无限制）
 * @return 归一化后电角度（°，0~360）
 */
extern inline float32_t angle_normalize_360(float32_t angle);

/**
 * @brief 角度制转弧度制
 * @param deg 输入角度（°）
 * @return 转换后弧度值（rad）
 */
extern inline float deg2rad(float deg);

/**
 * @brief 弧度制转角度制
 * @param rad 输入弧度（rad）
 * @return 转换后角度值（°）
 */
extern inline float rad2deg(float rad);

/**
 * @brief SVPWM通用扇区判断函数
 * @param alpha_beta αβ坐标系下的电压指针
 * @return 扇区编号（1~6，对应0~60°~360°）
 */
extern inline uint8_t svpwm_sector_calc(AlphaBetaTypeDef *alpha_beta);

/**
 * @brief SVPWM基本矢量作用时间计算
 * @param alpha_beta αβ坐标系下的电压指针
 * @param svpwm SVPWM结构体指针
 * @param vdc 母线电压（V）
 */
extern inline void svpwm_calc_times(AlphaBetaTypeDef *alpha_beta, SVPWM_t *svpwm, float32_t vdc);

/**
 * @brief SVPWM计算三相导通时间并转换为比较值
 * @param svpwm SVPWM结构体指针
 */
extern inline void svpwm_duty_calc(SVPWM_t *svpwm);

/**
 * @brief 反Park变换
 * @param d_ptr D轴输入指针
 * @param q_ptr Q轴输入指针
 * @param alpha_ptr Alpha轴输出指针
 * @param beta_ptr Beta轴输出指针
 * @param angle 电角度(角度, 0-360度)
 */
extern inline void inv_park_transform_f32(foc_control_t *foc_ctrl, AlphaBetaTypeDef *alpha_beta, float32_t sin_theta, float32_t cos_theta);

/**
 * @brief 将三相电压电流转换为αβ坐标系下的值
 * @param abc_i 三相电流指针
 * @param abc_v 三相电压指针
 * @param alpha_beta 输出的αβ轴值
 */
extern inline void clark_transform(void *abc_i, void *abc_v, AlphaBetaTypeDef *alpha_beta);

/**
 * @brief 将αβ转换为dq坐标系下的电流值
 * @param current_abc αβ电流值
 * @param current_αβ 输出的dq轴电流值
 */
inline void park_transform(AlphaBetaTypeDef *alpha_beta, foc_control_t *foc_ctrl, float32_t sin_theta, float32_t cos_theta);

/**
 * @brief 将三相电流转换为dq坐标系下的电流值
 * @param current_abc_ptr 三相电流值指针 (ia, ib, ic)
 * @param foc_ctrl FOC控制结构体指针
 * @param angle 电角度(弧度)
 */
extern inline void abc_to_dq_current(void *current_abc_ptr, foc_control_t *foc_ctrl, float angle);

/**
 * @brief D轴电流环PID计算
 * @param target_id 目标D轴电流
 * @param actual_id 实际D轴电流
 * @return D轴电压输出
 */
extern inline float32_t foc_id_pid_calculate(float32_t target_id, float32_t actual_id);

/**
 * @brief Q轴电流环PID计算
 * @param target_iq 目标Q轴电流
 * @param actual_iq 实际Q轴电流
 * @return Q轴电压输出
 */
extern inline float32_t foc_iq_pid_calculate(float32_t target_iq, float32_t actual_iq);

/**
 * @brief 速度环PID计算
 * @param target_speed 目标速度(RPM)
 * @param actual_speed 实际速度(RPM)
 * @return 输出值(Q轴电流)
 */
extern inline float32_t foc_speed_pid_calculate(float32_t target_speed, float32_t actual_speed);

/**
 * @brief 位置环PI控制器计算
 * @param target_position 目标位置（弧度）
 * @param current_position 当前位置（弧度）
 * @return PI控制器输出（速度指令，rad/s）
 */
extern inline float32_t foc_position_pid_calculate(float32_t target_position, float32_t current_position);

/**
 * @brief SMO参数初始化
 */
void SMO_Pare_init(void);

/**
 * @brief SMO扩展反电动势估算角度计算
 * @param alpha_beta alpha_beta坐标系下的电压电流值指针
 * @return 估算的电角度(弧度)
 */
extern inline float32_t SMO_bemf_angle(AlphaBetaTypeDef *alpha_beta);

#endif /* __FOC_CONVERSION_H */
