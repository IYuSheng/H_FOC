#ifndef __FOC_CONVERSION_H
#define __FOC_CONVERSION_H

#include "stm32f4xx.h"
#include "Config.h"
#include "bsp_adc.h"
#include "arm_math.h"

typedef struct AlphaBetaTypeDef
{
    float alpha;
    float beta;
    float alpha_i;
    float beta_i;
    float alpha_v;
    float beta_v;
} AlphaBetaTypeDef;

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

typedef struct
{
    float32_t out_d;
    float32_t out_q;
    float32_t target_d;
    float32_t target_q;
    float32_t angle;
    float32_t speed;
    float32_t speed_rpm;
    float32_t omega;
    float32_t target_speed;
    float32_t target_position;
    struct
    {
        float32_t current_d;
        float32_t current_q;
        float32_t current_d_filt;
        float32_t current_q_filt;
    } abc_dq;
} foc_control_t;

typedef struct {
    float32_t kp;
    float32_t ki;
    float32_t integral;
    float32_t integral_limit;
    float32_t error;
    float32_t output;
    float32_t target;
    float32_t current;
} pi_t;

typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float x[2];
    float y[2];
    float cutoff_freq;
    float sample_freq;
} ButterworthLPF_t;

extern foc_control_t foc_ctrl;
extern SVPWM_t svpwm;
extern AlphaBetaTypeDef alpha_beta;

void butterworth_init(ButterworthLPF_t *filt, float cutoff_freq, float sample_freq);
float butterworth_filter(ButterworthLPF_t *filt, float input);

float32_t angle_normalize(float32_t angle);
float32_t angle_normalize_pi(float32_t angle);
float32_t angle_normalize_360(float32_t angle);
float deg2rad(float deg);
float rad2deg(float rad);

uint8_t svpwm_sector_calc(AlphaBetaTypeDef *alpha_beta);
void svpwm_calc_times(AlphaBetaTypeDef *alpha_beta, SVPWM_t *svpwm, float32_t vdc);
void svpwm_duty_calc(SVPWM_t *svpwm);

void inv_park_transform_f32(foc_control_t *foc_ctrl, AlphaBetaTypeDef *alpha_beta,
                            float32_t sin_theta, float32_t cos_theta);
void clark_transform(void *abc_i, void *abc_v, AlphaBetaTypeDef *alpha_beta);
void park_transform(AlphaBetaTypeDef *alpha_beta, foc_control_t *foc_ctrl,
                    float32_t sin_theta, float32_t cos_theta);
void abc_to_dq_current(void *current_abc_ptr, foc_control_t *foc_ctrl, float angle);

float32_t foc_id_pid_calculate(float32_t target_id, float32_t actual_id);
float32_t foc_iq_pid_calculate(float32_t target_iq, float32_t actual_iq);
float32_t foc_speed_pid_calculate(float32_t target_speed, float32_t actual_speed);
float32_t foc_position_pid_calculate(float32_t target_position, float32_t current_position);

#endif /* __FOC_CONVERSION_H */
