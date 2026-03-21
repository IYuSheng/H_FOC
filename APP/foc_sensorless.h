/**
 * @file foc_sensorless.h
 * @brief Sensorless observers for FOC.
 */

#ifndef FOC_SENSORLESS_H
#define FOC_SENSORLESS_H

#include <math.h>
#include "Config.h"

typedef struct AlphaBetaTypeDef AlphaBetaTypeDef;

typedef struct {
    float x1;
    float x2;
    float cos_theta;
    float sin_theta;
    float omega_filt;
    float eta1;
    float eta2;
    float theta_hat;
    float theta;
    float omega_integ;
} FluxObserver_t;

extern FluxObserver_t g_flux_obs;

#define FLUX_PHI_M                MOTOR_FLUX_LINKAGE
#define _1_FLUX_PHI_M             (1.0f / MOTOR_FLUX_LINKAGE)
#define FLUX_PHI_M_2              (MOTOR_FLUX_LINKAGE * MOTOR_FLUX_LINKAGE)
#define FLUX_GAMMA                1000000000.0f
#define FLUX_GAMMA_K              (0.5f * FLUX_GAMMA)
#define FLUX_L_S                  MOTOR_INDUCTANCE
#define FLUX_R_S                  MOTOR_RESISTANCE
#define FLUX_ANGLE_OFFSET_COMP    0.0f
#define FLUX_ANGLE_OFFSET_ACTURE  (_PI_2 - FLUX_ANGLE_OFFSET_COMP)

#define PLL_BANDWIDTH_HZ          1000.0f
#define PLL_KP                    (2.0f * 0.707f * 6.2831853f * PLL_BANDWIDTH_HZ)
#define PLL_KI                    ((6.2831853f * PLL_BANDWIDTH_HZ) * (6.2831853f * PLL_BANDWIDTH_HZ))
#define MAX_SPEED                 3000.0f

void flux_observer_init(FluxObserver_t *obs);
void flux_observer_update(FluxObserver_t *obs, float u_alpha, float u_beta,
                          float i_alpha, float i_beta, float Ts);

/* SMO observer */

typedef struct {
    float Rs;
    float Ls;
    float Ts;
    uint16_t POLES;
    float Fsmopos;
    float Gsmopos;
} SMO_MotorPare_t;

typedef struct {
    float EstIalpha;
    float EstIbeta;
    float IalphaError;
    float IbetaError;
    float E0;
    float Zalpha;
    float Zbeta;
    float Ealpha;
    float Ebeta;
    float Theta;
    float Theta_pre;
    float Err;
    float Interg;
    float Ui;
    float Speed_Rpm;
    float SpeedLpf_Rpm;
    float Kslide;
    float Kslf_emf;
    struct {
        float Kp;
        float Ki;
        float Speed_coeff;
        float Kslf;
    } tPll;
} Ppll_obj_t;

extern SMO_MotorPare_t SMO_MotorPare;
extern Ppll_obj_t Angle_SMOPare;

#define SMO_SLIDE_GAIN_FACTOR      100.0f
#define SMO_EMF_FILTER_COEFF       0.10f
#define SMO_CURRENT_ERR_BAND       1.5f
#define SMO_PLL_KP                 1500.0f
#define SMO_PLL_KI                 1500.0f
#define SMO_PLL_SPEED_FILTER_COEFF 0.5f
#define SMO_PLL_INT_LIMIT          (2000)
#define SMO_ANGLE_COMPENSATION     0.0f

void SMO_Pare_init(void);
void SMO_Reset(void);
float32_t SMO_bemf_angle(AlphaBetaTypeDef *alpha_beta);
float32_t SMO_bemf_angle_from_voltage_current(float32_t u_alpha, float32_t u_beta,
                                              float32_t i_alpha, float32_t i_beta);

/* HFI observer */

typedef struct {
    float u_inject;
    float u_inject_d;
    uint8_t inject_flag;
    uint8_t init_done;

    float i_alpha_last[2];
    float i_beta_last[2];

    float i_alpha_h;
    float i_beta_h;
    float i_alpha_h_last;
    float i_beta_h_last;

    float i_env_alpha;
    float i_env_beta;

    float i_d_fund;
    float i_q_fund;

    uint16_t nsd_count;
    float nsd_target_d;
    float sum_pos;
    float sum_neg;
    uint8_t polarity_determined;

    uint8_t state;
    uint32_t converge_cnt;
} HFI_Observer_t;

typedef struct {
    float theta_est;
    float omega_est;
    float omega_est_filt;
    float kp;
    float ki;
    float error;
    float p_term;
    float i_term;
} HFI_PLL_t;

typedef enum {
    HFI_STATE_CONVERGE,
    HFI_STATE_NSD,
    HFI_STATE_RUN
} HFI_State_t;

extern HFI_Observer_t g_hfi_obs;
extern HFI_PLL_t g_hfi_pll;

#define HFI_INJECT_VOLTAGE      1.5f
#define HFI_PLL_BANDWIDTH_HZ    10.0f
#define HFI_PLL_KP              (2.0f * 0.707f * 6.2831853f * HFI_PLL_BANDWIDTH_HZ)
#define HFI_PLL_KI              ((6.2831853f * HFI_PLL_BANDWIDTH_HZ) * (6.2831853f * HFI_PLL_BANDWIDTH_HZ))
#define HFI_NSD_ENABLE          1

void hfi_observer_init(HFI_Observer_t *obs, float u_inject);
void hfi_observer_update(HFI_Observer_t *obs, float i_alpha, float i_beta);
void hfi_extract_fundamental(HFI_Observer_t *obs, float i_d, float i_q,
                             float *i_d_fund, float *i_q_fund);
float hfi_get_inject_voltage(HFI_Observer_t *obs);

void hfi_pll_init(HFI_PLL_t *pll, float kp, float ki);
void hfi_pll_update(HFI_PLL_t *pll, HFI_Observer_t *obs, float Ts);
void hfi_nsd_check(HFI_Observer_t *obs, HFI_PLL_t *pll, float i_d, float i_q);

#endif
