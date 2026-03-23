#ifndef __FOC_CONTROL_H
#define __FOC_CONTROL_H

#include "stm32f4xx.h"
#include "freertos.h"
#include "task.h"
#include "arm_math.h"
#include "foc_encoder.h"
#include "bsp_timer.h"
#include "Config.h"
#include "foc_debug.h"
#include "foc_gather.h"
#include "foc_conversion.h"
#include "foc_settings.h"
#include "foc_sensorless.h"
#include "foc_parameter_ident.h"

#define FLUX_OBSERVER_ENABLE                1
#define SMO_OBSERVER_ENABLE                 0
#define FOC_PARAMETER_IDENTIFICATION_ENABLE 0
#define HFI_ENABLE                          0
#define HFI_STANDALONE_MODE                 1

typedef enum {
    SENSORLESS_STATE_HFI,
    SENSORLESS_STATE_MIX,
    SENSORLESS_STATE_FLUX,
} sensorless_state_t;

static enum {
    SENSOR_HALL = 0,
    SENSOR_BLEND,
    SENSOR_OBSERVER
} sensor_state_t;

void foc_control_init(void);
void foc_control(void);
void foc_control_out(void);
void vFOCControlTask(void *pvParameters);

#endif /* __FOC_CONTROL_H */
