
#ifndef NRF_ADC_HELPER_H_
#define NRF_ADC_HELPER_H_

#include <stdbool.h>
#include <stddef.h>
#include "nrf.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_drv_saadc.h"
#include "nrf_saadc.h"
#include "boards.h"
#include "FreeRTOS.h"
#include "queue.h"


#define SAMPLES_IN_BUFFER 5

void saadc_init(void);
void saadc_sampling_event_init(void);
int32_t get_adc_time_av(void);
void saadc_callback(nrf_drv_saadc_evt_t const * p_event);

static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static uint32_t              adc_evt_counter;
static int32_t							 adc_time_av = 500;
#endif
