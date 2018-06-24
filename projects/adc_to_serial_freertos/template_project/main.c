/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example_freertos
 *
 * @brief Blinky FreeRTOS Example Application main file.
 *
 * This file contains the source code for a sample application using FreeRTOS to blink LEDs.
 *
 */



#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "boards.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_power.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_saadc.h"
#include "adc_helper.h"
#include "log_helper.h"

#if LEDS_NUMBER <= 2
#error "Board is not equipped with enough amount of LEDs"
#endif



#define SAMPLE_PERIOD      50          /**< Timer period. LED1 timer will expire after 1000 ms */

TaskHandle_t  led_toggle_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
TaskHandle_t log_flush_task_handle;
TimerHandle_t adc_sample_timer_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */

uint32_t led0_delay = 50;

void task_init(void);
uint32_t get_adc_value(void);

static void led_toggle_task_function (void * pvParameter);
static void log_flush_task_function (void * pvParameter);

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
		
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        NRF_LOG_INFO("ADC event number: %d", (int)adc_evt_counter);

				int32_t adc = 0;
        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            NRF_LOG_INFO("%d", p_event->data.done.p_buffer[i]);
						adc += ((p_event->data.done.p_buffer[i]) / (SAMPLES_IN_BUFFER));
        }

				adc_time_av = adc;
								
        adc_evt_counter++;
    }
}

static void led_toggle_task_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    while (true)
    {
        bsp_board_led_invert(BSP_BOARD_LED_0);

        /* Delay a task for a given number of ticks */
        vTaskDelay(get_adc_value());

        /* Tasks must be implemented to never return... */
    }
		
}

static void log_flush_task_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    while (true)
    {
        NRF_LOG_FLUSH();
        /* Tasks must be implemented to never return... */
    }
		
}


static void adc_sample_timer_callback (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
		nrf_drv_saadc_sample();
}




int main(void)
{
		//Log Setup
	  logger_init();
	
		//Run General Setup
		saadc_init();
		saadc_sampling_event_init();
		task_init();
		
    while (true)
    {

    }
}

/**
 *@}
 **/

void task_init(void)
{
	  ret_code_t err_code;

		
    /* Initialize clock driver for better time accuracy in FREERTOS */
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    /* Configure LED-pins as outputs */
    bsp_board_leds_init();

    /* Create task for LED0 blinking with priority set to 3 */
    UNUSED_VARIABLE(xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task_handle));
	
    /* Create task for LED0 blinking with priority set to 1 */
		UNUSED_VARIABLE(xTaskCreate(log_flush_task_function, "LOG_FLUSH", configMINIMAL_STACK_SIZE + 200, NULL, 1, &log_flush_task_handle));

    /* Start timer for LED1 blinking */
    adc_sample_timer_handle = xTimerCreate( "LED1", SAMPLE_PERIOD, pdTRUE, NULL, adc_sample_timer_callback);
    UNUSED_VARIABLE(xTimerStart(adc_sample_timer_handle, 0));

    /* Activate deep sleep mode */
    //SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();
}

uint32_t get_adc_value(void) {
	int32_t adc = get_adc_time_av();
	if (adc > 1000) return 1000;
	else if (adc < 50) return 50;
	else return adc;
}
