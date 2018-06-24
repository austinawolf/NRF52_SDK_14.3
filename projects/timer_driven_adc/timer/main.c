/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
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
 * @defgroup nrf_dev_timer_example_main main.c
 * @{
 * @ingroup nrf_dev_timer_example
 * @brief Timer Example Application main file.
 *
 * This file contains the source code for a sample application using Timer0.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "nordic_common.h"
#include "boards.h"
#include "nrf_drv_timer.h"
#include "bsp.h"

#define ON 1
#define OFF 0
#define LED_PERIOD 50

uint32_t time_ms = 1; //Time(in miliseconds) between consecutive compare events.
uint32_t time_ticks;
bool led_state = OFF;

const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(0);

/**
 * @brief Handler for timer events.
 */
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context)
{
		if (led_state) {
			led_state = OFF;
			nrf_drv_gpiote_out_set(BSP_LED_0);
			time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_LED, time_ms);
			nrf_drv_timer_extended_compare(&TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
		}
		else {
			led_state = ON;
			nrf_drv_gpiote_out_clear(BSP_LED_0);
			time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_LED, LED_PERIOD - time_ms);
			nrf_drv_timer_extended_compare(&TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
		}
}

static void gpio_init(void)
{
	
	ret_code_t err_code;
	
	//configure and initialize output pins
	err_code = nrf_drv_gpiote_init();
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
	err_code = nrf_drv_gpiote_out_init(BSP_LED_0, &out_config);
	APP_ERROR_CHECK(err_code);

}

static void timer_init(void) {
	uint32_t err_code = NRF_SUCCESS;



	//Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
	nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
	err_code = nrf_drv_timer_init(&TIMER_LED, &timer_cfg, timer_led_event_handler);
	APP_ERROR_CHECK(err_code);

	time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_LED, time_ms);
	nrf_drv_timer_extended_compare(&TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

	nrf_drv_timer_enable(&TIMER_LED);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    gpio_init();
		timer_init();

		int i = 0;
    while (1)
    {
			for(i = 0; i < 5; i++) {
				time_ms = i * 40;
				nrf_delay_ms(500);
				
			}
				
				
				
    }
}

/** @} */
