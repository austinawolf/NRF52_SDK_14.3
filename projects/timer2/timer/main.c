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
 * @defgroup pin_change_int_example_main main.c
 * @{
 * @ingroup pin_change_int_example
 * @brief Pin Change Interrupt Example Application main file.
 *
 * This file contains the source code for a sample application using interrupts triggered by GPIO pins.
 *
 */


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "boards.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_power.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"

#define ON 1
#define OFF 0
#define PIN_OUT BSP_LED_0
#define PERIOD 1000
#define SAMPLES_IN_BUFFER 5


volatile uint32_t led_state = OFF;
volatile uint32_t time_us = 500; //Time(in miliseconds) between consecutive compare events.

volatile uint8_t state = 1;
static const nrf_drv_timer_t led_timer = NRF_DRV_TIMER_INSTANCE(0);





void timer_init(void);
void gpio_init(void);
void timer_set(uint32_t _time_us);




void led_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            if (led_state == ON) {
							led_state = OFF;
							nrf_drv_gpiote_out_set(PIN_OUT);
							timer_set(PERIOD - time_us);
						}
						else {
							led_state = ON;
							nrf_drv_gpiote_out_clear(PIN_OUT);
							timer_set(time_us);
						} 
            break; 

        default:
            //Do nothing.
            break;
    }
}

void timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}


int main(void)
{

		
		//setup
	  uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    err_code = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(err_code);

    ret_code_t ret_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(ret_code);

	  NRF_LOG_INFO("SAADC HAL simple example.");
		nrf_delay_ms(1000);
	


	  gpio_init();
		NRF_LOG_INFO("GPIO INIT...done");
		timer_init();
		NRF_LOG_INFO("TIMER INIT...done");
		time_us = 5;
		timer_set( (uint32_t) 5 );
		NRF_LOG_INFO("TIMER SET...done");
		
 
    while (1)
    {
				NRF_LOG_INFO("LOOP");
				nrf_delay_ms(1000);
				time_us = 100;
				nrf_delay_ms(1000);
				time_us = 700;
        NRF_LOG_FLUSH();
    }
}



void gpio_init(void) {
		//Configure all leds on board.
    ret_code_t err_code;
		err_code = nrf_drv_gpiote_init();
		APP_ERROR_CHECK(err_code);
	
		nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
    APP_ERROR_CHECK(err_code);
}

void timer_init(void) {

    //Configure led_timer for generating simple light effect - leds on board will invert his state one after the other.
    uint32_t err_code = NRF_SUCCESS;
		
		nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
		timer_cfg.bit_width = 0;
    err_code = nrf_drv_timer_init(&led_timer, &timer_cfg, led_timer_event_handler);
    NRF_LOG_INFO("Err Code: %d", err_code);
		APP_ERROR_CHECK(err_code);

    nrf_drv_timer_enable(&led_timer);
}

void timer_set(uint32_t _time_us) {
		//set timer
		uint32_t time_ticks;
		time_ticks = nrf_drv_timer_us_to_ticks(&led_timer, _time_us);
    nrf_drv_timer_extended_compare(&led_timer, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}

