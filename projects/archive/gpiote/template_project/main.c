/**
 * Copyright (c) 2009 - 2017, Nordic Semiconductor ASA
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
* @brief This program toggles an LED when there is a pin change on its corresponding button..
* @defgroup
*
*/

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "nordic_common.h"
#include "boards.h"

#define PIN_IN_0 BSP_BUTTON_0
#define PIN_IN_1 BSP_BUTTON_1
#define PIN_IN_2 BSP_BUTTON_2
#define PIN_IN_3 BSP_BUTTON_3

#define PIN_OUT_0 BSP_LED_0
#define PIN_OUT_1 BSP_LED_1
#define PIN_OUT_2 BSP_LED_2
#define PIN_OUT_3 BSP_LED_3

void in_pin_handler_0(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if ( nrf_drv_gpiote_in_is_set(PIN_IN_0) ) nrf_drv_gpiote_out_set(PIN_OUT_0);
	else nrf_drv_gpiote_out_clear(PIN_OUT_0);
}

void in_pin_handler_1(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if ( nrf_drv_gpiote_in_is_set(PIN_IN_1) ) nrf_drv_gpiote_out_set(PIN_OUT_1);
	else nrf_drv_gpiote_out_clear(PIN_OUT_1);
}

void in_pin_handler_2(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if ( nrf_drv_gpiote_in_is_set(PIN_IN_2) ) nrf_drv_gpiote_out_set(PIN_OUT_2);
	else nrf_drv_gpiote_out_clear(PIN_OUT_2);
}

void in_pin_handler_3(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if ( nrf_drv_gpiote_in_is_set(PIN_IN_3) ) nrf_drv_gpiote_out_set(PIN_OUT_3);
	else nrf_drv_gpiote_out_clear(PIN_OUT_3);
}

static void gpio_init(void)
{
	
	ret_code_t err_code;
	
	//configure and initialize output pins
	err_code = nrf_drv_gpiote_init();
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
	err_code = nrf_drv_gpiote_out_init(PIN_OUT_0, &out_config);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(PIN_OUT_1, &out_config);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(PIN_OUT_2, &out_config);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(PIN_OUT_3, &out_config);
	APP_ERROR_CHECK(err_code);
	
	//configure and initialize input pins
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  in_config.pull = NRF_GPIO_PIN_PULLUP;
	
	err_code = nrf_drv_gpiote_in_init(PIN_IN_0, &in_config, in_pin_handler_0);
  APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_in_init(PIN_IN_1, &in_config, in_pin_handler_1);
  APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_in_init(PIN_IN_2, &in_config, in_pin_handler_2);
  APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_in_init(PIN_IN_3, &in_config, in_pin_handler_3);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(PIN_IN_0, true);
	nrf_drv_gpiote_in_event_enable(PIN_IN_1, true);
	nrf_drv_gpiote_in_event_enable(PIN_IN_2, true);
	nrf_drv_gpiote_in_event_enable(PIN_IN_3, true);
	
}
/**
 * @brief Function for application main entry.
 */
int main(void)
{
	
		gpio_init();
		nrf_drv_gpiote_out_set(PIN_OUT_0);
		nrf_drv_gpiote_out_set(PIN_OUT_1);
		nrf_drv_gpiote_out_set(PIN_OUT_2);
		nrf_drv_gpiote_out_set(PIN_OUT_3);
	
    while (true)
    {
        // Do nothing.
    }
}
/** @} */
