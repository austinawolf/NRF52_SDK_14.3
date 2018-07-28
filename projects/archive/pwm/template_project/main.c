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
#include <stdio.h>
#include <string.h>

#include "app_util_platform.h"
#include "nrf_drv_pwm.h"
#include "app_error.h"
#include "boards.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define PIN_IN_0 BSP_BUTTON_0

#define PIN_OUT_0 BSP_LED_0

static void gpio_init(void)
{
	

	
}
/**
 * @brief Function for application main entry.
 */
int main(void)
{
	
		uint32_t err_code;
		nrf_drv_pwm_config_t const config0 =
		{
				.output_pins =
				{
						BSP_LED_0 | NRF_DRV_PWM_PIN_INVERTED, // channel 0
						NRF_DRV_PWM_PIN_NOT_USED,             // channel 1
						NRF_DRV_PWM_PIN_NOT_USED,             // channel 2
						NRF_DRV_PWM_PIN_NOT_USED,             // channel 3
				},
				.irq_priority = APP_IRQ_PRIORITY_LOW,
				.base_clock   = NRF_PWM_CLK_1MHz,
				.count_mode   = NRF_PWM_MODE_UP,
				.top_value    = 1000,
				.load_mode    = NRF_PWM_LOAD_COMMON,
				.step_mode    = NRF_PWM_STEP_AUTO
		};
		err_code = nrf_drv_pwm_init(&m_pwm0, &config0, NULL);
		if (err_code != NRF_SUCCESS)
		{
				// Initialization failed. Take recovery action.
		}
				

	
    while (true)
    {
        // Do nothing.
    }
}
/** @} */
