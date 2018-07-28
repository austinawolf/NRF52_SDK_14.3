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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "bsp.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"

#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h" 
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "app_util.h"
#include "nrf_drv_saadc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "log_helper.h"
#include "esb_logger.h"
#include "led_error.h"
#include "flash_helper.h"

#define SAMPLE_PERIOD 2000

uint32_t event_number = 0;

TaskHandle_t log_flush_task_handle;
TimerHandle_t payload_create_timer_handle;

static void log_flush_task_function (void * pvParameter);
static void payload_create_timer_callback(void * pvParameter);
uint8_t value = 0;


static uint8_t	*m_tx_buf;									      /**< TX buffer. */
static uint8_t 	*m_rx_buf;						    				/**< RX buffer. */


static void log_flush_task_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    while (true)
    {
        NRF_LOG_FLUSH();
    }
		
}

static void payload_create_timer_callback (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
		esb_log_write_value("\tCount:%d",value++);
}

void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}





void gpio_init( void )
{
    bsp_board_leds_init();
}



int main(void)
{
		//GPIO/LEDS INIT
    gpio_init();

		//LOG INIT
    logger_init();
		
		//ESB LOGGER INIT
		esb_logger_init();

		//CLOCKS INIT
    clocks_start();
	
		//SPI SETUP
		spi_helper_init();

		/* Create task for log flush with priority set to 1 */
		UNUSED_VARIABLE(xTaskCreate(log_flush_task_function, "LOG_FLUSH", configMINIMAL_STACK_SIZE + 200, NULL, 1, &log_flush_task_handle));
    
		/* Start timer for packet generation */
    payload_create_timer_handle = xTimerCreate( "ADC_SAMPLE", SAMPLE_PERIOD, pdTRUE, NULL, payload_create_timer_callback);
    UNUSED_VARIABLE(xTimerStart(payload_create_timer_handle, 0));


    NRF_LOG_DEBUG("Enhanced ShockBurst Transmitter Example running.");
		
		m_tx_buf = malloc(sizeof(uint8_t)*FRAME_LEN);
		m_rx_buf = malloc(sizeof(uint8_t)*(FRAME_LEN+1));
	
		memset(m_tx_buf, 0, FRAME_LEN);
		memset(m_rx_buf, 0, FRAME_LEN+1);
		
		
		spi_write(0x9E, m_tx_buf, m_rx_buf);
		

		vTaskStartScheduler();
		
		while (true) 
		{
			
		}

}


