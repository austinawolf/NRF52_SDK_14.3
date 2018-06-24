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


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "queue.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"
 
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "app_util.h"
#include "nrf_drv_saadc.h"
#include "adc_helper.h"
#include "log_helper.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SAMPLE_PERIOD 200

#define QUEUE_LENGTH 10
#define ITEM_SIZE 4

QueueHandle_t adc_queue;
BaseType_t xStatus;
static uint32_t queue_out;

static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x04, 0x00, 0x00, 0x00, 0x00);
static nrf_esb_payload_t        rx_payload;

TaskHandle_t packet_write_task_handle;
TaskHandle_t log_flush_task_handle;
TimerHandle_t adc_sample_timer_handle;

static void packet_write_task_function( void * pvParameter);
static void log_flush_task_function (void * pvParameter);
static void adc_sample_timer_callback(void * pvParameter);

void uint32_to_array(uint32_t data, uint8_t* array);
uint32_t array_to_uint32(uint8_t* array);

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
		
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;

				int32_t adc = 0;
        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
						adc += ((p_event->data.done.p_buffer[i]) / (SAMPLES_IN_BUFFER));
        }

				adc_time_av = (adc_evt_counter % LEDS_NUMBER);//adc;
				
				NRF_LOG_INFO("ADC event number: %d", (int) adc_evt_counter);
				xQueueSendToBack(adc_queue,&adc_time_av,0);
				
        adc_evt_counter++;
    }
}


static void packet_write_task_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
		const TickType_t xTicksToWait = pdMS_TO_TICKS(1000);
		
	
    while (true)
    {
				xStatus = xQueueReceive( adc_queue, &queue_out,xTicksToWait);
			
				if (xStatus == pdPASS) {
						NRF_LOG_DEBUG("Queue out: %u",queue_out);
						uint32_to_array(queue_out,tx_payload.data);
						NRF_LOG_DEBUG("Transmitting packet: %u,%u,%u,%u", tx_payload.data[0], tx_payload.data[1], tx_payload.data[2], tx_payload.data[3]);
						
						tx_payload.noack = false;
						if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
						{
								bsp_board_led_invert((adc_evt_counter-1) % LEDS_NUMBER);
						}
						else
						{
								NRF_LOG_WARNING("Sending packet failed");
						}

				}
				
    }
		
}

static void log_flush_task_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    while (true)
    {
        NRF_LOG_FLUSH();
    }
		
}
static void adc_sample_timer_callback (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
		nrf_drv_saadc_sample();
		
}

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            NRF_LOG_DEBUG("TX SUCCESS EVENT");
						
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            NRF_LOG_DEBUG("TX FAILED EVENT");
            (void) nrf_esb_flush_tx();
            (void) nrf_esb_start_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            NRF_LOG_DEBUG("RX RECEIVED EVENT");
            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                if (rx_payload.length > 0)
                {
                    NRF_LOG_DEBUG("RX RECEIVED PAYLOAD");
                }
            }
            break;
    }
}


void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


void gpio_init( void )
{
    //nrf_gpio_range_cfg_output(8, 15);
    bsp_board_leds_init();
}


uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay         = 600;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = false;

    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    return err_code;
}


int main(void)
{
		//GPIO/LEDS INIT
    ret_code_t err_code;
    gpio_init();

		//LOG INIT
    logger_init();

		//CLOCKS INIT
    clocks_start();
    err_code = esb_init();
    APP_ERROR_CHECK(err_code);
	
		//SDAAC INIT
		saadc_init();
	
		//QUEUE INIT
		adc_queue = xQueueCreate(QUEUE_LENGTH,sizeof(int32_t));
		if (adc_queue == NULL) NRF_LOG_DEBUG("QUEUE CREATE FAILED");
	
		//TASKS INIT
    /* Create task for packet write with priority set to 2 */
    UNUSED_VARIABLE(xTaskCreate(packet_write_task_function, "PACKET_WRITE", configMINIMAL_STACK_SIZE + 200, NULL, 2, &packet_write_task_handle));		
   
		/* Create task for log flush with priority set to 1 */
		UNUSED_VARIABLE(xTaskCreate(log_flush_task_function, "LOG_FLUSH", configMINIMAL_STACK_SIZE + 200, NULL, 1, &log_flush_task_handle));

    /* Start timer for adc sampling */
    adc_sample_timer_handle = xTimerCreate( "ADC_SAMPLE", SAMPLE_PERIOD, pdTRUE, NULL, adc_sample_timer_callback);
    UNUSED_VARIABLE(xTimerStart(adc_sample_timer_handle, 0));


    NRF_LOG_DEBUG("Enhanced ShockBurst Transmitter Example running.");
		

		
		vTaskStartScheduler();
		
		while (true) 
		{
			
		}

}

void uint32_to_array(uint32_t data, uint8_t* array) {
	array[0] = (uint8_t) (data>>24);
	array[1] = (uint8_t) (data>>16);
	array[2] = (uint8_t) (data>>8);
	array[3] = (uint8_t) (data);
}

uint32_t array_to_uint32(uint8_t* array) {
	uint32_t data;
	data = (array[0]<<24) + (array[1]<<16) + (array[2]<<8) + (array[3]);
	return data;
}
