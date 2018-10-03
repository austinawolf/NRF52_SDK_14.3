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
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "boards.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_esb.h"
#include "app_util_platform.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "led_error.h"
#include "uart_helper.h"
#include "app_fifo.h"
#include "SEGGER_RTT.h"

#define TX_BUFFER_SIZE 256


uint8_t led_nr;

static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x00);
static nrf_esb_payload_t        rx_payload;

uint32_t success_event_total, fail_event_total = 0;

/*lint -save -esym(40, BUTTON_1) -esym(40, BUTTON_2) -esym(40, BUTTON_3) -esym(40, BUTTON_4) -esym(40, LED_1) -esym(40, LED_2) -esym(40, LED_3) -esym(40, LED_4) */
#define PREAMBLE 0xaa
#define POSTAMBLE 0xbb

app_fifo_t my_fifo;

static void fifo_init(void);

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            NRF_LOG_DEBUG("TX SUCCESS EVENT");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            NRF_LOG_DEBUG("TX FAILED EVENT");
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            if (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
											
                // Set LEDs identical to the ones on the PTX.
                nrf_gpio_pin_write(LED_1, !(success_event_total%8>0 && success_event_total%8<=4));
                nrf_gpio_pin_write(LED_2, !(success_event_total%8>1 && success_event_total%8<=5));
                nrf_gpio_pin_write(LED_3, !(success_event_total%8>2 && success_event_total%8<=6));
                nrf_gpio_pin_write(LED_4, !(success_event_total%8>3));

					
				char rx_data[32];
				memcpy(rx_data,rx_payload.data,32);				
				printf("%s",rx_data);


				success_event_total++;
								
							
            }
			else {
				NRF_LOG_DEBUG("FAILED EVENT: %u",fail_event_total++);
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
    bsp_board_leds_init();
}
		
uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.payload_length           = 32;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.selective_auto_ack       = true;

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



static void bsp_evt_handler(bsp_event_t evt)
{
    switch (evt)
    {
        // Button 1 - switch to the previous demo.
        case BSP_EVENT_KEY_0:
			
			printf("Setting Baud to 1M...\n\r");
			set_baud(UART_BAUDRATE_BAUDRATE_Baud1M);
		
			return;
				
        // Button 2 - switch to the next demo.
        case BSP_EVENT_KEY_1:
  
		
			printf("Setting Baud to 115200...\n\r");
			set_baud(UART_BAUDRATE_BAUDRATE_Baud115200);
		
			return;
				
        default:
            return;
    }

}

static void init_bsp()
{
    APP_ERROR_CHECK(nrf_drv_clock_init());
    nrf_drv_clock_lfclk_request(NULL);

    APP_ERROR_CHECK(app_timer_init());
    APP_ERROR_CHECK(bsp_init(BSP_INIT_BUTTONS, bsp_evt_handler));
    APP_ERROR_CHECK(bsp_buttons_enable());
}

static void fifo_init(void) {

	// Create a buffer for the FIFO
	uint16_t buffer_size = TX_BUFFER_SIZE;
	uint8_t buffer[buffer_size];

	// Initialize FIFO structure
	uint32_t err_code = app_fifo_init(&my_fifo, buffer, (uint16_t)sizeof(buffer));	
	
}


static int load_packet(void) {

	nrf_esb_flush_tx();	
	
	uint8_t byte; //takes bytes from fifo
	uint8_t i = 0; //packet index for loading
	uint8_t integer; //integer
	char str[12]; //temp string to be converted to integer
	uint8_t k; //temp string index
	
	//clear payload
	memset(tx_payload.data, 0, 32);
				
	//load payload
	k = 0;

	tx_payload.data[i++] = PREAMBLE;
	while(!app_fifo_get(&my_fifo, &byte)) {

		
		if (byte == ' ') {
			NRF_LOG_HEXDUMP_DEBUG(str, k);
			integer = (uint8_t) strtol(str, NULL, 0);
			if (integer) {
				tx_payload.data[i++] = integer;
			}
			k = 0;
			continue;
		}

		if (i > 30) {
			return -1;
		}
		NRF_LOG_DEBUG("load: %d", byte);

		str[k++] = byte;

	}
	tx_payload.data[i++] = POSTAMBLE;	
	tx_payload.length = i;
	
	//terminate payload
	
	return 0;
}

int main(void)
{
    uint32_t err_code;

	//gpio init
    gpio_init();
	
	//bsp init
	init_bsp();
	
	//clocks start
    clocks_start();

	//shockburst init
    err_code = esb_init();
    APP_ERROR_CHECK(err_code);

	//shockburst rx start
    err_code = nrf_esb_start_rx();
    APP_ERROR_CHECK(err_code);
		
	//uart helper init
	uart_helper_init();   
	
	//fifo init
	fifo_init();
	
	//nrf log init
	err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	NRF_LOG_INFO("esb logger rx running.");	
	
	
	//running		
    printf("Enhanced ShockBurst Receiver Example running.\n\r");

	#define LENGTH 32
	char segger_rx_buffer[LENGTH];
	uint8_t len;
	
    while (true)
    {	
		uint32_t err_code;
        uint8_t cr;
		
		
		//CHECK UART BUFFER
        if(app_uart_get(&cr) == NRF_SUCCESS) {
			
			//if endline send packet
			if (cr == '\r') {
				
				if (my_fifo.write_pos == my_fifo.read_pos) {
					continue;
				}
				
				err_code = app_fifo_put(&my_fifo, ' ');
				if (err_code) {
					printf("app_fifo_put ret: %d", err_code);
				}
				
				load_packet();
				
				//log payload
				NRF_LOG_DEBUG("SENDING FROM UART");
				NRF_LOG_HEXDUMP_DEBUG(tx_payload.data, tx_payload.length);
				
				//send payload
				if (nrf_esb_write_payload(&tx_payload) != NRF_SUCCESS) {
					printf("Sending packet failed\n\r");
				}
				memset(tx_payload.data,0,32);
				nrf_esb_flush_tx();
				nrf_esb_flush_rx();
				
				
			}
			
			//if char load fifo
			else {
				
				//load fifo on uart rx
				err_code = app_fifo_put(&my_fifo, cr);
				NRF_LOG_DEBUG("char: %d", cr);
				if (err_code) {
					printf("app_fifo_put ret: %d", err_code);
				}

			}			
		}
		
		//CHECK RTT BUGGER
		len = SEGGER_RTT_Read(0, segger_rx_buffer, LENGTH);
		if (len > 0) {
			
			for (int i = 0; i < len && segger_rx_buffer[i] != '\r'; i++) {
				app_fifo_put(&my_fifo, segger_rx_buffer[i]);
				
			}		
			app_fifo_put(&my_fifo, ' ');

		
			load_packet();
			
			//log payload
			NRF_LOG_DEBUG("SENDING FROM RTT");
			NRF_LOG_HEXDUMP_DEBUG(tx_payload.data, tx_payload.length);
			//send payload
			if (nrf_esb_write_payload(&tx_payload) != NRF_SUCCESS) {
				printf("Sending packet failed\n\r");
			}
			memset(tx_payload.data,0,32);
		
		}	
	}
}



