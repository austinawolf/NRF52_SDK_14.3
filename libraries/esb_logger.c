#include "esb_logger.h"

#ifdef ESB_LOGGER

#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "app_error.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"
#include "nrf_esb.h"
#include "led_error.h"
#include "app_fifo.h"
#include "nrf_delay.h"
#include "nrf_esb.h"


#define NRF_LOG_MODULE_NAME esb_log
#if ESB_LOG_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL ESB_LOG_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR ESB_LOG_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR ESB_LOG_CONFIG_DEBUG_COLOR
#else
#define NRF_LOG_LEVEL 0
#endif
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


#define TX_BUFFER_SIZE 256

static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x00);
static nrf_esb_payload_t        rx_payload;

uint32_t tx_count = 0;

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event);
uint32_t esb_log(uint8_t data);
void fifo_init(void);

app_fifo_t my_fifo;

void esb_logger_init(void) {
	esb_init();
	fifo_init();
}	
	
uint32_t esb_init(void) {
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

void fifo_init(void) {

	// Create a buffer for the FIFO
	uint16_t buffer_size = TX_BUFFER_SIZE;
	uint8_t buffer[buffer_size];

	// Initialize FIFO structure
	uint32_t err_code = app_fifo_init(&my_fifo, buffer, (uint16_t)sizeof(buffer));	
	
}

uint32_t esb_log(uint8_t data) {	
	uint32_t err_code;
	err_code = app_fifo_put(&my_fifo, data);
	return err_code;
}

void esb_log_print(const char* format, ...) {
	char string[32];
	
	va_list arglist;
	va_start(arglist,format);
	vsprintf(string,format,arglist);
	va_end(arglist);

	uint8_t i = 0;
	while(string[i] != 0) {
		esb_log( (uint8_t) string[i++]);
	}

	return;
}

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            //NRF_LOG_DEBUG("TX SUCCESS EVENT");
						
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            NRF_LOG_DEBUG("TX FAILED EVENT");
            (void) nrf_esb_flush_tx();
            (void) nrf_esb_start_tx();
            break;
		
        case NRF_ESB_EVENT_RX_RECEIVED:
            //NRF_LOG_DEBUG("RX RECEIVED EVENT");
            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                if (rx_payload.length > 0)
                {
                    NRF_LOG_DEBUG("RX RECEIVED PAYLOAD");
                    NRF_LOG_HEXDUMP_DEBUG(rx_payload.data, rx_payload.length);
					memset(rx_payload.data,0,32);
					
                }
            }
            break;
    }
}


void esb_log_flush(void)
{

	uint8_t  return_val;
	uint8_t i;
	uint32_t err_code;
	
    while (my_fifo.write_pos > my_fifo.read_pos)
    {

		//Load payload
		i = 0;
		while(my_fifo.write_pos > my_fifo.read_pos) {
			err_code = app_fifo_get(&my_fifo, &return_val);
			tx_payload.data[i++] = return_val;
			
			if (err_code) {
				alert(BSP_BOARD_LED_0, ESB_TRANSMISSION_ERR);
				break;
			}
			else if (i > 30) {
				break;
			}
			else if (return_val == 10) {
				break;
			}
				
		}
		
		tx_payload.data[i++] = 0;
		tx_payload.length = i;
		
		//Send payload
		//NRF_LOG_DEBUG("pl:%s, len: %d",tx_payload.data, tx_payload.length);

		if (nrf_esb_write_payload(&tx_payload) != NRF_SUCCESS) {
			NRF_LOG_WARNING("Sending packet failed");
			alert(BSP_BOARD_LED_0, ESB_TRANSMISSION_ERR);		
		}
    }		
}

#endif

