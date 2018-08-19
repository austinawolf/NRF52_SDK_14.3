#include "esb_logger.h"

#ifdef ESB_LOGGER

#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "queue.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"
#include "nrf_esb.h"
#include "led_error.h"
#include "app_fifo.h"

static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x00);
static nrf_esb_payload_t        rx_payload;
BaseType_t xStatus;
uint8_t *temp;

uint32_t tx_count = 0;
QueueHandle_t payload_queue;
TaskHandle_t packet_write_task_handle;

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event);
static void packet_write_task_function( void * pvParameter);
uint8_t string_to_array(char *string, uint8_t* array);
void esb_tx_task_init(void);
void fifo_init(void);

app_fifo_t my_fifo;

void esb_logger_init(void) {
	
	esb_init();
	esb_tx_task_init();
	fifo_init();
}	

void set_payload_queue(QueueHandle_t queue) {
		payload_queue = queue;
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

void esb_tx_task_init(void) {
	
		//TASKS INIT
    /* Create task for packet write with priority set to 2 */
    UNUSED_VARIABLE(xTaskCreate(packet_write_task_function, "PACKET_WRITE", configMINIMAL_STACK_SIZE + 200, NULL, 2, &packet_write_task_handle));	
	
	

}

void fifo_init(void) {

		// Create a buffer for the FIFO
		uint16_t buffer_size = 512;
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
	esb_log(0);
	return;
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
						//alert(BSP_BOARD_LED_0, ESB_TRANSMISSION_ERR);
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


static void packet_write_task_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
		const TickType_t wait_500ms = pdMS_TO_TICKS(50);
		const TickType_t wait_1ms = pdMS_TO_TICKS(1);

		uint8_t  return_val = 1;
		uint32_t err_code;
		uint8_t i;
    while (true)
    {

				
				taskENTER_CRITICAL();
				if (my_fifo.write_pos > my_fifo.read_pos) {
						
						i = 0;
						while(my_fifo.write_pos > my_fifo.read_pos) {
							err_code = app_fifo_get(&my_fifo, &return_val);
							tx_payload.data[i++] = return_val;
							if (i > 31) break;
							if (return_val == 0) break;
						}
						tx_payload.length = i;
						
						NRF_LOG_DEBUG("payload:%s",tx_payload.data);

						UNUSED_PARAMETER(err_code);
					
						tx_payload.noack = false;
						
						if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS); //bsp_board_led_invert((tx_count++) % LEDS_NUMBER);
						else {
							NRF_LOG_WARNING("Sending packet failed");
							alert(BSP_BOARD_LED_0, ESB_TRANSMISSION_ERR);
						}
						
						vTaskDelay(wait_1ms);
						
				}
				else {
					vTaskDelay(wait_500ms);
				}

				
				taskEXIT_CRITICAL();
    }
		
}


uint8_t string_to_array(char* string, uint8_t* array) {
	uint8_t i = 0;
	while(string[i] != 0) {
		array[i] = (uint8_t) string[i];
		i++;	
	}
	array[i++] = 0;
	return i;
}
	
	
#endif

