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
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */



#include <stdint.h>
#include <string.h>

//nordic
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"

//ble
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_ble_gatt.h"

//services
#include "ble_nus.h"

//softdevice
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"

//libraries
#include "logger.h"
#include "twi_interface.h"
#include "led_error.h"
#include "imu.h"


#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include <stdarg.h>
#undef NRF_LOG_INFO
#undef NRF_LOG_DEBUG
#undef NRF_LOG_ERROR
#define NRF_LOG_INFO printfl1
#define NRF_LOG_DEBUG printfl1
#define NRF_LOG_ERROR printfl1

//freertos
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define SAMPLE_PERIOD 1000
uint32_t event_num = 0;

//Timer
TimerHandle_t imu_read_timer_handle;
static void imu_read_timer_callback(void * pvParameter);

//Task function
TaskHandle_t imu_get_fifo_task_handle;
static void imu_get_fifo_task_function (void * pvParameter);

void printfl1(const char* format, ...) {
	va_list arglist;
	va_start(arglist,format);
	vprintf(format,arglist);
	printf("\n\r");
	va_end(arglist);
	return;
}

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                    /**< UART RX buffer size. */


static void imu_read_timer_callback (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	//LOG_PRINT("Timer Number: %d\n\r", event_num++);
	alert(BSP_BOARD_LED_1,1);
	vTaskResume( imu_get_fifo_task_handle );

}

static void imu_get_fifo_task_function (void * pvParameter)
{
		UNUSED_PARAMETER(pvParameter);
		while(true)	{
			LOG_PRINT("Task Number: %d\n\r", event_num++);
			//nrf_delay_ms(1000);
			//imu_get_fifo();
			vTaskSuspend( imu_get_fifo_task_handle );
		}
}

void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\r') || (index >= (BLE_LOGGER_MAX_DATA_SIZE)))
            {
                //NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                //NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                do
                {
                    uint16_t length = (uint16_t) index;
										data_array[length] = 0;
                    err_code = LOG_PRINT( (char*) data_array);
									
                    if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) )
                    {
                        APP_ERROR_CHECK(err_code);
                    }
										
                } while (err_code == NRF_ERROR_BUSY);

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t	err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = NRF_UART_BAUDRATE_115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}




/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;

    // Initialize.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

		//UART init
    uart_init();
		printf("\r\nUART Start!\r\n");
		
		//TWI INIT
		twi_interface_init();
	
		//MPU INIT
		int imu_status;
		imu_status = imu_init();	

		//BLE logger init
		LOG_INIT();
		LOG_PRINT("BLE Logger Running\r\n");
		LOG_PRINT("IMU init: %d\r\n", imu_status);	

		/* Start timer for packet generation */
    imu_read_timer_handle = xTimerCreate( "PAYLOAD_GEN", SAMPLE_PERIOD, pdTRUE, NULL, imu_read_timer_callback);
    UNUSED_VARIABLE(xTimerStart(imu_read_timer_handle, 0));	

    UNUSED_VARIABLE(xTaskCreate(imu_get_fifo_task_function, "IMU_FIFO", configMINIMAL_STACK_SIZE, NULL, 1, &imu_get_fifo_task_handle));	
		vTaskSuspend( imu_get_fifo_task_handle );
		
		alert(BSP_BOARD_LED_0,3);

		vTaskStartScheduler();

    for (;;)
    {
				alert(BSP_BOARD_LED_1,2);
		}
}
