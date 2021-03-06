/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
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
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define COMMAND_LEN 1 
#define ADDR_LEN 3
#define PAGE_LEN 256
#define FRAME_LEN COMMAND_LEN+ADDR_LEN+PAGE_LEN


#define STATUS_REG 0x05
#define RD_FLAG_STATUS_REG 0x70
#define ENH_VOL_CONFIG_REG 0x65
#define WRITE_ENABLE 0x06
#define MEM_ADDR 0x000000
#define RESET_ENABLE 0x66
#define RESET_MEMORY 0x99
#define RD_NONVOL_CONFIG 0xB5

static uint8_t	*m_tx_buf;									      /**< TX buffer. */
static uint8_t 	*m_rx_buf;						    				/**< RX buffer. */
static uint8_t m_length = sizeof(m_tx_buf);       /**< Transfer length. */

void byte_to_3led(uint8_t data);

/**
 * @brief SPI user event handler.
 * @param event
 */
void wait_for_transfer() {
		while (!spi_xfer_done)
		{
				__WFE();
		}
}
 
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Master Transfer completed.\r\n");

}

void spi_write(uint8_t command) {
		memset(m_tx_buf,0,FRAME_LEN); 	
		m_tx_buf[0] = command;
		m_length = sizeof(m_tx_buf);       /**< Transfer length. */
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
		wait_for_transfer();
}

void subsector_erase() {
		memset(m_tx_buf,0,FRAME_LEN); 	
		m_tx_buf[0] = 0x20;
		m_tx_buf[1] = 0;
		m_tx_buf[2] = 0;
		m_tx_buf[3] = 0;
		m_length = 4;       /**< Transfer length. */
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
		wait_for_transfer();
}



int main(void)
{
		m_tx_buf = malloc(sizeof(uint8_t)*FRAME_LEN);
		m_rx_buf = malloc(sizeof(uint8_t)*(FRAME_LEN+1));
	
		
	
    bsp_board_leds_init();

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

    NRF_LOG_INFO("SPI example\r\n");

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = 29;
    spi_config.miso_pin = 30;
    spi_config.mosi_pin = 30;
    spi_config.sck_pin  = 31;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    while (1)
    {
        // Reset rx buffer and transfer done flag
        memset(m_rx_buf, 0, m_length);
        spi_xfer_done = false;

				memset(m_rx_buf, 0, PAGE_LEN);
				spi_write(STATUS_REG);

        NRF_LOG_FLUSH();
								
				if (m_rx_buf[0] != 0)
				{
						NRF_LOG_INFO("Master Received from Slave: \r\n");
						NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
						while(1) {
							byte_to_3led(m_rx_buf[0]);
						}
						
				}
				else {
						while(1) {
							bsp_board_led_invert(BSP_BOARD_LED_2);
							nrf_delay_ms(50);

						}
				}
				
    }
}

void byte_to_3led(uint8_t data) {
	for (int i = 0; i < 6; i++) {
		bsp_board_led_invert(BSP_BOARD_LED_0);
		bsp_board_led_invert(BSP_BOARD_LED_1);
		bsp_board_led_invert(BSP_BOARD_LED_2);
		nrf_delay_ms(200);
	}
	nrf_delay_ms(500);
	bsp_board_led_invert(BSP_BOARD_LED_0);
	nrf_delay_ms(1500);
	
	for (int i = 7; i >= 0; i--) {
		nrf_delay_ms(500);
		bsp_board_led_invert(BSP_BOARD_LED_1);
		if ((0x01<<i) & data) {
			bsp_board_led_invert(BSP_BOARD_LED_2);
			nrf_delay_ms(1000);
			bsp_board_led_invert(BSP_BOARD_LED_2);
			
		}
		else nrf_delay_ms(1000);
		
		bsp_board_led_invert(BSP_BOARD_LED_1);
		nrf_delay_ms(750);
	}
	bsp_board_led_invert(BSP_BOARD_LED_0);
	nrf_delay_ms(500);
}

