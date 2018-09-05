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
#include "spi_interface.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "N25Q.h"

/*
static uint8_t       m_tx_buf[] = {0x9E};           // TX buffer.
static uint8_t       m_rx_buf[30];    // RX buffer
static const uint8_t tx_length = sizeof(m_tx_buf);        // Transfer length
static const uint8_t rx_length = 30;        // Transfer length
*/





/**
 * @brief SPI user event handler.
 * @param event
 
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
		for (uint8_t i = 0; i < 30; i++) {
				NRF_LOG_INFO("Index: %u, %p", i, m_rx_buf[i]); 
		}
}
*/

int main(void)
{
	//leds init
    bsp_board_leds_init();

	//log init
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    
	//flash init
    FLASH_DEVICE_OBJECT fdo; /* flash device object */
    ParameterType para; /* parameters used for all operation */
    ReturnType ret; /* return variable */
    NMX_uint8 rbuffer[16];
    NMX_uint8 wbuffer[16] = /* write buffer */
    {
		0xBE, 0xEF, 0xFE, 0xED, 0xBE, 0xEF, 0xFE, 0xED,
		0xBE, 0xEF, 0xFE, 0xED, 0xBE, 0xEF, 0xFE, 0xED
    };
	
	NRF_LOG_RAW_INFO("\n\nSPI Example Running.\n\n");
	NRF_LOG_FLUSH();
	
	//spi init
	spi_init();
		
	//driver init
    ret = Driver_Init(&fdo); /* initialize the flash driver */
    if (Flash_WrongType == ret)
    {
        NRF_LOG_RAW_INFO("Sorry, no device detected."); 
    }
	else {
		NRF_LOG_RAW_INFO("Device detected.");	
	}
	NRF_LOG_FLUSH();	
	
	NRF_LOG_RAW_INFO("\n\nSECTOR ERASE\n");
	fdo.GenOp.SectorErase(0); /* erase first sector */
    para.PageProgram.udAddr = 0; /* program 16 byte at address 0 */
    para.PageProgram.pArray = wbuffer;
    para.PageProgram.udNrOfElementsInArray = 16;
    
	NRF_LOG_RAW_INFO("\n\nDATA PROGRAM\n");
	fdo.GenOp.DataProgram(PageProgram, &para);
    para.Read.udAddr = 0; /* read 16 byte at address 0 */
    para.Read.pArray = rbuffer;
    para.Read.udNrOfElementsToRead = 16;

	NRF_LOG_RAW_INFO("\n\nDATA READ\n");
    fdo.GenOp.DataRead(Read, &para);
	
	NRF_LOG_RAW_INFO("\n\nDATA\n");
	for (int i = 0; i < 16; i++) {
		NRF_LOG_RAW_INFO("%d,0x%x\n", i, rbuffer[i]); /* now rbuffer contains written elements */	
        NRF_LOG_FLUSH();
		nrf_delay_ms(1);
	}
    while (1)
    {

        //spi_transfer(m_tx_buf, tx_length, m_rx_buf, rx_length);

        NRF_LOG_FLUSH();
        bsp_board_led_invert(BSP_BOARD_LED_0);
        nrf_delay_ms(1000);
    }
	
}
