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
#include "nrf_drv_gpiote.h"

#include <string.h>
#include "N25Q.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/*
static uint8_t       m_tx_buf[] = {0x9E};           // TX buffer.
static uint8_t       m_rx_buf[30];    // RX buffer
static const uint8_t tx_length = sizeof(m_tx_buf);        // Transfer length
static const uint8_t rx_length = 30;        // Transfer length
*/
#define PIN NRF_GPIO_PIN_MAP(1,3)
#define TRIGGER nrf_gpio_pin_set(PIN)


void gpio_init(void) {
	
	nrf_gpio_cfg_output(PIN);
	nrf_gpio_pin_clear(PIN);
}
int main(void)
{
	//leds init
    //bsp_board_leds_init();
	gpio_init();

	
	
	//log init
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    
	//flash init
    FLASH_DEVICE_OBJECT fdo; /* flash device object */
    ParameterType para; /* parameters used for all operation */
    ReturnType ret; /* return variable */
	NMX_uint16 reg16;
	NMX_uint8 reg8;
    NMX_uint8 rbuffer[20];
    NMX_uint8 wbuffer[32] = /* write buffer */
    {
		0xBE, 0xEF, 0xFE, 0xED, 0xBE, 0xEF, 0xFE, 0xED,
		0xBE, 0xEF, 0xFE, 0xED, 0xBE, 0xEF, 0xFE, 0xED
    };
	
	NRF_LOG_RAW_INFO("SPI Example Running.\n\n");
	
	//spi init
	spi_init();

	//driver init
	ret = Driver_Init(&fdo); /* initialize the flash driver */
    if (Flash_WrongType == ret)
    {
        NRF_LOG_RAW_INFO("Sorry, no device detected.\n"); 
    }
	else {
		NRF_LOG_RAW_INFO("Device detected.\n");		
	}	
		
	
	NRF_LOG_RAW_INFO("\nSECTOR ERASE\n");
	ret = fdo.GenOp.SectorErase(0); /* erase first subsector */
	NRF_LOG_RAW_INFO("TrRet: %d\n", ret);
	
	NRF_LOG_RAW_INFO("\nDATA PROGRAM\n");	
    para.PageProgram.udAddr = 0x0; /* program 16 byte at address 0 */
    para.PageProgram.pArray = wbuffer;
    para.PageProgram.udNrOfElementsInArray = 16;
	ret = fdo.GenOp.DataProgram(PageProgram, &para);
    NRF_LOG_RAW_INFO("Ret: %d\n", ret);
	
	NRF_LOG_RAW_INFO("\nDATA READ\n");
    para.Read.udAddr = 0x0; /* read 16 byte at address 0 */
    para.Read.pArray = rbuffer;
    para.Read.udNrOfElementsToRead = 20;
    ret = fdo.GenOp.DataRead(Read, &para);
    NRF_LOG_RAW_INFO("Ret: %d\n", ret);


    while (1)
    {
		
        NRF_LOG_FLUSH();
        bsp_board_led_invert(BSP_BOARD_LED_0);
        nrf_delay_ms(1000);
    }
	
}
