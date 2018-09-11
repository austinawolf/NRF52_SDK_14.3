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
 //std libraries
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"

//nrf libraries
#include "nrf.h"
#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "bsp.h"
#include "nordic_common.h"
#include "sdk_errors.h"
#include "app_error.h" 
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_util.h"

//nrf drivers
#include "nrf_drv_clock.h"

//local libraries
#include "log_helper.h"
#include "logger.h"
#include "twi_interface.h"
#include "led_error.h"
#include "imu.h"
#include "config.h"

uint32_t event_number = 0;
uint32_t packet_number = 0;


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



int main(void)
{
	uint32_t err_code;


	int imu_status;

	//GPIO/LEDS INIT
	gpio_init();
	
	//ESB LOGGER INIT
	LOG_INIT();

	//TWI INIT
	twi_interface_init();

	//MPU INIT
	imu_status = imu_init();	

	//CLOCKS INIT
	clocks_start();

	//log init
	err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_DEFAULT_BACKENDS_INIT();
		
	//running	
	NRF_LOG_DEBUG("Enhanced ShockBurst Transmitter Example running.");
	LOG_PRINT("Esb Logger Running\r\n");
	LOG_PRINT("IMU init: %d\r\n", imu_status);
	
	while (true) 
	{
		Motion motion;
		motion = get_motion_data();

		
		LOG_PRINT("Packet:%u,",motion.event);
		LOG_PRINT("%u,",motion.sensor_timestamp);
		LOG_PRINT("%i,", motion.quat[0]);
		LOG_PRINT("%i,", motion.quat[1]);
		LOG_PRINT("%i,", motion.quat[2]);
		LOG_PRINT("%i,", motion.quat[3]);
		LOG_PRINT("%i,", motion.accel[0]);
		LOG_PRINT("%i,", motion.accel[1]);
		LOG_PRINT("%i,", motion.accel[2]);
		LOG_PRINT("%i,", motion.gyro[0]);
		LOG_PRINT("%i,", motion.gyro[1]);
		LOG_PRINT("%i,", motion.gyro[2]);
		LOG_PRINT("%u,",motion.sensor_num);
		LOG_PRINT("%i\r\n", motion.status);
		LOG_FLUSH();
		nrf_delay_ms(50);


	}

}
