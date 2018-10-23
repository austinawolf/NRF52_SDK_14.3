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

//nrf log
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//local libraries
#include "logger.h"
#include "twi_interface.h"
#include "led_error.h"
#include "imu.h"
#include "config.h"
#include "app_timer.h"
#include "clock_interface.h"
#include "nrf_drv_timer.h"


uint32_t event_number = 0;
uint32_t packet_number = 0;


uint32_t sample_num = 0;
int accel_sum[XYZ] = {0, 0, 0};
int gyro_sum[XYZ] = {0, 0, 0};
int accel_av[XYZ] = {0, 0, 0};
int	gyro_av[XYZ] = {0, 0, 0};



const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(0);

void timeout_handler(void * p_context);

void clocks_start( void )
{
	
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);	

	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_LFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);

}





void gpio_init( void )
{
    //nrf_gpio_range_cfg_output(8, 15);
    bsp_board_leds_init();
}


void timer_init(void) {
	
	ret_code_t err_code;
	app_timer_init();

	APP_TIMER_DEF(my_timer_id);
	err_code = app_timer_create(&my_timer_id, APP_TIMER_MODE_REPEATED, timeout_handler);
	if (err_code) {
		NRF_LOG_ERROR("app_timer_create ret:%d",err_code);
	}

	void * p_context;
	uint32_t ticks = APP_TIMER_TICKS(IMU_SAMPLE_PERIOD_MS);
	
	err_code = app_timer_start(my_timer_id,ticks,p_context);
	if (err_code) {
		NRF_LOG_ERROR("app_timer_start ret:%d",err_code);
	}	
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
	
	//CLOCKS INIT
	clocks_start();
	
	//log init
	err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	NRF_LOG_INFO("Enhanced ShockBurst Transmitter Example running.");
	
	//MPU INIT
	int addr = twi_scan();
	NRF_LOG_DEBUG("addr: %d", addr);	
	
	imu_status = imu_init();			
	if (imu_status) {
		NRF_LOG_INFO("IMU init failed. Restarting.");
		NRF_LOG_FINAL_FLUSH();
		nrf_delay_ms(1000);
		NVIC_SystemReset();
	}
	imu_start();
	//imu_self_test();
	
	
	LOG_PRINT("Esb Logger Running\r\n");
	LOG_FLUSH();
	
	//run
	nrf_delay_ms(10);
	timer_init();
	
	while (true) 
	{
		//WFE
	}

}
void timeout_handler(void * p_context) {
	Motion motion;
	

	
	imu_get_data(&motion);
	if (motion.status) {
		return;	
	}
	
	imu_get_compass(&motion);
	imu_send_to_mpl(&motion);
	imu_log_motion_cal(&motion);
	
	if (!motion.status) {
		sample_num++;
		if (sample_num <= NUM_CAL_SAMPLES) {
			NRF_LOG_INFO("Event: %d / %d", sample_num, (int) NUM_CAL_SAMPLES);
			accel_sum[X] += motion.accel[X];
			accel_sum[Y] += motion.accel[Y];
			accel_sum[Z] += motion.accel[Z];
			gyro_sum[X] += motion.gyro[X];
			gyro_sum[Y] += motion.gyro[Y];
			gyro_sum[Z] += motion.gyro[Z];
		}			
	}
	if (sample_num == NUM_CAL_SAMPLES) {
		accel_av[X] = accel_sum[X] / NUM_CAL_SAMPLES;
		accel_av[Y] = accel_sum[Y] / NUM_CAL_SAMPLES;
		accel_av[Z] = accel_sum[Z] / NUM_CAL_SAMPLES;
		gyro_av[X] 	= gyro_sum[X] / NUM_CAL_SAMPLES;
		gyro_av[Y] 	= gyro_sum[Y] / NUM_CAL_SAMPLES;
		gyro_av[Z] 	= gyro_sum[Z] / NUM_CAL_SAMPLES;
		NRF_LOG_INFO("Accel Offsets: x=%d, y=%d, z=%d", accel_av[X], accel_av[Y], accel_av[Z]-RAW_1G_REFERENCE);
		NRF_LOG_INFO("Gyro Offsets: x=%d, y=%d, z=%d", gyro_av[X], gyro_av[Y], gyro_av[Z]);
		
	}
}
