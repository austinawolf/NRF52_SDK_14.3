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
#include "MadgwickAHRS.h"

app_timer_id_t my_timer_id;


uint32_t event_number = 0;
uint32_t packet_number = 0;


//#define LOG_OUTPUT
#define MAG_CAL
#define CAL_OUTPUT


void timeout_handler(void * p_context);
									
void clocks_start()
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
	uint32_t ticks = APP_TIMER_TICKS(IMU_SAMPLE_PERIOD_MS+10);
	
	err_code = app_timer_start(my_timer_id,ticks,p_context);
	if (err_code) {
		NRF_LOG_ERROR("app_timer_create ret:%d",err_code);
	}	
}

void timeout_handler(void * p_context) {
	
	Motion motion;
	
	imu_get_data(&motion);
	imu_get_compass(&motion);

	float x, y, z, mx, my, mz, gx, gy, gz, ax, ay, az;
	
	
	/* PROCESS COMPASS DATA */
	#ifndef MAG_CAL
	// Apply mag offset compensation (base values in uTesla)
	x = motion.compass[0] - mag_offsets[0];
	y = motion.compass[1] - mag_offsets[1];
	z = motion.compass[2] - mag_offsets[2];
	// Apply mag soft iron error compensation
	mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
	my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
	mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
	#else
	(void) x;
	(void) y;
	(void) z;
	mx = (float) motion.compass[0];
	my = (float) motion.compass[1];
	mz = (float) motion.compass[2];
	#endif
	
	/* PROCESS GYRO DATA */
	gx = motion.gyro[0] * RAW_GYRO_TO_RADS;
	gy = motion.gyro[1] * RAW_GYRO_TO_RADS;
	gz = motion.gyro[2] * RAW_GYRO_TO_RADS;			
	
	/* PROCESS ACCEL DATA */
	ax = motion.accel[0] * RAW_ACCEL_TO_GS;
	ay = motion.accel[1] * RAW_ACCEL_TO_GS;
	az = motion.accel[2] * RAW_ACCEL_TO_GS;		
	
	/* RUN SENSOR FUSION */	
	if (motion.status == 0) {	
		if (false) {
			MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
		}
		else {
			MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		}

	}
	else {
		NRF_LOG_DEBUG("Motion.status: %d", motion.status);
	}
	
	#ifdef LOG_OUTPUT
	LOG_PRINT("\r\nPacket:%u,",motion.event);
	LOG_PRINT("%u,",motion.sensor_timestamp);
	LOG_PRINT("%.3f,", q0);
	LOG_PRINT("%.3f,", q1);
	LOG_PRINT("%.3f,", q2);
	LOG_PRINT("%.3f,", q3);
	LOG_PRINT("%d,", motion.accel[0]);
	LOG_PRINT("%d,", motion.accel[1]);
	LOG_PRINT("%d,", motion.accel[2]);
	LOG_PRINT("%d,", motion.gyro[0]);
	LOG_PRINT("%d,", motion.gyro[1]);
	LOG_PRINT("%d,", motion.gyro[2]);
	LOG_PRINT("%i,",motion.sensor_num);
	LOG_PRINT("%i\r\n", motion.status);				
	#endif
	
	#ifdef CAL_OUTPUT
	LOG_PRINT("Raw:");
	LOG_PRINT("%d,%d,%d,",motion.accel[0],motion.accel[1],motion.accel[2]);		
	LOG_PRINT("%d,%d,%d,",motion.gyro[0],motion.gyro[1],motion.gyro[2]);
	LOG_PRINT("%d,%d,%d", (int) mx,(int) my, (int) mz);
	LOG_PRINT("\n\r");
	#endif
	
	LOG_FLUSH();
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
	
	//MPU INIT
	imu_status = imu_init_madgwick();		
		
	NRF_LOG_INFO("Enhanced ShockBurst Transmitter Example running.");		
	
	//running	
	LOG_PRINT("Esb Logger Running\r\n");
	if (imu_status) {
		//alert(BSP_BOARD_LED_2, RESET);	
		NVIC_SystemReset();
	}
	
	//TIMER INIT
	timer_init();
	
	while(true) {
		
	}
	

}


