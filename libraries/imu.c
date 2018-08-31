//std lib
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

//nordic
#include "bsp.h"

//local libraries
#include "twi_interface.h"
#include "led_error.h"
#include "imu.h"
#include "config.h"

//mpu9250 drivers
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "logger.h"

//mpl
#include "invensense.h"
#include "invensense_adv.h"

//freertos
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define MPU9150_ADDR 0x68
#define AK8963_ADDR 0x0C

#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B
#define WHO_AM_I 0x75


uint32_t fifo_num = 0;

int mpu_helper_init(void);
int mpu_helper_dmp_setup(void);
int	mpu_helper_inv_setup(void);

int imu_init(void) {
		int ret;
	
		ret = mpu_helper_init();
		if (ret != 0) return ret;

		ret = mpu_helper_dmp_setup();
		if (ret != 0) return ret;	

		mpu_compass_config();
	
		ret = mpu_helper_inv_setup();
		if (ret != 0) return ret;
		
		
	
		return 0;
}

int mpu_helper_init(void) {
		int ret;
		struct int_param_s int_param;
		
	  //int_param.cb = NULL;
		int_param.pin = 0;
		int_param.lp_exit = 0;
		int_param.active_low = 1;
	
		ret = mpu_init(&int_param);
		if (ret != 0) return -1;


	
		ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
		if (ret != 0) return -1;
		
		mpu_configure_fifo(INV_XYZ_ACCEL);
		if (ret != 0) return -1;
		
		mpu_set_gyro_fsr(2000);
		if (ret != 0) return -1;
		
		mpu_set_accel_fsr(16);
		if (ret != 0) return -1;
		
		//enable compass
		//mag_write_reg(0x0A, 0x16);
		
    //const long accel_bias[3] = {0, 0, 0};
		//const long accel_bias[3] = {33, 12, -162};
    const long accel_bias[3] = {5, 20, -200};
		mpu_set_accel_bias_6500_reg(accel_bias);

		//long gyro_bias[3] = {0, 0, 0};
		//long gyro_bias[3] = {-40, 1, 3};
		long gyro_bias[3] = {-45, 20, -58};
		mpu_set_gyro_bias_reg(gyro_bias);

		return 0;

}

int mpu_helper_dmp_setup(void) {
    int ret;

		ret = dmp_load_motion_driver_firmware();
		if (ret != 0) return -1;
	
		ret = mpu_set_dmp_state(1);
		if (ret != 0) return -2;

		ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | 
															DMP_FEATURE_SEND_RAW_ACCEL | 
															DMP_FEATURE_SEND_CAL_GYRO |
															DMP_FEATURE_GYRO_CAL);
		if (ret != 0) return -3;
		
		#define DEFAULT_MPU_HZ 21
	
		ret = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
		if (ret != 0) return -4;
		
		return ret;
}


int	mpu_helper_inv_setup(void) {
	inv_error_t result; 
	int ret;

	result = inv_init_mpl();
	if (result) return -1;

	
	
	static struct platform_data_s gyro_pdata = {
			.orientation = { 0, 0, 1,
											 0, 1, 0,
											 -1, 0, 0}
	};

  ret = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
	if (ret) return -2;
	
	return 0;
}

void mpu_compass_config(void) {
		mpu_set_bypass(0);
}

unsigned char imu_get_fifo(void) {
	int ret;
	
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4];
	unsigned long sensor_timestamp;

	//taskENTER_CRITICAL();
	ret = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
	//taskEXIT_CRITICAL();
	//if (ret < 0) critical_error(BSP_BOARD_LED_0,2);

	
	fifo_num++;
	
	
	#define QUAT
	#define ACC
	#define GYRO
	
	LOG_PRINT("\r\nPacket:%d,",fifo_num);
	#ifdef QUAT
		LOG_PRINT("%i,", quat[0]);
		LOG_PRINT("%i,", quat[1]);
		LOG_PRINT("%i,", quat[2]);
		LOG_PRINT("%i,", quat[3]);
	#endif
	
	#ifdef ACC
		LOG_PRINT("%i,", accel[0]);
		LOG_PRINT("%i,", accel[1]);
		LOG_PRINT("%i,", accel[2]);
	#endif
	
	#ifdef GYRO
		LOG_PRINT("%i,", gyro[0]);
		LOG_PRINT("%i,", gyro[1]);
		LOG_PRINT("%i,", gyro[2]);
	#endif
	LOG_PRINT("%u,",sensor_timestamp);
	LOG_PRINT("%i\r\n", ret);
	
	
	return ret;
}

Motion get_motion_data(void) {

	Motion motion;
	motion.sensor_num = SENSOR_NUM;
	motion.event = fifo_num++;
	motion.status = dmp_read_fifo(motion.gyro, motion.accel, motion.quat, &motion.sensor_timestamp, &motion.sensors, &motion.more);	
	return motion;
}



Acc mpu_get_acc(void) {
	
	int ret;
	Acc acc;
	
	unsigned char data[7];
	
	ret = nrf_twi_read(MPU9150_ADDR,ACCEL_XOUT_H, 6, data);
	if (ret != 0) {
		critical_error(BSP_BOARD_LED_0,3);
	}	

	acc.x = (data[0]<<8) + data[1];
	acc.y = (data[2]<<8) + data[3];
	acc.z = (data[4]<<8) + data[5];
	
	return acc;
		
}

Gyro mpu_get_gyro(void) {
	
	int ret;
	Gyro gyro;
	
	unsigned char data[7];
	
	ret = nrf_twi_read(MPU9150_ADDR,GYRO_XOUT_H, 6, data);
	if (ret != 0) {
		critical_error(BSP_BOARD_LED_0,3);
	}	

	gyro.x = (data[0]<<8) + data[1];
	gyro.y = (data[2]<<8) + data[3];
	gyro.z = (data[4]<<8) + data[5];
	
	return gyro;
		
}

void mpu_get_reg(uint8_t reg) {
	
	int ret;
	
	unsigned char data[7];
	
	ret = nrf_twi_read(MPU9150_ADDR, reg, 1, data);
	if (ret != 0) {
		critical_error(BSP_BOARD_LED_0,3);
	}	

	LOG_PRINT("Read Reg: %d, Data: %d\n\r",reg, data[0]);

	return;
		
}




void mpu_write_reg(uint8_t reg, unsigned char data) {
	
	int ret;
		
	ret = nrf_twi_write(MPU9150_ADDR, reg, 1, &data);
	
	if (ret != 0) {
		critical_error(BSP_BOARD_LED_0,3);
	}	

	LOG_PRINT("Write Reg: %d, Data: %d\n\r",reg, data);

	return;
		
}






Mag mpu_get_mag(void) {
	
	int ret;
	Mag mag;
	
	unsigned char data[7];
	
	ret = nrf_twi_read(AK8963_ADDR,0x03, 6, data);
	if (ret != 0) {
		critical_error(BSP_BOARD_LED_0,3);
	}	

	mag.x = (data[0]<<8) + data[1];
	mag.y = (data[2]<<8) + data[3];
	mag.z = (data[4]<<8) + data[5];
	
	return mag;
		
}

void mag_get_reg(uint8_t reg) {
	
	int ret;
	
	unsigned char data[7];
	
	ret = nrf_twi_read(AK8963_ADDR, reg, 1, data);
	if (ret != 0) {
		critical_error(BSP_BOARD_LED_0,3);
	}	

	LOG_PRINT("Read Reg: %d, Data: %d\n\r",reg, data[0]);

	return;
		
}

void mag_write_reg(uint8_t reg, unsigned char data) {
	
	int ret;
			
	ret = nrf_twi_write(AK8963_ADDR, reg, 1, &data);
	
	if (ret != 0) {
		critical_error(BSP_BOARD_LED_0,3);
	}	

	LOG_PRINT("Write Reg: %d, Data: %d\n\r",reg, data);

	return;
		
		
}



