#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "twi_interface.h"
#include "led_error.h"
#include "mpu9150_helper.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "esb_logger.h"

#define MPU9150_ADDR 0x68

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define PWR_MGMT_1 0x6B

#define WHO_AM_I 0x75

uint32_t event_num = 0;

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
		
		mpu_set_gyro_fsr(500);
		if (ret != 0) return -1;
		
		mpu_set_accel_fsr(16);
		if (ret != 0) return -1;
		

    const long accel_bias[3] = {88, 70, -188};
		mpu_set_accel_bias_6500_reg(accel_bias);


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
		
		#define DEFAULT_MPU_HZ 1
	
		ret = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
		if (ret != 0) return -4;
		
		return ret;
}

unsigned char mpu_test(void) {
	int ret;
	
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4];
	unsigned long sensor_timestamp;

	ret = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
	event_num++;
	
	
	#define QUAT
	#define ACC
	#define GYRO
	
	esb_log_print("\r\nPacket:%d,",event_num);
	#ifdef QUAT
		esb_log_print("%i,", quat[0]);
		esb_log_print("%i,", quat[1]);
		esb_log_print("%i,", quat[2]);
		esb_log_print("%i,", quat[3]);
	#endif
	
	#ifdef ACC
		esb_log_print("%i,", accel[0]);
		esb_log_print("%i,", accel[1]);
		esb_log_print("%i,", accel[2]);
	#endif
	
	#ifdef GYRO
		esb_log_print("%i,", gyro[0]);
		esb_log_print("%i,", gyro[1]);
		esb_log_print("%i,", gyro[2]);
	#endif	
	esb_log_print("%i\r\n", ret);

	return ret;
}

Acc mpu_get_acc(void) {
	
	int ret;
	Acc acc;
	
	unsigned char data[7];
	
	ret = nrf_twi_read(MPU9150_ADDR,ACCEL_XOUT_H, 6, data);
	if (ret != 0) {
		acc.status = -1;
		return acc;
	}	

	acc.x = (data[0]<<8) + data[1];
	acc.y = (data[2]<<8) + data[3];
	acc.z = (data[4]<<8) + data[5];
	acc.status = 0;
	
	return acc;
		
}

Gyro mpu_get_gyro(void) {
	
	int ret;
	Gyro gyro;
	
	unsigned char data[7];
	
	ret = nrf_twi_read(MPU9150_ADDR,GYRO_XOUT_H, 6, data);
	if (ret != 0) {
		gyro.status = -1;
		return gyro;
	}	

	gyro.x = (data[0]<<8) + data[1];
	gyro.y = (data[2]<<8) + data[3];
	gyro.z = (data[4]<<8) + data[5];
	gyro.status = 0;
	
	return gyro;
		
}




Acc mpu_get_acc_old(void) {
	
	uint8_t high_byte, low_byte;
	
	int ret;
	Acc acc;
	
	//Ask for upper 8 bytes of x-plane accerometer data
	ret = nrf_twi_read(MPU9150_ADDR,ACCEL_XOUT_H, 1, &high_byte);
	if (ret) {
		acc.status = -1;
		return acc;
	}
	ret = nrf_twi_read(MPU9150_ADDR,ACCEL_XOUT_L, 1, &low_byte);
	if (ret) {
		acc.status = -1;
		return acc;
	}
	acc.x = (high_byte<<8) + low_byte;
	
	//Ask for upper 8 bytes of y-plane accerometer data
	ret = nrf_twi_read(MPU9150_ADDR,ACCEL_YOUT_H, 1, &high_byte);
	if (ret) {
		acc.status = -1;
		return acc;
	}
	ret = nrf_twi_read(MPU9150_ADDR,ACCEL_YOUT_L, 1, &low_byte);
	if (ret) {
		acc.status = -1;
		return acc;
	}
	acc.y = (high_byte<<8) + low_byte;
	
	
	//Ask for upper 8 bytes of z-plane accerometer data
	ret = nrf_twi_read(MPU9150_ADDR,ACCEL_ZOUT_H, 1, &high_byte);
	if (ret) {
		acc.status = -1;
		return acc;
	}
	ret = nrf_twi_read(MPU9150_ADDR,ACCEL_ZOUT_L, 1, &low_byte);
	if (ret) {
		acc.status = -1;
		return acc;
	}
	acc.z = (high_byte<<8) + low_byte;
	
	return acc;
	
}


Gyro mpu_get_gyro_old(void) {
	uint8_t high_byte, low_byte;
	
	int ret;
	Gyro gyro;
	
	//Ask for upper 8 bytes of x-plane gyroerometer data
	high_byte = twi_write_read(MPU9150_ADDR,GYRO_XOUT_H);
	if (ret == -1) {
		gyro.status = -1;
		return gyro;
	}
	low_byte = twi_write_read(MPU9150_ADDR,GYRO_XOUT_L);
	if (ret == -1) {
		gyro.status = -1;
		return gyro;
	}
	gyro.x = (high_byte<<8) + low_byte;
	
	//Ask for upper 8 bytes of y-plane gyroerometer data
	high_byte = twi_write_read(MPU9150_ADDR,GYRO_YOUT_H);
	if (ret == -1) {
		gyro.status = -1;
		return gyro;
	}
	low_byte = twi_write_read(MPU9150_ADDR,GYRO_YOUT_L);
	if (ret == -1) {
		gyro.status = -1;
		return gyro;
	}
	gyro.y = (high_byte<<8) + low_byte;
	
	
	//Ask for upper 8 bytes of z-plane gyroerometer data
	high_byte = twi_write_read(MPU9150_ADDR,GYRO_ZOUT_H);
	if (ret == -1) {
		gyro.status = -1;
		return gyro;
	}
	low_byte = twi_write_read(MPU9150_ADDR,GYRO_ZOUT_L);
	if (ret == -1) {
		gyro.status = -1;
		return gyro;
	}
	gyro.z = (high_byte<<8) + low_byte;
	
	return gyro;
	
}




