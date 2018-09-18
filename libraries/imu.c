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
#include "clock_interface.h"

//mpu9250 drivers
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "logger.h"

//mpl
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"

#define MPU9150_ADDR 0x68
#define AK8963_ADDR 0x0C

#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B
#define WHO_AM_I 0x75


#define NRF_LOG_MODULE_NAME mpu
#if MPU_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL MPU_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR MPU_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR MPU_CONFIG_DEBUG_COLOR
#else //SPI_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //SPI_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

static struct platform_data_s compass_pdata = {
    .orientation = {0, 1, 0,
					1, 0, 0,
					0, 0,-1}

};

/*
{ 0, 1, 0,
 1, 0, 0,
 0, 0,-1}

*/

uint32_t fifo_num = 0;

int mpu_helper_init(void);
int mpu_helper_dmp_setup(void);
int	mpu_helper_inv_setup(void);

int imu_init(void) {
	
	int ret;

	struct int_param_s int_param;
	int_param.cb = NULL;
	int_param.pin = 0;
	int_param.lp_exit = 0;
	int_param.active_low = 1;

	//mpu init
	ret = mpu_init(&int_param);
	if (ret != 0) {
		NRF_LOG_DEBUG("mpu_init ret:%d",ret);
		return ret;
	}
	
	//inv setup
	ret = mpu_helper_inv_setup();
	if (ret != 0) {
		NRF_LOG_DEBUG("mpu_helper_inv_setup ret:%d",ret);
		return ret;
	}
	
	//config
	ret = mpu_helper_init();
	if (ret != 0) {
		NRF_LOG_DEBUG("mpu_helper_init ret:%d",ret);
		return ret;
	}	
	
	//dmp
	ret = mpu_helper_dmp_setup();
	if (ret != 0) {
		NRF_LOG_DEBUG("mpu_helper_dmp_setup ret:%d",ret);		
		return ret;
	}

	NRF_LOG_DEBUG("IMU INIT SUCCESS",ret);


	return 0;
}

int mpu_helper_init(void) {
	
		int ret;
		unsigned short gyro_rate, gyro_fsr, compass_fsr;
		unsigned char accel_fsr;

		ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
		if (ret != 0) {
			NRF_LOG_ERROR("mpu_set_sensors ret:%d",ret);		
			return ret;		
		}	
		
		ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
		if (ret != 0) {
			NRF_LOG_ERROR("mpu_configure_fifo ret:%d",ret);		
			return ret;		
		}			

		mpu_set_sample_rate(DMP_RATE);

		#define COMPASS_READ_MS 100
		mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
		
		mpu_get_sample_rate(&gyro_rate);
		mpu_get_gyro_fsr(&gyro_fsr);
		mpu_get_accel_fsr(&accel_fsr);
		mpu_get_compass_fsr(&compass_fsr);

		NRF_LOG_DEBUG("Gyro Rate: %d",gyro_rate);
		NRF_LOG_DEBUG("Gyro FSR: %d",gyro_fsr);
		NRF_LOG_DEBUG("Accel FSR: %d",accel_fsr);
		NRF_LOG_DEBUG("Compass FSR: %d",compass_fsr);

		inv_set_gyro_sample_rate(1000000L / gyro_rate);				
		inv_set_accel_sample_rate(1000000L / gyro_rate);					
		inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);		

		inv_set_gyro_orientation_and_scale(
				inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
				(long)gyro_fsr<<15);	
	
		inv_set_accel_orientation_and_scale(
				inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
				(long)accel_fsr<<15);
		
		inv_set_compass_orientation_and_scale(
				inv_orientation_matrix_to_scalar(compass_pdata.orientation),
				(long)compass_fsr<<15);
	
	
		//const long accel_bias[3] = {0, 0, 0};
		//const long accel_bias[3] = {33, 12, -162};
		//const long accel_bias[3] = {5, 20, -200};
		//mpu_set_accel_bias_6500_reg(accel_bias);

		//long gyro_bias[3] = {0, 0, 0};
		//long gyro_bias[3] = {-40, 1, 3};
		//long gyro_bias[3] = {-45, 20, -58};
		//mpu_set_gyro_bias_reg(gyro_bias);

		return 0;

}

int mpu_helper_dmp_setup(void) {
    int ret;

	ret = dmp_load_motion_driver_firmware();
	if (ret != 0) {
		NRF_LOG_ERROR("mpu_set_sensors ret:%d",ret);		
		return ret;		
	}	

    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));

    dmp_register_tap_cb(NULL);
	dmp_register_android_orient_cb(NULL);

	ret = dmp_enable_feature(	DMP_FEATURE_6X_LP_QUAT |
								DMP_FEATURE_SEND_RAW_ACCEL | 
								DMP_FEATURE_SEND_CAL_GYRO |
								DMP_FEATURE_GYRO_CAL);
	
	if (ret != 0) {
		NRF_LOG_ERROR("mpu_set_sensors ret:%d",ret);		
		return ret;		
	}	
	

	ret = dmp_set_fifo_rate(DMP_RATE);
	if (ret != 0) {
		NRF_LOG_ERROR("mpu_set_sensors ret:%d",ret);		
		return ret;		
	}	
	
	inv_set_quat_sample_rate(INV_RATE);
	
	ret = mpu_set_dmp_state(1);
	if (ret != 0) {
		NRF_LOG_ERROR("mpu_set_sensors ret:%d",ret);		
		return ret;		
	}		
	
	return ret;
}


int	mpu_helper_inv_setup(void) {
	inv_error_t result; 

	result = inv_init_mpl();
	if (result) {
		NRF_LOG_ERROR("inv_init_mpl ret:%d",result);
		return result;		
    }

	result = inv_enable_quaternion();
	if (result) {
		NRF_LOG_ERROR("inv_enable_quaternion ret:%d",result);
		return result;		
    }
	
    result = inv_enable_9x_sensor_fusion();
	if (result) {
		NRF_LOG_ERROR("inv_enable_9x_sensor_fusion ret:%d",result);
		return result;		
    }

	result = inv_enable_fast_nomot();
	if (result) {
		NRF_LOG_ERROR("inv_enable_fast_nomot ret:%d",result);
		return result;		
    }	

    result = inv_enable_gyro_tc();
	if (result) {
		NRF_LOG_ERROR("inv_enable_gyro_tc ret:%d",result);
		return result;		
    }
    
	result = inv_enable_vector_compass_cal();
	if (result) {
		NRF_LOG_ERROR("inv_enable_vector_compass_cal ret:%d",result);
		return result;		
    }    
	
	result = inv_enable_magnetic_disturbance();	
	if (result) {
		NRF_LOG_ERROR("inv_enable_magnetic_disturbance ret:%d",result);
		return result;		
    }

	result = inv_enable_eMPL_outputs();
	if (result) {
		NRF_LOG_ERROR("inv_enable_eMPL_outputs ret:%d",result);
		return result;		
    }	
	
	result = inv_start_mpl();
    if (result == INV_ERROR_NOT_AUTHORIZED) {
		NRF_LOG_ERROR("inv_start_mpl ret:INV_ERROR_NOT_AUTHORIZED");
		return result;
    }
    else if (result) {
		NRF_LOG_ERROR("inv_start_mpl ret:%d",result);
		return result;		
    }
	
	
	return 0;
}

void mpu_compass_config(void) {
	mpu_set_bypass(0);
}

void imu_self_test(void) {
	
    int result;
    long gyro[3], accel[3];

    result = mpu_run_6500_self_test(gyro, accel, 0);

    if (result == 0x7) {
		NRF_LOG_INFO("Passed!\n");
        NRF_LOG_INFO("accel: %7.4f %7.4f %7.4f\n",
                    accel[0]/65536.f,
                    accel[1]/65536.f,
                    accel[2]/65536.f);
        NRF_LOG_INFO("gyro: %7.4f %7.4f %7.4f\n",
                    gyro[0]/65536.f,
                    gyro[1]/65536.f,
                    gyro[2]/65536.f);
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

		
        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
    	unsigned short accel_sens;
    	float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
    }
    else {
            if (!(result & 0x1))
                NRF_LOG_INFO("Gyro failed.\n");
            if (!(result & 0x2))
                NRF_LOG_INFO("Accel failed.\n");
            if (!(result & 0x4))
                NRF_LOG_INFO("Compass failed.\n");
     }

}


void imu_log_fifo(void) {

	short gyro[3], accel_short[3], sensors;
	unsigned char more;
	long accel[3], quat[4], temperature;	
    unsigned long sensor_timestamp;
	inv_error_t result; 
	
    dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
	
	NRF_LOG_DEBUG("Sample @ %d ms",sensor_timestamp);	
	NRF_LOG_DEBUG("Gyro X:%d, Gyro Y:%d ,Gyro Z:%d", gyro[0],gyro[1],gyro[2]);
	NRF_LOG_DEBUG("Acc X:%d, Acc Y:%d ,Acc Z:%d", accel_short[0],accel_short[1],accel_short[2]);
	NRF_LOG_DEBUG("q0:%d, q1:%d , q2:%d, q3:%d", quat[0],quat[1],quat[2],quat[3]);
	

	result = inv_build_gyro(gyro, sensor_timestamp);
	if (result) {
		NRF_LOG_ERROR("inv_build_gyro ret:%d",result);		
    }	
	
	mpu_get_temperature(&temperature, &sensor_timestamp);
    result = inv_build_temp(temperature, sensor_timestamp);
	if (result) {
		NRF_LOG_ERROR("inv_build_temp ret:%d",result);		
    }	
	accel[0] = (long)accel_short[0];
	accel[1] = (long)accel_short[1];
	accel[2] = (long)accel_short[2];
	
	result = inv_build_accel(accel, 0, sensor_timestamp);	
	if (result) {
		NRF_LOG_ERROR("inv_build_accel ret:%d",result);		
    }	


	result = inv_build_quat(quat, 0, sensor_timestamp);
	if (result) {
		NRF_LOG_ERROR("inv_build_quat ret:%d",result);		
    }	
	
	short compass_short[3];
	long compass[3];

	if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
		compass[0] = (long)compass_short[0];
		compass[1] = (long)compass_short[1];
		compass[2] = (long)compass_short[2];
	}
	
	result = inv_build_compass(compass, 0, sensor_timestamp);
	if (result) {
		NRF_LOG_ERROR("inv_build_compass ret:%d",result);		
    }	
	
    long data[9];
    int8_t accuracy;
    unsigned long timestamp;

	
	result = inv_execute_on_data();
	if (result) {
		NRF_LOG_ERROR("inv_execute_on_data ret:%d",result);		
    }	
		
	result = inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp);
	
	#define UPDATED 1
	if (result != UPDATED) {
		NRF_LOG_ERROR("inv_get_sensor_type_quat ret:%d",result);		
    }
	else {
		NRF_LOG_DEBUG("accuracy: %d, q0: %d, q1: %d, q2: %d, q3: %d", accuracy, data[0], data[1], data[2], data[3]);
	}
}

Motion get_motion_data(void) {

	Motion motion;
	nrf_get_ms(&motion.sensor_timestamp);
	NRF_LOG_DEBUG("Sample @ %d ms",motion.sensor_timestamp);
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



