#include "imu.h"

//std lib
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

//nordic
#include "bsp.h"

//local libraries
#include "twi_interface.h"
#include "led_error.h"
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

//config
#include "config.h"

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

long gyro_bias[3] = 		GYRO_BIAS;
const long accel_bias[3] = 	ACCEL_BIAS;
// Offsets applied to raw x/y/z values
const float mag_offsets[3]            = MAG_OFFSETS;
// Soft iron error compensation matrix
const float mag_softiron_matrix[3][3] = MAG_SOFTIRON_MATRIX;
		

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
		NRF_LOG_ERROR("mpu_init ret:%d",ret);
		return ret;
	}
	
	//inv setup
	ret = mpu_helper_inv_setup();
	if (ret != 0) {
		NRF_LOG_ERROR("mpu_helper_inv_setup ret:%d",ret);
		return ret;
	}
	
	//config
	ret = mpu_helper_init();
	if (ret != 0) {
		NRF_LOG_ERROR("mpu_helper_init ret:%d",ret);
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


int	mpu_helper_inv_setup(void) {
	inv_error_t result; 

	result = inv_init_mpl();
	if (result) {
		NRF_LOG_ERROR("inv_init_mpl ret:%d",result);
		return result;		
    }
	
	inv_init_9x_fusion();

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
    
	//result = inv_enable_vector_compass_cal();
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

		mpu_set_sample_rate(IMU_SAMPLE_RATE_HZ);

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
	
	

		mpu_set_accel_bias_6500_reg(accel_bias);
		mpu_set_gyro_bias_reg(gyro_bias);

		return 0;

}

int mpu_helper_dmp_setup(void) {
    int ret;

	ret = dmp_load_motion_driver_firmware();
	if (ret != 0) {
		NRF_LOG_ERROR("dmp_load_motion_driver_firmware ret:%d",ret);		
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
		NRF_LOG_ERROR("dmp_enable_feature ret:%d",ret);		
		return ret;		
	}	
	

	ret = dmp_set_fifo_rate(IMU_SAMPLE_RATE_HZ);
	if (ret != 0) {
		NRF_LOG_ERROR("dmp_set_fifo_rate ret:%d",ret);		
		return ret;		
	}	
	
	inv_set_quat_sample_rate(INV_QUAT_SAMPLE_RATE);
	
	
	return 0;
	
}

int imu_start(void) {
	int ret;
	ret = mpu_set_dmp_state(1);
	if (ret != 0) {
		NRF_LOG_ERROR("mpu_set_dmp_state ret:%d",ret);		
		return ret;		
	}
	return ret;
}

int imu_stop(void) {
	int ret;
	ret = mpu_set_dmp_state(0);
	if (ret != 0) {
		NRF_LOG_ERROR("mpu_set_dmp_state ret:%d",ret);		
		return ret;		
	}
	return ret;
}	
	

int imu_init_madgwick(void) {
	
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

	//sensor init
	ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	if (ret != 0) {
		NRF_LOG_ERROR("mpu_set_sensors ret:%d",ret);		
		return ret;		
	}	
	
	//configure fifo
	ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	if (ret != 0) {
		NRF_LOG_ERROR("mpu_configure_fifo ret:%d",ret);		
		return ret;		
	}			

	//set sample rate
	mpu_set_sample_rate(IMU_SAMPLE_RATE_HZ);

	//compass sample rate
	#define COMPASS_READ_MS 100
	mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);

	//program biases
	#define Ka 1/8
	#define Kg 2


	mpu_set_accel_bias_6500_reg(accel_bias);


	mpu_set_gyro_bias_reg(gyro_bias);	
	
	
	//load dmp
	ret = dmp_load_motion_driver_firmware();
	if (ret != 0) {
		NRF_LOG_ERROR("dmp_load_motion_driver_firmware ret:%d",ret);		
		return ret;		
	}	
	
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));

    dmp_register_tap_cb(NULL);
	dmp_register_android_orient_cb(NULL);

	ret = dmp_enable_feature(	DMP_FEATURE_SEND_RAW_ACCEL | 
								DMP_FEATURE_SEND_RAW_GYRO);
	
	if (ret != 0) {
		NRF_LOG_ERROR("dmp_enable_feature ret:%d",ret);		
		return ret;		
	}	
	
	ret = dmp_set_fifo_rate(IMU_SAMPLE_RATE_HZ);
	if (ret != 0) {
		NRF_LOG_ERROR("dmp_set_fifo_rate ret:%d",ret);		
		return ret;		
	}		
	ret = mpu_set_dmp_state(1);
	if (ret != 0) {
		NRF_LOG_ERROR("mpu_set_dmp_state ret:%d",ret);		
		return ret;		
	}			
	
	return 0;
}



void imu_self_test(void) {
	
    int result;
    long gyro[3], accel[3];

    result = mpu_run_6500_self_test(gyro, accel, 1);

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

void imu_send_to_mpl(Motion *motion) {

	unsigned long timestamp;
	long temperature;
	inv_error_t result;
	long accel[3];
	long compass[3];	
    long data[9];
    int8_t accuracy;
	
	accel[0] = (long) motion->accel[0];
	accel[1] = (long) motion->accel[1];
	accel[2] = (long) motion->accel[2];
	
	result = inv_build_accel(accel, 0, motion->sensor_timestamp);	
	if (result) {
		NRF_LOG_ERROR("inv_build_accel ret:%d",result);		
    }	
	
	result = inv_build_quat(motion->quat, 0, motion->sensor_timestamp);
	if (result) {
		NRF_LOG_ERROR("inv_build_quat ret:%d",result);		
    }		
	
	result = inv_build_gyro(motion->gyro, motion->sensor_timestamp);
	if (result) {
		NRF_LOG_ERROR("inv_build_gyro ret:%d",result);		
    }	
	
	mpu_get_temperature(&temperature, &timestamp);
    result = inv_build_temp(temperature, timestamp);
	if (result) {
		NRF_LOG_ERROR("inv_build_temp ret:%d",result);		
    }

	compass[0] = (long)motion->compass[0];
	compass[1] = (long)motion->compass[1];
	compass[2] = (long)motion->compass[2];
	
	result = inv_build_compass(compass, 0, motion->compass_timestamp);
	if (result) {
		NRF_LOG_ERROR("inv_build_compass ret:%d",result);		
    }	
	
	result = inv_execute_on_data();
	if (result) {
		NRF_LOG_ERROR("inv_execute_on_data ret:%d",result);		
    }	
		
	result = inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp);
	
	#define UPDATED 1
	if (result != UPDATED) {
		NRF_LOG_ERROR("inv_get_sensor_type_quat ret:%d",result);
		return;
	}
	
	motion->quat[0] = data[0];
	motion->quat[1] = data[1];
	motion->quat[2] = data[2];
	motion->quat[3] = data[3];

	result = inv_get_sensor_type_heading(data, &accuracy, (inv_time_t*)&timestamp);
	if (result != UPDATED) {
		NRF_LOG_ERROR("inv_get_sensor_type_quat ret:%d",result);
		return;
	}

	NRF_LOG_DEBUG("Mag State: %d", inv_get_magnetic_disturbance_state());
}

#ifdef LOG_PRINT
#ifdef LOG_FLUSH
void imu_log_data(Motion *motion) {	
	LOG_PRINT("\r\nPacket:%u,",motion->event);
	LOG_PRINT("%u,",motion->sensor_timestamp);
	LOG_PRINT("%d,", motion->quat[0]);
	LOG_PRINT("%d,", motion->quat[1]);
	LOG_PRINT("%d,", motion->quat[2]);
	LOG_PRINT("%d,", motion->quat[3]);
	LOG_PRINT("%d,", motion->accel[0]);
	LOG_PRINT("%d,", motion->accel[1]);
	LOG_PRINT("%d,", motion->accel[2]);
	LOG_PRINT("%d,", motion->gyro[0]);
	LOG_PRINT("%d,", motion->gyro[1]);
	LOG_PRINT("%d,", motion->gyro[2]);
	LOG_PRINT("%.2f,", motion->compass[0] * RAW_MAG_TO_uT);
	LOG_PRINT("%.2f,", motion->compass[1] * RAW_MAG_TO_uT);
	LOG_PRINT("%.2f,", motion->compass[2] * RAW_MAG_TO_uT);	
	LOG_PRINT("%i,",motion->sensor_num);
	LOG_PRINT("%i\r\n", motion->status);
	LOG_FLUSH();
}

void imu_log_motion_cal(Motion *motion) {		
	LOG_PRINT("Raw:");
	LOG_PRINT("%d,%d,%d,",motion->accel[0],motion->accel[1],motion->accel[2]);		
	LOG_PRINT("%d,%d,%d,",motion->gyro[0],motion->gyro[1],motion->gyro[2]);
	LOG_PRINT("%d,%d,%d", (int) motion->compass[0], (int)motion->compass[1], (int) motion->compass[2]);
	LOG_PRINT("\n\r");
	LOG_FLUSH();
}
#endif	
#endif

void imu_get_data(Motion *motion) {
	
	motion->sensor_num = SENSOR_NUM;
	motion->event = fifo_num++;
	motion->status = dmp_read_fifo(motion->gyro, motion->accel, motion->quat, &motion->sensor_timestamp, &motion->sensors, &motion->more);
	if (motion->status) {
		NRF_LOG_DEBUG("dmp_read_fifo ret: %d",motion->status);		
	}	
	NRF_LOG_DEBUG("Motion Sample @ %d ms",motion->sensor_timestamp);

	return;
}

void imu_get_compass(Motion *motion) {
	
	float x, y, z;
	short compass[3];
	
	motion->cstatus = mpu_get_compass_reg( compass, &motion->compass_timestamp);
	if (motion->cstatus) {
		NRF_LOG_DEBUG("mpu_get_compass_reg ret: ",motion->status);		
	}
	NRF_LOG_DEBUG("Compass Sample @ %d ms",motion->compass_timestamp);
	
	NRF_LOG_DEBUG("RAW COMPASS x: %x, y: %x, z: %x", compass[0], compass[1], compass[2]);
	
	
	/* PROCESS COMPASS DATA */
	#ifdef MAG_USE_CAL
	// Apply mag offset compensation (base values in uTesla)
	x = (float) compass[0] - mag_offsets[0];
	y = (float) compass[1] - mag_offsets[1];
	z = (float) compass[2] - mag_offsets[2];
	
	// Apply mag soft iron error compensation
	motion->compass[0] = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
	motion->compass[1] = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
	motion->compass[2] = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
	
	#else	
	motion->compass[0] = (float) compass[0];
	motion->compass[1] = (float) compass[1];
	motion->compass[2] = (float) compass[2];
	#endif	
	
	
	
}


