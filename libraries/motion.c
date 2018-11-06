#include "motion.h"

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
#include "logger.h"
#include "app_timer.h"

//mpu9250 drivers
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"

//mpl
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"

#define MPU9150_ADDR 0x68
#define WHO_AM_I 0x75

#define NRF_LOG_MODULE_NAME motion
#define NRF_LOG_LEVEL 4
#define NRF_LOG_INFO_COLOR 0
#define NRF_LOG_DEBUG_COLOR 0
#include "nrf_log.h"

NRF_LOG_MODULE_REGISTER();

Motion motion;

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


APP_TIMER_DEF(m_orientation_timer_id);                         /**< Quaternion Orientation timer. */
APP_TIMER_DEF(m_compass_timer_id);                         /**< Quaternion Orientation timer. */


long gyro_bias[3] = GYRO_BIAS;
const long accel_bias[3] = ACCEL_BIAS;

uint32_t fifo_num = 0;

static void motion_timeout_handler(void * p_context);
static void compass_timeout_handler(void * p_context);
static void motion_timers_init(void);
static void motion_timers_start(void);
static void motion_timers_stop(void);
static uint8_t motion_get_sample(MotionSample *motion_sample);

int imu_init(MotionInit * motion_init) {
	
	int ret;
	inv_error_t result; 	
	unsigned short gyro_rate, gyro_fsr, compass_fsr;
	unsigned char accel_fsr;
	struct int_param_s int_param =
	{
		.cb = NULL,
		.pin = 0,
		.lp_exit = 0,
		.active_low = 1,
	};
		
	unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT;
	
	motion.motion_event_cb = motion_init->event_cb;
	motion.motion_sample_interval = APP_TIMER_TICKS(ORIENTATION_MEAS_INTERVAL_MS);
	motion.compass_sample_interval = APP_TIMER_TICKS(COMPASS_MEAS_INTERVAL_MS);	
	
	motion_timers_init();
	
	if (motion_init->sensor_config & SENSOR_SAMPLE_RAW_IMU) {
		dmp_features |= DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO;		
	}
	
	if (motion_init->sensor_config & SENSOR_USE_GYRO_CAL) {
		dmp_features |= DMP_FEATURE_GYRO_CAL;		
	}	
	
	//mpu init
	ret = mpu_init(&int_param);
	if (ret != 0) {
		NRF_LOG_ERROR("mpu_init ret:%d",ret);
		return ret;
	}
	
	//inv setup
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
	
	//config
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

	mpu_set_sample_rate(MPU_INTERAL_RATE);

	mpu_set_compass_sample_rate(COMPASS_SAMPLE_RATE_HZ);
	
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
	inv_set_compass_sample_rate(1000000L / COMPASS_SAMPLE_RATE_HZ);		

	
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
	
	//dmp
	ret = dmp_load_motion_driver_firmware();
	if (ret != 0) {
		NRF_LOG_ERROR("dmp_load_motion_driver_firmware ret:%d",ret);		
		return ret;		
	}	

    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));

    dmp_register_tap_cb(NULL);
	dmp_register_android_orient_cb(NULL);

	ret = dmp_enable_feature(dmp_features);
	
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
	

	NRF_LOG_DEBUG("IMU INIT SUCCESS",ret);

	return 0;
}



int imu_start(void) {
	int ret;
	ret = mpu_set_dmp_state(1);
	if (ret != 0) {
		NRF_LOG_ERROR("mpu_set_dmp_state ret:%d",ret);		
		return ret;		
	}
	motion_timers_start();

	return ret;
}

int imu_stop(void) {
	int ret;
	ret = mpu_set_dmp_state(0);
	if (ret != 0) {
		NRF_LOG_ERROR("mpu_set_dmp_state ret:%d",ret);		
		return ret;		
	}
	motion_timers_stop();

	return ret;
}	

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
void motion_timers_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_create(&m_orientation_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                motion_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_create(&m_compass_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                compass_timeout_handler);
    APP_ERROR_CHECK(err_code);	
}


/**@brief Function for starting application timers.
 */
void motion_timers_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_start(m_orientation_timer_id, motion.motion_sample_interval, NULL);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_start(m_compass_timer_id, motion.compass_sample_interval, NULL);
    APP_ERROR_CHECK(err_code);	
}

/**@brief Function for starting application timers.
 */
void motion_timers_stop(void)
{
    ret_code_t err_code;

    err_code = app_timer_stop(m_orientation_timer_id);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_stop(m_compass_timer_id);
    APP_ERROR_CHECK(err_code);	
}


void mpu_self_test(void) {
	
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

static uint8_t motion_get_sample(MotionSample * motion_sample) {

	short sensors;
	uint8_t more;
	unsigned long compass_timestamp;
	int compass_status;
	
	//Get quaternion and IMU data from fifo
	//Check status
	//Get number of remaining samples in fifo
	motion_sample->event = fifo_num++;
	
	motion_sample->status = dmp_read_fifo(motion_sample->gyro, motion_sample->accel, (long *) motion_sample->quat, (unsigned long *) &motion_sample->timestamp, &sensors, &more);
	NRF_LOG_DEBUG("Motion Sample @ %d ms",motion_sample->timestamp);	
	if (motion_sample->status) {
		NRF_LOG_ERROR("dmp_read_fifo ret: %d",motion_sample->status);
		return 0;
	}
	if (sensors & INV_WXYZ_QUAT) {
		motion_sample->data_flags |= QUATERNION_DATA;
	}
	if (sensors & INV_XYZ_GYRO && sensors & INV_XYZ_ACCEL) {
		motion_sample->data_flags |= IMU_DATA;
	}
	
	//if compass ready, sample compass
	if (motion.compass_ready) {
		compass_status = mpu_get_compass_reg(motion_sample->compass, &compass_timestamp);
		if (compass_status) {
			NRF_LOG_DEBUG("mpu_get_compass_reg ret: ",compass_status);		
		}		
		NRF_LOG_DEBUG("Compass Sample @ %d ms",compass_timestamp);
		motion_sample->data_flags |= COMPASS_DATA;
		motion.compass_ready = 0;
	}
	
	return more;
}


static void motion_timeout_handler(void * p_context) {
	NRF_LOG_DEBUG("Motion Timeout Handler");
	
	static MotionSample motion_sample;
	memset(&motion_sample,0,sizeof(MotionSample));
	
	uint8_t samples_ready;
	do {
		samples_ready = motion_get_sample(&motion_sample);
		motion.motion_event_cb(&motion_sample);
	} while (samples_ready);
		

}

static void compass_timeout_handler(void * p_context) {
	NRF_LOG_DEBUG("Compass Timeout Handler");
	motion.compass_ready = 1;
}
