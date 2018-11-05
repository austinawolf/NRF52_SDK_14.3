#include "motion.h"

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "imu.h"
#include "ble_gatts.h"
#include "app_timer.h"
#include "motion.h"
#include "battery_level.h"

APP_TIMER_DEF(m_orientation_timer_id);                         /**< Quaternion Orientation timer. */
APP_TIMER_DEF(m_sensor_contact_timer_id);                           /**< Sensor contact detected timer. */
APP_TIMER_DEF(m_compass_timer_id);                         /**< Quaternion Orientation timer. */

static void orientation_meas_timeout_handler(void * p_context);
static void sensor_contact_detected_timeout_handler(void * p_context);
static void compass_meas_timeout_handler(void * p_context);

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
void motion_timers_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_create(&m_orientation_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                orientation_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sensor_contact_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_contact_detected_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_create(&m_compass_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                compass_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);	
}

/**@brief Function for starting application timers.
 */
void motion_timers_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_start(m_orientation_timer_id, ORIENTATION_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_start(m_compass_timer_id, COMPASS_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);	
}


/**@brief Function for starting application timers.
 */
void motion_timers_stop(void)
{
    ret_code_t err_code;

    err_code = app_timer_stop(m_orientation_timer_id);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_stop(m_sensor_contact_timer_id);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_stop(m_compass_timer_id);
    APP_ERROR_CHECK(err_code);	
}

/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void orientation_meas_timeout_handler(void * p_context)
{	
    ret_code_t      err_code;
    UNUSED_PARAMETER(p_context);

	Motion motion_sample;
	
	#ifdef IMU_ENABLED
	imu_get_data(&motion_sample);
	if (motion_sample.status) {
		return;	
	}	
	imu_motion_to_mpl(&motion_sample);
	#else
	motion_data.quat[0] = 0xa;
	motion_data.quat[1] = 0xb;
	motion_data.quat[2] = 0xc;
	motion_data.quat[3] = 0xd;
	#endif
	
	//NRF_LOG_DEBUG("Orientation: q0 = %d, q1 = %d, q2 = %d, q3 = %d", motion_data.quat[0], motion_data.quat[1], motion_data.quat[2], motion_data.quat[3]);

	
    err_code = ble_motion_quaternion_send(&m_motion, &motion_sample);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }

    err_code = ble_motion_imu_send(&m_motion, &motion_sample);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
	
	
}

/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void compass_meas_timeout_handler(void * p_context)
{
    ret_code_t      err_code;

    UNUSED_PARAMETER(p_context);
	
	CompassSample compass_sample;
	
	#ifdef IMU_ENABLED
	imu_get_compass(&compass_sample);
	imu_compass_to_mpl(&compass_sample);
	#endif

    err_code = ble_motion_compass_send(&m_motion, &compass_sample);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
	
}


/**@brief Function for handling the Sensor Contact Detected timer timeout.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void sensor_contact_detected_timeout_handler(void * p_context)
{
    static bool sensor_contact_detected = false;

    UNUSED_PARAMETER(p_context);

    sensor_contact_detected = !sensor_contact_detected;
    ble_motion_sensor_contact_detected_update(&m_motion, sensor_contact_detected);
}
