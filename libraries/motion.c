#include "motion.h"

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "imu.h"
#include "ble_gatts.h"

/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
void orientation_meas_timeout_handler(void * p_context)
{
    static uint32_t cnt = 0;
    ret_code_t      err_code;

    UNUSED_PARAMETER(p_context);

    cnt++;
	
	Motion motion_data;
	
	#ifdef IMU_ENABLED
	imu_get_data(&motion_data);
	if (motion_data.status) {
		return;	
	}	
	imu_get_compass(&motion_data);
	imu_send_to_mpl(&motion_data);
	#else
	motion_data.quat[0] = 0xa;
	motion_data.quat[1] = 0xb;
	motion_data.quat[2] = 0xc;
	motion_data.quat[3] = 0xd;
	#endif
	
	//NRF_LOG_DEBUG("Orientation: q0 = %d, q1 = %d, q2 = %d, q3 = %d", motion_data.quat[0], motion_data.quat[1], motion_data.quat[2], motion_data.quat[3]);

	
    err_code = ble_motion_orientation_send(&m_motion, &motion_data);
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
void sensor_contact_detected_timeout_handler(void * p_context)
{
    static bool sensor_contact_detected = false;

    UNUSED_PARAMETER(p_context);

    sensor_contact_detected = !sensor_contact_detected;
    ble_motion_sensor_contact_detected_update(&m_motion, sensor_contact_detected);
}
