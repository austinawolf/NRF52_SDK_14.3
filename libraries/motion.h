#ifndef MOTION_H_
#define MOTION_H_
#include "ble_motion.h"

#define ORIENTATION_MEAS_INTERVAL 			APP_TIMER_TICKS(1000/IMU_SAMPLE_RATE_HZ)          /**< Heart rate measurement interval (ticks). */
#define SENSOR_CONTACT_DETECTED_INTERVAL    APP_TIMER_TICKS(5000)                   /**< Sensor Contact Detected toggle interval (ticks). */
#define COMPASS_MEAS_INTERVAL 				APP_TIMER_TICKS(1000/COMPASS_SAMPLE_RATE_HZ)          /**< Heart rate measurement interval (ticks). */

extern ble_motion_t m_motion;

void motion_timers_init(void);
void motion_timers_start(void);
void motion_timers_stop(void);
void motion_service_init(void);

#endif
