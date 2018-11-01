#ifndef MOTION_H_
#define MOTION_H_
#include "ble_motion.h"

extern ble_motion_t m_motion;

void orientation_meas_timeout_handler(void * p_context);
void sensor_contact_detected_timeout_handler(void * p_context);
void motion_service_init(void);

#endif
