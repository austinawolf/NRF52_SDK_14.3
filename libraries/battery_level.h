#ifndef BATTERY_LEVEL_H_
#define BATTERY_LEVEL_H_

#include "ble_bas.h"

extern ble_bas_t m_bas;

void battery_level_meas_timeout_handler(void * p_context);

#endif
