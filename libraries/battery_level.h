#ifndef BATTERY_LEVEL_H_
#define BATTERY_LEVEL_H_

#include "ble_bas.h"

#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(5000)                   /**< Battery level measurement interval (ticks). */

extern ble_bas_t m_bas;

void battery_level_timers_init(void);
void battery_level_timers_start(void);
void battery_level_timers_stop(void);

void battery_level_meas_timeout_handler(void * p_context);



#endif
