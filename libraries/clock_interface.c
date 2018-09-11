#include "clock_interface.h"
#include "app_timer.h"

void nrf_get_ms(unsigned long *timestamp) {
	if (timestamp == NULL) return;
	
	*timestamp = app_timer_cnt_get()  * ( (APP_TIMER_CONFIG_RTC_FREQUENCY + 1 ) * 1000 ) / APP_TIMER_CLOCK_FREQ;
		
	return;
}
