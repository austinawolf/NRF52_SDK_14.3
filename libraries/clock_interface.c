#include "clock_interface.h"
#include "FreeRTOS.h"
#include "task.h"

void nrf_get_ms(unsigned long *timestamp) {
	if (timestamp == NULL) return;
	
	*timestamp = xTaskGetTickCount();
	return;
}
