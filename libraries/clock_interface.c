#include "clock_interface.h"
#include "app_timer.h"

#define NRF_LOG_MODULE_NAME mpu
#if MPU_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL MPU_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR MPU_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR MPU_CONFIG_DEBUG_COLOR
#else //SPI_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //SPI_CONFIG_LOG_ENABLED
#include "nrf_log.h"


void nrf_get_ms(unsigned long *timestamp) {
	if (timestamp == NULL) return;
	uint32_t count = app_timer_cnt_get();
	
	*timestamp = count  * ( (APP_TIMER_CONFIG_RTC_FREQUENCY + 1 ) * 1000 ) / APP_TIMER_CLOCK_FREQ;
	
	return;
}
