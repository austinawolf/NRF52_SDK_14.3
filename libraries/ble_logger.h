
#ifndef NRF_BLE_LOGGER_H_
#define NRF_BLE_LOGGER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "queue.h"



/*
DEVELOPMENT ENVIRONMENT REQUIREMENTS (DRAFT)

Header Paths:


Source Files:


*/





uint32_t ble_init(void);
void ble_logger_init(void);
uint32_t ble_log(uint8_t data);
uint32_t ble_log_print(const char* format, ...);

	


#endif

