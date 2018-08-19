
#ifndef NRF_BLE_LOGGER_H_
#define NRF_BLE_LOGGER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "bsp.h"

/*
DEVELOPMENT ENVIRONMENT REQUIREMENTS (DRAFT)

Header Paths:


Source Files:


*/

#define BLE_LOGGER_MAX_DATA_SIZE 32

void ble_logger_init(void);
uint32_t ble_log_print(const char* format, ...);

#endif

