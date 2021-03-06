
#ifndef LOGGER_H_
#define LOGGER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/*
DEVELOPMENT ENVIRONMENT REQUIREMENTS (DRAFT)

Header Paths:


Source Files:


*/

#if defined BLE_LOGGER
    #include "ble_logger.h"
    #define LOG         ble_log
	#define LOG_PRINT	ble_log_print
    #define LOG_INIT    ble_logger_init
#elif defined ESB_LOGGER
    #include "esb_logger.h"
    #define LOG         esb_log
	#define LOG_PRINT	esb_log_print
    #define LOG_INIT    esb_logger_init
	#define LOG_FLUSH 	esb_log_flush
#elif defined UART_LOGGER
    #include "uart_helper.h"
	#define LOG_PRINT	printf
    #define LOG_INIT    uart_init
#elif defined RTT_LOGGER
	#include "SEGGER_RTT.h"
	#define LOG_PRINT	SEGGER_RTT_printf0
#else


    //#warning NO LOGGER DEFINED
#endif

#endif

