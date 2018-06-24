
#ifndef NRF_LOG_HELPER_H_
#define NRF_LOG_HELPER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/*
DEVELOPMENT ENVIRONMENT REQUIREMENTS (DRAFT)

Header Paths:
..\..\..\..\..\..\components\libraries\experimental_log
..\..\..\..\..\..\components\libraries\experimental_log\src
..\..\..\..\..\..\components\libraries\cli
..\..\..\..\..\..\components\libraries\cli\uart

Source Files:
nrf_log_frontend.c
nrf_log_str_formatter.c
nrf_log_backend_rtt.c
nrf_log_backend_serial.c
nrf_log_backend_uart.c
nrf_log_default_bankends.c

*/

void logger_init(void);
void logger_write(void);

#endif
