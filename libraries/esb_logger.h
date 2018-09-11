
#ifndef NRF_ESB_LOGGER_H_
#define NRF_ESB_LOGGER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "nrf_esb.h"




/*
DEVELOPMENT ENVIRONMENT REQUIREMENTS (DRAFT)

Header Paths:


Source Files:


*/





uint32_t esb_init(void);
void esb_logger_init(void);
void esb_log_print(const char* format, ...);
void esb_log_flush(void);
	


#endif

