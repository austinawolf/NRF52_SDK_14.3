
#include "log_helper.h"


void logger_write(void)
{

}

void logger_init(void)
{
		//Log Setup
	  uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("LOGGER HELPER INIT");

}


