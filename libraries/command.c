#include "command.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>


#define NRF_LOG_MODULE_NAME cmd
#if ESB_CONFIG_CMD_ENABLED
#define NRF_LOG_LEVEL CMD_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR CMD_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR CMD_CONFIG_DEBUG_COLOR
#else
#define NRF_LOG_LEVEL 0
#endif
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();



