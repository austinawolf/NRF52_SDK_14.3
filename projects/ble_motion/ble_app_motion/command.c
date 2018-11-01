#include "command.h"

#include <stdbool.h>
#include <string.h>

#define NRF_LOG_MODULE_NAME cmd
#define NRF_LOG_LEVEL       3
#define NRF_LOG_INFO_COLOR  0
#define NRF_LOG_DEBUG_COLOR 0
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();


static SdsReturnType get_fw_version(Command * p_command, Response * p_response);
static SdsReturnType get_sensor_location(Command * p_command, Response * p_response);
static SdsReturnType get_sample_rate(Command * p_command, Response * p_response);
static SdsReturnType set_sample_rate(Command * p_command, Response * p_response);
	
static SdsReturnType (*CommandSet[4]) (Command * command, Response * response) =
{
	get_fw_version,
	get_sensor_location,
	get_sample_rate,
	set_sample_rate,
};


SdsReturnType command_call(Command * p_command, Response * p_response) {
	SdsReturnType ret = SDS_SUCCESS;


	ret = CommandSet[p_command->opcode - BASE_OFFSET](p_command, p_response);

	
	return ret;
}


static SdsReturnType get_fw_version(Command * p_command, Response * p_response) {
	NRF_LOG_INFO("get_fw_version");

	//load response arguments
	const uint8_t resp_arg_len = 4;
	static uint8_t fw_version[resp_arg_len] = {0x0a, 0x0b , 0x0c, 0x0d};

	p_response->preamble = RESPONSE_PREAMBLE;
	p_response->opcode = p_command->opcode;
	p_response->arg_len = resp_arg_len;
	p_response->p_args = fw_version;
	p_response->err_code = SDS_SUCCESS;
	
	return SDS_SUCCESS;
}

static SdsReturnType get_sensor_location(Command * p_command, Response * p_response) {
	NRF_LOG_INFO("get_sensor_location");

	return SDS_SUCCESS;
}

static SdsReturnType get_sample_rate(Command * p_command, Response * p_response) {
	NRF_LOG_INFO("get_sample_rate");
	
	return SDS_SUCCESS;
}

static SdsReturnType set_sample_rate(Command * p_command, Response * p_response) {
	NRF_LOG_INFO("set_sample_rate");
	
	return SDS_SUCCESS;
}
