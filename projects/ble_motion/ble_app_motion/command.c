#include "command.h"

#include <stdbool.h>
#include <string.h>
#include "motion.h"

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
static SdsReturnType run_motion_cal(Command * p_command, Response * p_response);
static SdsReturnType get_sample_state(Command * p_command, Response * p_response);
static SdsReturnType set_sample_state(Command * p_command, Response * p_response);

static SdsReturnType (*CommandSet[MAX_OPCODE_VAL - BASE_OFFSET + 1]) (Command * command, Response * response) =
{
	get_fw_version,
	get_sensor_location,
	get_sample_rate,
	set_sample_rate,
	run_motion_cal,
	get_sample_state,
	set_sample_state,
};


SdsReturnType command_call(Command * p_command, Response * p_response) {
	SdsReturnType ret = SDS_SUCCESS;
	
	

	ret = CommandSet[p_command->opcode - BASE_OFFSET](p_command, p_response);

	
	return ret;
}


static SdsReturnType get_fw_version(Command * p_command, Response * p_response) {
	NRF_LOG_INFO("Command: get_fw_version");

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
	
	//load response arguments
	static uint8_t sample_rate;
	sample_rate = motion_get_sample_rate();

	p_response->preamble = RESPONSE_PREAMBLE;
	p_response->opcode = p_command->opcode;
	p_response->arg_len = 1;
	p_response->p_args = &sample_rate;
	p_response->err_code = SDS_SUCCESS;

	return SDS_SUCCESS;
}

static SdsReturnType set_sample_rate(Command * p_command, Response * p_response) {
	NRF_LOG_INFO("Command: set_sample_rate");
	
	if (p_command -> arg_len != 1) {
		return SDS_INVALID_ARG_LEN;
	}
	
	SAMPLE_RATE sample_rate = (SAMPLE_RATE) p_command -> p_args[0];
	
	if ( sample_rate > MAX_SAMPLE_RATE ) {
		return SDS_INVALID_ARG;
	}
	
	motion_set_sample_rate(sample_rate);
	
	p_response->preamble = RESPONSE_PREAMBLE;
	p_response->opcode = p_command->opcode;
	p_response->arg_len = 0;
	p_response->p_args = NULL;
	p_response->err_code = SDS_SUCCESS;
	
	return SDS_SUCCESS;
}

static SdsReturnType run_motion_cal(Command * p_command, Response * p_response) {

	NRF_LOG_INFO("Command: run_motion_cal");
	
	if (p_command -> arg_len != 0) {
		return SDS_INVALID_ARG_LEN;
	}
	
	motion_run_imu_cal(10);
	
	p_response->preamble = RESPONSE_PREAMBLE;
	p_response->opcode = p_command->opcode;
	p_response->arg_len = 0;
	p_response->p_args = NULL;
	p_response->err_code = SDS_SUCCESS;
	
	return SDS_SUCCESS;
}

static SdsReturnType get_sample_state(Command * p_command, Response * p_response) {
	NRF_LOG_INFO("get_sample_state");
	
	//load response arguments
	static uint8_t sample_state;
	sample_state = motion_get_sample_state();

	p_response->preamble = RESPONSE_PREAMBLE;
	p_response->opcode = p_command->opcode;
	p_response->arg_len = 1;
	p_response->p_args = &sample_state;
	p_response->err_code = SDS_SUCCESS;

	return SDS_SUCCESS;
}

static SdsReturnType set_sample_state(Command * p_command, Response * p_response) {
	NRF_LOG_INFO("Command: set_sample_state");
	
	if (p_command -> arg_len != 1) {
		return SDS_INVALID_ARG_LEN;
	}
	
	SAMPLE_STATE sample_state = (SAMPLE_STATE) p_command -> p_args[0];
	
	if ( sample_state > MAX_SAMPLE_STATE ) {
		return SDS_INVALID_ARG;
	}
	
	motion_set_sample_state(sample_state);
	
	p_response->preamble = RESPONSE_PREAMBLE;
	p_response->opcode = p_command->opcode;
	p_response->arg_len = 0;
	p_response->p_args = NULL;
	p_response->err_code = SDS_SUCCESS;
	
	return SDS_SUCCESS;
}
