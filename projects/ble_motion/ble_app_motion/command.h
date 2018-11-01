
#ifndef NRF_COMMAND_H_
#define NRF_COMMAND_H_

#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

#define COMMAND_PREAMBLE 0xbb
#define RESPONSE_PREAMBLE 0xcc

#define MIN_COMMAND_LEN 3
#define MIN_RESPONSE_LEN 4


typedef enum
{
	SDS_SUCCESS,
	SDS_ERROR,
	SDS_INVALID_ARG,
	SDS_INVALID_ARG_LEN,
	SDS_INVALID_OPCODE,
	SDS_INVALID_PREAMBLE,
	SDS_SHORT_COMMAND,
} SdsReturnType;

typedef enum
{
	BASE_OFFSET = 0x20,
	MIN_OPCODE_VAL = BASE_OFFSET,
	GET_FW_VERSION = MIN_OPCODE_VAL,
	GET_SENSOR_LOCATION,
	GET_SAMPLE_RATE,
	SET_SAMPLE_RATE,
	MAX_OPCODE_VAL = SET_SAMPLE_RATE,
} OPCODE;

typedef struct {
	const uint8_t		preamble;
	const OPCODE 		opcode;
	const uint8_t		arg_len;	
	const uint8_t * 	p_args;
} Command;

	
typedef struct {
	uint8_t				preamble;
	OPCODE 				opcode;
	SdsReturnType		err_code;	
	uint8_t				arg_len;
	uint8_t * 			p_args;
} Response;

SdsReturnType command_call(Command * p_command, Response * p_response);

#endif

