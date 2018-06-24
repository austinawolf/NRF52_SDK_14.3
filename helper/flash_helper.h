
#ifndef NRF_FLASH_HELPER_H_
#define NRF_FLASH_HELPER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "nrf_drv_spi.h"

#define COMMAND_LEN 1 
#define ADDR_LEN 3
#define PAGE_LEN 256
#define FRAME_LEN COMMAND_LEN + ADDR_LEN

#define STATUS_REG 0x05
#define RD_FLAG_STATUS_REG 0x70
#define ENH_VOL_CONFIG_REG 0x65
#define WRITE_ENABLE 0x06
#define MEM_ADDR 0x000000
#define RESET_ENABLE 0x66
#define RESET_MEMORY 0x99
#define RD_NONVOL_CONFIG 0xB5
#define SPI_FLASH_INS_RDID 0x9E

typedef enum
{
	RetSpiError,
	RetSpiSuccess
} SPI_STATUS;

typedef unsigned char Bool;

// Acceptable values for SPI master side configuration
typedef enum _SpiConfigOptions
{
	OpsNull,  			// do nothing
	OpsWakeUp,			// enable transfer
	OpsInitTransfer,
	OpsEndTransfer,

} SpiConfigOptions;


// char stream definition for
typedef struct _structCharStream
{
	uint8_t* pChar;                                // buffer address that holds the streams
	uint32_t length;                               // length of the stream in bytes
} CharStream;

SPI_STATUS Serialize_SPI(const CharStream* char_stream_send,
                         CharStream* char_stream_recv,
                         SpiConfigOptions optBefore, SpiConfigOptions optAfter) ;

SPI_STATUS SpiRodanPortInit(void);
void ConfigureSpi(SpiConfigOptions opt);
void four_byte_addr_ctl(int enable) ;



void spi_write(uint8_t command, uint8_t* m_tx_buf, uint8_t* m_rx_buf);
void spi_helper_init(void);











#endif
