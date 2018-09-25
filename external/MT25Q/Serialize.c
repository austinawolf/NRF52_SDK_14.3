/**********************  DRIVER FOR SPI CONTROLLER ON ORION**********************

   Filename:    Serialize.c
   Description:  Support to . This files is aimed at giving a basic example of
   SPI Controller to simulate the SPI serial interface.

   Version:    0.2
   Date:       Decemb. 2011
   Authors:    Micron S.r.l. Arzano (Napoli)


   THE FOLLOWING DRIVER HAS BEEN TESTED ON ORION WITH FPGA RELEASE 1.4 NON_DDR.

   Since there is no dedicated QUAD SPI controller on PXA processor, the peripheral is
   synthesized on FPGA and it is interfaced with CPU through memory controller. It is
   implemented on chip select-5 region of PXA processor and communicates with device using
   32bits WRFIFO and RDFIFO. This controller is aligned with  Micron SPI Flash Memory.
   That's mean that in extended, dual and quad mode works only with command of these memory.

   These are list of address for different SPI controller registers:

   			Chip Base  is mapped at 0x16000000

   Register         |     Address          |    Read/Write
                    |                      |
   RXFIFO           | (Chip Base + 0x0)    |      Read
   WRFIFO           | (Chip Base + 0x4)    |      Write
   Control Register | (Chip Base + 0x8)    |      R/W
   Status Register  | (Chip Base + 0xC)    |      Read



********************************************************************************

*******************************************************************************/
#include "Serialize.h"
#include "spi_interface.h"


#define EXT_MOD

#define ENABLE_PRINT_DEBUG
#define printf NRF_LOG_RAW_INFO
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/*******************************************************************************
Function:     ConfigureSpi(SpiConfigOptions opt)
Arguments:    opt configuration options, all acceptable values are enumerated in
              SpiMasterConfigOptions, which is a typedefed enum.
Return Values:There is no return value for this function.
Description:  This function can be used to properly configure the SPI master
              before and after the transfer/receive operation
Pseudo Code:
   Step 1  : perform or skip select/deselect slave
   Step 2  : perform or skip enable/disable transfer
   Step 3  : perform or skip enable/disable receive
*******************************************************************************/

void ConfigureSpi(SpiConfigOptions opt)
{
	switch (opt)
	{
	case OpsWakeUp:
		//CHECK_BSY;
		//SET_CS;
		break;
	case OpsInitTransfer:
		//FLUSHRWFIFO;
		break;
	case OpsEndTransfer:
		//CLEAR_CS;
		//FLUSHRWFIFO;
		break;
	default:
		break;
	}
}



/*******************************************************************************
Function:     Serialize(const CharStream* char_stream_send,
					CharStream* char_stream_recv,
					SpiMasterConfigOptions optBefore,
					SpiMasterConfigOptions optAfter
				)
Arguments:    char_stream_send, the char stream to be sent from the SPI master to
              the Flash memory, usually contains instruction, address, and data to be
              programmed.
              char_stream_recv, the char stream to be received from the Flash memory
              to the SPI master, usually contains data to be read from the memory.
              optBefore, configurations of the SPI master before any transfer/receive
              optAfter, configurations of the SPI after any transfer/receive
Return Values:TRUE
Description:  This function can be used to encapsulate a complete transfer/receive
              operation
Pseudo Code:
   Step 1  : perform pre-transfer configuration
   Step 2  : perform transfer/ receive
   Step 3  : perform post-transfer configuration
*******************************************************************************/
SPI_STATUS Serialize_SPI(const CharStream* char_stream_send,
                         CharStream* char_stream_recv,
                         SpiConfigOptions optBefore,
                         SpiConfigOptions optAfter
                        )
{
	uint8 *char_send, *char_recv;
	uint16 rx_len = 0, tx_len = 0;



	tx_len = char_stream_send->length;
	char_send = char_stream_send->pChar;

	#ifdef ENABLE_PRINT_DEBUG
		int i;
		printf("SEND: ");
		for(i=0; i<tx_len; i++)
			printf(" 0x%x ", char_send[i]);
		printf("\n");
	#endif	
	
	
	if (NULL_PTR != char_stream_recv)
	{
		rx_len = char_stream_recv->length;
		char_recv = char_stream_recv->pChar;
	}
	else {
		rx_len = 0;
		char_recv = NULL_PTR;		
	}



	ConfigureSpi(optBefore);


	spi_transfer(char_send, tx_len, char_recv, rx_len);

	

#ifdef ENABLE_PRINT_DEBUG
	printf("RECV: ");
	for(i=0; i<rx_len; i++)
		printf(" 0x%x ", char_recv[i]);
	printf("\n");
#endif

	ConfigureSpi(optAfter);


	return RetSpiSuccess;
}

void four_byte_addr_ctl(int enable)
{
	if(enable)
		//FOUR_BYTE_ENABLE;
		enable = 0;

	if(!enable)
		//FOUR_BYTE_DISABLE;
		enable = 0;
}
