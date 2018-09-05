/**********************  DRIVER FOR SPI CONTROLLER ON ORION**********************

   Filename:     Serialize.h
   Description:  Header file of Serialize.c
   Version:      0.1
   Date:         Decemb. 2011
   Authors:      Micron S.r.l. Arzano (Napoli)


   THE FOLLOWING DRIVER HAS BEEN TESTED ON ORION WITH FPGA RELEASE 1.4 NON_DDR.

********************************************************************************

   Version History.
   Ver.   Date      Comments

   0.2   Dec 2011  Alpha version

*******************************************************************************/

#ifndef _SERIALIZE_H_
#define _SERIALIZE_H_

//#include "rodan_mmrb.h"

typedef unsigned char	uint8;
typedef signed char		sint8;
typedef unsigned int	uint16;
typedef int				sint16;
typedef unsigned long	uint32;
typedef long			sint32;

#define NULL_PTR 0x0   // a null pointer
#define DSPI 0x20000



#define True  1
#define False 0
   
/*Return Type*/

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
	uint8* pChar;                                // buffer address that holds the streams
	uint32 length;                               // length of the stream in bytes
} CharStream;

SPI_STATUS Serialize_SPI(const CharStream* char_stream_send,
                         CharStream* char_stream_recv,
                         SpiConfigOptions optBefore, SpiConfigOptions optAfter) ;

SPI_STATUS SpiRodanPortInit(void);
void ConfigureSpi(SpiConfigOptions opt);

#endif //end of file


