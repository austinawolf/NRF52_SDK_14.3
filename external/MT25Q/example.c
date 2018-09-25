#include <stdio.h>
#include "MT25Q.h"
#include "Serialize.h"

int main(int argc, char ** argv)
{
	/* flash device object */
	FLASH_DEVICE_OBJECT fdo;
	/* parameters used for all operation */
	ParameterType para;
	/* return variable */
	ReturnType ret;
	/* read buffer */
	uint8 rbuffer[16];
	/* write buffer */
	uint8 wbuffer[16] = {
		0xBE, 0xEF, 0xFE, 0xED, 0xBE, 0xEF, 0xFE, 0xED,
		0xBE, 0xEF, 0xFE, 0xED, 0xBE, 0xEF, 0xFE, 0xED
	};

	/* inizialize SPI interface */
	SpiRodanPortInit();

	/* inizialize the driver */
	ret = Driver_Init(&fdo);
	if (Flash_WrongType == ret) {
		printf("Sorry, no device detected.\n");
		return -1;
	}

	/* erase first sector */
	fdo.GenOp.SectorErase(0);

	/* program 16 byte at address 0 */
	para.PageProgram.udAddr = 0;
	para.PageProgram.pArray = wbuffer;
	para.PageProgram.udNrOfElementsInArray = 16;
	fdo.GenOp.DataProgram(PageProgram, &para);

	/* read 16 byte at address 0 */
	para.Read.udAddr = 0;
	para.Read.pArray = rbuffer;
	para.Read.udNrOfElementsToRead = 16;
	fdo.GenOp.DataRead(Read, &para);

	/* now rbuffer contains written elements */
	printf("The first device byte is: 0x%x\n", rbuffer[0]);

	return 0;
}

