#include "LPC2103.H"
#include "config.h"
#include "hal.h"

/* SPI communication driver */
unsigned char COMM_COUNTER;

void SPI_Init()
{
	/*TODO: PINSEL needs to be set, otherwise no SPI comm!!!*/
	S0SPCCR = 0x1E; /*SCK rate will be at 1Mbits/sec for a 30Mhz PCLK*/
	S0SPCR = 0x824; /*CPOL = 0, 8 bits per transfer, CPHA = 0, MSB first*/
	/*Select P0.22, P0.23, P0.24 as Chip Select*/
	reqDioFunction(22,0); /*Chip Select 1*/
	reqDioDirection(22,Output); /*Output*/
	reqDioOutput(22, 1); /*Output high*/
}

unsigned short SPI_Send(spi_comm_t *dataStruct)
{
	unsigned short retValue, index;
	unsigned int saveStatusReg;
	reqDioOutput((dataStruct->chipSelectPin + 21), 0); /*Activate Chip Select*/
	/*TODO: check that the pin is really an output at this point*/
	for(index = 0; index < (dataStruct->numberOfBytes); index++)
	{
		/*Send the data*/
		S0SPDR = *(dataStruct->pTxBuffer);
		/*Get the result and store it in the Rx Buffer*/
		do
		{
			saveStatusReg = S0SPSR;
		} while(saveStatusReg == 0);
		/*Clear the Status Register for the next transfer*/
		if((S0SPSR & 0x80) != 0) /*Transfer OK*/
		{
			retValue++;
			*(dataStruct->pRxBuffer+index) = S0SPDR;
		}
		else if((saveStatusReg & 0x40) != 0) /*Write collision occured = clear by reading status reg then spi data register*/
		{
			*(dataStruct->pRxBuffer) = S0SPDR;
			break;
		}
		else if((saveStatusReg & 0x10) != 0)
		{
			S0SPCR = 0x824;
			break;
		}
	}
	reqDioOutput((dataStruct->chipSelectPin + 21), 1); /*Deactivate Chip Select*/
	return retValue; /*You know the transfer was succesfull if we received the number of bytes we sent*/
}
