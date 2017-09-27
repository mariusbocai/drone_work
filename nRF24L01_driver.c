#include "LPC2103.H" 
#include "config.h"
#include "hal.h"

unsigned char TxBuffer[5] = {0, 0, 0, 0, 0};
unsigned char RxBuffer[5] = {0, 0, 0, 0, 0};

//static unsigned char Initialized = 0;

spi_comm_t Reg_Write_1byte = {1, 2, TxBuffer, RxBuffer};
spi_comm_t Reg_Write_2bytes = {1, 1, TxBuffer, RxBuffer};

#if USE_24L01_TELEMETRY
#endif

/*===== Init registers =====*/
void initTelemetry(void)
{
#if 0
	Initialized = 0xAA;
	
	TxBuffer[0] = 0x20;
	TxBuffer[1] = 0xB;
	(void)SPI_Send(&Reg_Write_2bytes);
#endif
}

void telemetryMain(void)
{
#if 0
	if(Initialized != 0xAA) initTelemetry();
	TxBuffer[0] = 0xA0;
	TxBuffer[1] = 0x55;
	(void)SPI_Send(&Reg_Write_2bytes);
#endif
}
