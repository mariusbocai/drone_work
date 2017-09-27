/*===== Interface to the UART =====*/
#include "lpc2103.h"
#include "hal.h"

#define fact 1
#define USE_UART1 1

void serialInit()
{
#if (USE_UART1 == 0)
	PINSEL0 &= 0xFFFFFFF0;
	PINSEL0 |= 5; /*select UART0 TxD and RxD pin functions*/
	U0LCR = 0x80; //enable DLAB
	U0DLM = 0;
	U0DLL = 16;
	U0FDR = 0x10; //uncomment this for 115200 baudrate
	U0LCR = 3; //8 data bits, no parity, 1 stopbit, disable DLAB
	U0FCR = 1; //enable FIFO
#else
	//PINSEL1 &= 0xFFFFFFF0;
	//PINSEL1 |= 5; /*select UART0 TxD and RxD pin functions*/
	reqDioFunction(8,1); //P0.8 = TXD1
	U1LCR = 0x80; //enable DLAB
	U1DLM = 0;
	U1DLL = 16;
	U1FDR = 0x10; //uncomment this for 115200 baudrate
	U1LCR = 3; //8 data bits, no parity, 1 stopbit, disable DLAB
	U1FCR = 1; //enable FIFO
#endif
}

/* Send one character on serial, function is blocking */
static void sendSingleChar(unsigned char valToSend)
{
#if (USE_UART1)
	U1THR = valToSend;
	do
	{} while((U1LSR&0x20)==0);
#else
	U0THR = valToSend;
	do
	{} while((U0LSR&0x20)==0);
#endif
}

void serialSendChar(double* value, unsigned char newLine)
{
	int value_int;
	unsigned int value_uint;
	unsigned char separateChars[5],index;

	index = 0;
	if(*value < 0)
	{
		sendSingleChar(45);		/* Send a "-" */
	}
	value_int = (int)((*value));
	if(*value < 0)
	{
		value_uint = ~value_int + 1;
	}
	else
	{
		value_uint = value_int;	
	}
	do
	{
		separateChars[index++] = value_uint%10;
		value_uint /= 10;
	}
	while(value_uint > 0);
	for(;index>0;index--)
	{
		sendSingleChar((unsigned char)(separateChars[index-1]+48));
	}	
	if(newLine==1)
	{
		sendSingleChar(10); 
	}
	else
	{
		/*if this is not the end of the row, send a ","*/
		sendSingleChar(44);
	}
}
