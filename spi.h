#ifndef SPI_H_INCLUDED_
#define SPI_H_INCLUDED_

#define Output 0
#define Input 1

typedef struct {
	unsigned char chipSelectPin;
	unsigned char numberOfBytes;
	unsigned char *pTxBuffer;
	unsigned char *pRxBuffer;
} spi_comm_t;

extern void SPI_Init(void);
extern unsigned short SPI_Send(spi_comm_t *dataStruct);

#endif
