/*=======================*/
/* IIC com driver header */
/*=======================*/
#ifndef IIC_COM_DRIVER_INCLUDED
#define IIC_COM_DRIVER_INCLUDED

#define MasterReceive 1
#define MasterTransmit 2

#define Idle 0
#define ComInProgress 1
#define ComSuccesful 2
#define ComFailed 3
#define ComPending 4

#define GetRmode(x) x&1
#define AddRtoSlaveAddress ((IIC_STRUCT.IIC_SLAVE_ADDR<<1)|1)
#define AddWtoSlaveAddress ((IIC_STRUCT.IIC_SLAVE_ADDR<<1)/*&(~1)*/)
#define StopBitMask 0x10
#define StartBitMask 0x20
#define EnableBitMask 0x40
#define ClearIntBitMask 0x8
#define AAbitMask 0x4

typedef struct iic_comm_tag
{
   unsigned char IIC_OP; /*select operating mode: master send or master receive*/
   unsigned char IIC_SLAVE_ADDR; /* 7 - bit slave address WARNING!!!!  the driver must add the R/W bit at the end (LSB)*/
   unsigned char IIC_SLAVE_REG; /*address from slave that will be accesed*/
   unsigned char* IIC_DATA_PNT; /* pointer to data */
   unsigned char IIC_MULTI_MODE; /*set this var to N if N bytes need to be transmitted*/
   unsigned char IIC_RECEIVE_ORDERED; 
} iic_comm_t;

typedef struct iic_com_sm_tag
{
	unsigned char IIC_STATE;
} iic_com_sm_t;

extern iic_comm_t IIC_STRUCT;
extern void IIC_Init(unsigned int desiredBaud);
extern void IIC_COMM_REQ(iic_comm_t* inputStruct);
void ISR_IIC_COMM(void) __irq;

#endif
