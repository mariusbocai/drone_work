/*=======================*/
/* IIC communication driver */
/*=======================*/
#include "iic_com_driver.h"
#include "declarations.h"
#include "LPC2103.H"                       /* LPC21xx definitions  */

static iic_comm_t IIC_STRUCT;
volatile static iic_com_sm_t STATE_MACHINE;
unsigned char IIC_RECEIVE_CONFIRMED;
unsigned char IIC_COM_RETURN_VAL;
unsigned char LAST_ERR_LOOP;
extern unsigned int CLOCK_VAR;

void IIC_Init(unsigned int desiredBaud)
{
   /* First, let's set a baudrate*/
	if(desiredBaud == 150)
	{
	 I2C0SCLL = 100;
	 I2C0SCLH = 100;
	}
	else if(desiredBaud == 400)
	{
		I2C0SCLL = 35; //35
	 	I2C0SCLH = 40; /* 40 for a 400Khz bit frequency, at 30Mhz PCLK */
	}
   	IIC_RECEIVE_CONFIRMED = 0;
	//I2C0CONSET = EnableBitMask;
}

void ISR_IIC_COMM(void) __irq
{
   unsigned int STATUS_REG;
	
	 VICIntEnClr = 0x8000220;
   /*read the status register*/
   STATUS_REG = I2C0STAT;

   switch(STATUS_REG)
   {
      case 0x08:     /* A start condition has been transmitted */
      {
	  	I2C0DAT = AddWtoSlaveAddress;
		  I2C0CONCLR = StartBitMask;
        STATE_MACHINE.IIC_STATE = ComInProgress;
        break;
      }
      case 0x10:     /* A restart condition has been transmitted */
      {
		I2C0CONCLR = StartBitMask;
        I2C0DAT = AddRtoSlaveAddress;
        break;
      }
      case 0x18:
      {
         I2C0DAT = IIC_STRUCT.IIC_SLAVE_REG; /* Slave address has been transmitted, and ACK has been received */
         break;
      }
      case 0x20:     /*Slave address has been transmitted, an NACK has been received */
      {
				 I2C0CONSET = StopBitMask;
				 STATE_MACHINE.IIC_STATE = ComFailed;
         break;
      }
      case 0x28:     /* Data byte in DATA register has been transmitted, ACK received */
      {
         if(IIC_STRUCT.IIC_RECEIVE_ORDERED)
         {
            /*Now switch to receive mode*/
            IIC_STRUCT.IIC_OP = MasterReceive;
            IIC_STRUCT.IIC_RECEIVE_ORDERED = 0;
            /*Send restart sequence, bit STA*/
            I2C0CONSET = StartBitMask;
         }
         else
         {
            if(IIC_STRUCT.IIC_MULTI_MODE != 0)
            {
               I2C0DAT = *IIC_STRUCT.IIC_DATA_PNT++;
               IIC_STRUCT.IIC_MULTI_MODE--;
            }
            else
            {
               /*send stop, by setting bit STO*/
               I2C0CONSET = StopBitMask;
			   STATE_MACHINE.IIC_STATE = ComSuccesful;
            }
         }
         break;
      }
      case 0x30:     /* Data byte in DATA register has been transmitted, NACK received */
      {
				 //I2C0CONCLR = StopBitMask;////////
				 STATE_MACHINE.IIC_STATE = ComFailed;
         break;
      }
      case 0x38:     /* Arbitration lost */
      {
				STATE_MACHINE.IIC_STATE = ComFailed;
         break;
      }
      case 0x40:
      {
         IIC_RECEIVE_CONFIRMED = 1;
         if(IIC_STRUCT.IIC_MULTI_MODE > 1)
         {
            I2C0CONSET = AAbitMask;     /* Set the AA flag because more than 1 byte is expected, so send ACK to slave device */
         }
         break;
      }
      case 0x48:
      {
         I2C0CONSET = StopBitMask;
         break;
      }
      case 0x50:
      {
         *IIC_STRUCT.IIC_DATA_PNT++ = I2C0DAT;
         if(IIC_STRUCT.IIC_MULTI_MODE == 2)
         {
            /*Clear AA bit*/
            I2C0CONCLR = AAbitMask;
            /*Stop comm*/
         }
         IIC_STRUCT.IIC_MULTI_MODE--;
         break;
      }
	  case 0x58:
	  {
			*IIC_STRUCT.IIC_DATA_PNT++ = I2C0DAT;
	  	STATE_MACHINE.IIC_STATE = ComSuccesful;
			//I2C0CONSET = StopBitMask;//////////
		  break;
	  }
	  default:
	  {
	  	STATE_MACHINE.IIC_STATE = ComFailed;
			break;
	  }
   }
   I2C0CONCLR = ClearIntBitMask;
	 VICIntEnable = 0x8000220;
   VICVectAddr = 0;
}

void IIC_COMM_REQ(iic_comm_t* inputStruct)
{
	int i;
	IIC_STRUCT.IIC_OP = inputStruct->IIC_OP;
	IIC_STRUCT.IIC_SLAVE_ADDR = inputStruct->IIC_SLAVE_ADDR;
	IIC_STRUCT.IIC_SLAVE_REG = inputStruct->IIC_SLAVE_REG;
	IIC_STRUCT.IIC_DATA_PNT = inputStruct->IIC_DATA_PNT;
	IIC_STRUCT.IIC_MULTI_MODE = inputStruct->IIC_MULTI_MODE;

   /* Initiate the transfer, according to the request */
   if(IIC_STRUCT.IIC_OP == MasterReceive)
   {
      IIC_STRUCT.IIC_OP = MasterTransmit;
      IIC_STRUCT.IIC_RECEIVE_ORDERED = 1;
   }
   /* set EN bit to start */
   I2C0CONSET = EnableBitMask;
	 /* set STA bit to send a START condition */
	 I2C0CONSET = StartBitMask;
	 /* Change state */
	 STATE_MACHINE.IIC_STATE = ComPending;
	 /* Block function until comm is ready */
	 do {
		 i++;
	 } while (i<100);
	 if(STATE_MACHINE.IIC_STATE == ComInProgress)
	 {
		 i--;
	 }
#if !SIM_DEBUG
	 for(;;)
	 {
	 	if((STATE_MACHINE.IIC_STATE==ComSuccesful)||(STATE_MACHINE.IIC_STATE==ComFailed)||(startLoop == 1))
		{
			if(startLoop == 1)
			{
				IIC_STRUCT.IIC_FAIL_ID = IIC_STRUCT.IIC_SLAVE_REG;
				errorCause |= Error_i2c_com_too_long;
        LAST_ERR_LOOP = CLOCK_VAR;
			}
			if (STATE_MACHINE.IIC_STATE==ComFailed)
			{
			   IIC_STRUCT.IIC_FAIL_ID = IIC_STRUCT.IIC_SLAVE_REG;
			   errorCause |= Error_i2c_failed;
			}
			if((STATE_MACHINE.IIC_STATE==ComFailed) || (startLoop == 1))
			{
			   if(IIC_STRUCT.IIC_FAIL_ID == 0x69)
			   {
			      errorCause |= Error_i2c_com_gyro;
			   }
			   if(IIC_STRUCT.IIC_FAIL_ID == 0x53)
			   {
			      errorCause |= Error_i2c_com_acc;
			   }
			}
			break;
		}
	 }
	 if(startLoop == 1)
	 {
	  startLoop = 0; /*Not sure if this is a good idea*/
	 }
#endif
	I2C0CONCLR = EnableBitMask;
	STATE_MACHINE.IIC_STATE = Idle;
	 /* Possibility for improvement: return if comm is succesfull or failed */
}

