#line 1 "iic_com_driver.c"
 
 
 
#line 1 "iic_com_driver.h"
 
 
 












#line 24 "iic_com_driver.h"

typedef struct iic_comm_tag
{
   unsigned char IIC_OP;  
   unsigned char IIC_SLAVE_ADDR;  
   unsigned char IIC_SLAVE_REG;  
   unsigned char* IIC_DATA_PNT;  
   unsigned char IIC_MULTI_MODE;  
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

#line 5 "iic_com_driver.c"
#line 1 "declarations.h"










#line 18 "declarations.h"

typedef struct
{
	unsigned int currentEdge;
	unsigned int nextEdge;
	unsigned int risingEdgeTime;
	unsigned int fallingEdgeTime;
	unsigned int newDataAvail;
	unsigned int commandUs;
	unsigned int commandPercent;
	unsigned short calibrationDone;
	unsigned short lowRange;					
	unsigned short hiRange;					
	unsigned int noDataCounter;
} input_capture_channel_t;

extern input_capture_channel_t channel[4];
extern unsigned int errorCause;
extern unsigned short startLoop;
extern unsigned int startCondCounter;
extern unsigned short startCond;



#line 6 "iic_com_driver.c"
#line 1 "LPC2103.H"
 
 
 
 
 
 
 
 
 
 




 
#line 59 "LPC2103.H"

 



 





 






 




 





 


 



 





 
#line 120 "LPC2103.H"

 
#line 140 "LPC2103.H"

 
#line 156 "LPC2103.H"


 
#line 174 "LPC2103.H"

 
#line 183 "LPC2103.H"

 






 
#line 218 "LPC2103.H"

 
#line 232 "LPC2103.H"

 
#line 241 "LPC2103.H"

 
#line 252 "LPC2103.H"

 
#line 271 "LPC2103.H"

 
#line 290 "LPC2103.H"

 


 


 


 





#line 7 "iic_com_driver.c"

static iic_comm_t IIC_STRUCT;
volatile static iic_com_sm_t STATE_MACHINE;
unsigned char IIC_RECEIVE_CONFIRMED;
unsigned char IIC_COM_RETURN_VAL;

void IIC_Init(unsigned int desiredBaud)
{
    
	if(desiredBaud == 150)
	{
	 (*((volatile unsigned short*) 0xE001C014)) = 200;
	 (*((volatile unsigned short*) 0xE001C010)) = 200;  
	}
	else if(desiredBaud == 400)
	{
		(*((volatile unsigned short*) 0xE001C014)) = 35; 
	 	(*((volatile unsigned short*) 0xE001C010)) = 40;  
	}
   	IIC_RECEIVE_CONFIRMED = 0;
	
}

void ISR_IIC_COMM(void) __irq
{
   unsigned int STATUS_REG;
    
   STATUS_REG = (*((volatile unsigned char *) 0xE001C004));

   switch(STATUS_REG)
   {
      case 0x08:      
      {
	  	(*((volatile unsigned char *) 0xE001C008)) = ((IIC_STRUCT . IIC_SLAVE_ADDR<<1) );
		(*((volatile unsigned char *) 0xE001C018)) = 0x20;
        STATE_MACHINE.IIC_STATE = 1;
        break;
      }
      case 0x10:      
      {
		(*((volatile unsigned char *) 0xE001C018)) = 0x20;
        (*((volatile unsigned char *) 0xE001C008)) = ((IIC_STRUCT . IIC_SLAVE_ADDR<<1)|1);
        break;
      }
      case 0x18:
      {
         (*((volatile unsigned char *) 0xE001C008)) = IIC_STRUCT.IIC_SLAVE_REG;  
         break;
      }
      case 0x20:      
      {
				 (*((volatile unsigned char *) 0xE001C000)) = 0x10;
				 STATE_MACHINE.IIC_STATE = 3;
         break;
      }
      case 0x28:      
      {
         if(IIC_STRUCT.IIC_RECEIVE_ORDERED)
         {
             
            IIC_STRUCT.IIC_OP = 1;
            IIC_STRUCT.IIC_RECEIVE_ORDERED = 0;
             
            (*((volatile unsigned char *) 0xE001C000)) = 0x20;
         }
         else
         {
            if(IIC_STRUCT.IIC_MULTI_MODE != 0)
            {
               (*((volatile unsigned char *) 0xE001C008)) = *IIC_STRUCT.IIC_DATA_PNT++;
               IIC_STRUCT.IIC_MULTI_MODE--;
            }
            else
            {
                
               (*((volatile unsigned char *) 0xE001C000)) = 0x10;
			   STATE_MACHINE.IIC_STATE = 2;
            }
         }
         break;
      }
      case 0x30:      
      {
				 
				 STATE_MACHINE.IIC_STATE = 3;
         break;
      }
      case 0x38:      
      {
				STATE_MACHINE.IIC_STATE = 3;
         break;
      }
      case 0x40:
      {
         IIC_RECEIVE_CONFIRMED = 1;
         if(IIC_STRUCT.IIC_MULTI_MODE > 1)
         {
            (*((volatile unsigned char *) 0xE001C000)) = 0x4;      
         }
         break;
      }
      case 0x48:
      {
         (*((volatile unsigned char *) 0xE001C000)) = 0x10;
         break;
      }
      case 0x50:
      {
         *IIC_STRUCT.IIC_DATA_PNT++ = (*((volatile unsigned char *) 0xE001C008));
         if(IIC_STRUCT.IIC_MULTI_MODE == 2)
         {
             
            (*((volatile unsigned char *) 0xE001C018)) = 0x4;
             
         }
         IIC_STRUCT.IIC_MULTI_MODE--;
         break;
      }
	  case 0x58:
	  {
			*IIC_STRUCT.IIC_DATA_PNT++ = (*((volatile unsigned char *) 0xE001C008));
	  	STATE_MACHINE.IIC_STATE = 2;
		  break;
	  }
	  default:
	  {
	  	STATE_MACHINE.IIC_STATE = 3;
			break;
	  }
   }
   (*((volatile unsigned char *) 0xE001C018)) = 0x8;
   (*((volatile unsigned long *) 0xFFFFF030)) = 0;
}

void IIC_COMM_REQ(iic_comm_t* inputStruct)
{
	IIC_STRUCT.IIC_OP = inputStruct->IIC_OP;
	IIC_STRUCT.IIC_SLAVE_ADDR = inputStruct->IIC_SLAVE_ADDR;
	IIC_STRUCT.IIC_SLAVE_REG = inputStruct->IIC_SLAVE_REG;
	IIC_STRUCT.IIC_DATA_PNT = inputStruct->IIC_DATA_PNT;
	IIC_STRUCT.IIC_MULTI_MODE = inputStruct->IIC_MULTI_MODE;

    
   if(IIC_STRUCT.IIC_OP == 1)
   {
      IIC_STRUCT.IIC_OP = 2;
      IIC_STRUCT.IIC_RECEIVE_ORDERED = 1;
   }
    
   (*((volatile unsigned char *) 0xE001C000)) = 0x40;
	  
	 (*((volatile unsigned char *) 0xE001C000)) = 0x20;
	  
	 STATE_MACHINE.IIC_STATE = 4;
	  

	 for(;;)
	 {
	 	if((STATE_MACHINE.IIC_STATE==2)||(STATE_MACHINE.IIC_STATE==3)||(startLoop == 1))
		{
			if(startLoop == 1)
			{
				errorCause = 6;
			}
			break;
		}
	 }
	 if(startLoop == 1)
	 {
	  startLoop = 0;
	 }

	(*((volatile unsigned char *) 0xE001C018)) = 0x40;
	STATE_MACHINE.IIC_STATE = 0;
	  
}

