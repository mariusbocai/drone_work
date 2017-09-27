#include <stdio.h>                         /* standard I/O .h-file */
#include "LPC2103.H"                       /* LPC2103 definitions  */
#include "declarations.h"
#include "iic_com_driver.h"
#include "sensors_i2c.h"
#include "uart.h"
#include "hal.h"
#include "spi.h"
#include "pid.h"

/*Last changed: 19/11/2013*/
//Remote Futaba T7CP:
// Channel1 = Aileron
// Channel2 = Elevator

//• CAP0.0: P0.2   = not used
//• CAP0.1: P0.4   = not used
//• CAP0.2: P0.6   = not used
//• CAP1.0: P0.10  = THR
//• CAP1.1: P0.11  = AIL (roll)
//• CAP1.2: P0.17  = ELE
//• CAP1.3: P0.18  = RUD

//outputs for LPC2103
//MAT3.0 P0.21 = Motor 1
//MAT3.1 P0.0  = Motor 2
//MAT3.2 P0.1  = Motor 3
//MAT2.0 P0.7  = Motor 4

input_capture_channel_t channel[4];
unsigned int errorCause;
unsigned short startLoop;
unsigned int startCondCounter;
unsigned short startCond;

unsigned long CLOCK_VAR;
unsigned long nextMagReadLoop;

static unsigned char initDone;

stick_data_t INPUT_COMM;

unsigned int SAVE_TC;
double MCU_LOAD; /*Holds the loop time in percentage*/ 

/*===== Declare and Initialize the array of function pointers =====*/
INIT_FUNC_ARRAY;

/*===== The interrupt below is called once every 5ms =====*/
void IRQ_Loop (void) __irq
{
	int i;
	CLOCK_VAR++;
	i = T3IR;
	T3IR = 8;
	i=i;
	if(startLoop == 0)
	{
		MCU_LOAD = SAVE_TC/50;
	}
	else
	{
		errorCause = Error_loop_time;
	}
	if(initDone == 1) /*Only start a new loop if the INIT is done! Otherwise the IIC initializations might not have enough time*/
	{
		startLoop = 1;
	}
	VICVectAddr =0;
} 

/*===== The interrupt below is called whenever we get a timer capture event =====*/
void IRQ_Capture (void) __irq 
{
	int intSource = 0;
	VICIntEnClr = 0x20;			//disable Timer1 interrupts by writing 1 to the Timer1 interrupt slot
	intSource = T1IR;			//read the IR register to see who caused the interrupt
	if((intSource&16)==16)
	{
		if(channel[0].nextEdge == Rising)
		{
			channel[0].risingEdgeTime = T1CR0;
			channel[0].nextEdge = Falling;
			channel[0].newDataAvail = False;
			//disable CAP1.0 rising edge interrupt, enable falling edge interrupt
			T1CCR &= 0xFFFFFFFE;
			T1CCR |= 2;
		}
		else
		{
			channel[0].fallingEdgeTime = T1CR0;
			channel[0].nextEdge = Rising;
			channel[0].newDataAvail = True;
			//process the data here
			if(channel[0].fallingEdgeTime < channel[0].risingEdgeTime)
			{
				//the following line is executed in case the timer1 has overflowed between the rising and falling edges
				channel[0].commandUs = 65536 - channel[0].risingEdgeTime + channel[0].fallingEdgeTime;
			}
			else
			{
				channel[0].commandUs = channel[0].fallingEdgeTime - channel[0].risingEdgeTime;
			}
			//disable CAP1.0 falling edge interrupt, enable rising edge interrupt
			T1CCR &= 0xFFFFFFFD;
			T1CCR |= 1;	
		}
		T1IR = 16;
	}
	else if((intSource&32)==32)
	{
		if(channel[1].nextEdge == Rising)
		{
			channel[1].risingEdgeTime = T1CR1;
			channel[1].nextEdge = Falling;
			channel[1].newDataAvail = False;
			//disable CAP1.1 rising edge interrupt, enable falling edge interrupt
			T1CCR &= 0xFFFFFFF7;
			T1CCR |= 0x10;
		}
		else
		{
			channel[1].fallingEdgeTime = T1CR1;
			channel[1].nextEdge = Rising;
			channel[1].newDataAvail = True;
			//process the data here
			if(channel[1].fallingEdgeTime < channel[1].risingEdgeTime)
			{
				//the following line is executed in case the timer1 has overflowed between the rising and falling edges
				channel[1].commandUs = 65536 - channel[1].risingEdgeTime + channel[1].fallingEdgeTime;
			}
			else
			{
				channel[1].commandUs = channel[1].fallingEdgeTime - channel[1].risingEdgeTime;
			}
			//disable CAP1.1 falling edge interrupt, enable rising edge interrupt
			T1CCR &= 0xFFFFFFEF;
			T1CCR |= 0x8;	
		}
		T1IR = 32;
	}
	else if((intSource&64)==64)
	{
		if(channel[2].nextEdge == Rising)
		{
			channel[2].risingEdgeTime = T1CR2;
			channel[2].nextEdge = Falling;
			channel[2].newDataAvail = False;
			//disable CAP1.2 rising edge interrupt, enable falling edge interrupt
			T1CCR &= 0xFFFFFFBF;
			T1CCR |= 0x80;
		}
		else
		{
			channel[2].fallingEdgeTime = T1CR2;
			channel[2].nextEdge = Rising;
			channel[2].newDataAvail = True;
			//process the data here
			if(channel[2].fallingEdgeTime < channel[2].risingEdgeTime)
			{
				//the following line is executed in case the timer1 has overflowed between the rising and falling edges
				channel[2].commandUs = 65536 - channel[2].risingEdgeTime + channel[2].fallingEdgeTime;
			}
			else
			{
				channel[2].commandUs = channel[2].fallingEdgeTime - channel[2].risingEdgeTime;
			}
			//disable CAP1.2 falling edge interrupt, enable rising edge interrupt
			T1CCR &= 0xFFFFFF7F;
			T1CCR |= 0x40;		
		}
		T1IR = 64;
	}
	else if((intSource&128)==128)
	{
		if(channel[3].nextEdge == Rising)
		{
			channel[3].risingEdgeTime = T1CR3;
			channel[3].nextEdge = Falling;
			channel[3].newDataAvail = False;
			//disable CAP1.3 rising edge interrupt, enable falling edge interrupt
			T1CCR &= 0xFFFFFDFF;
			T1CCR |= 0x400;
		}
		else
		{
			channel[3].fallingEdgeTime = T1CR3;
			channel[3].nextEdge = Rising;
			channel[3].newDataAvail = True;
			//process the data here
			if(channel[3].fallingEdgeTime < channel[3].risingEdgeTime)
			{
				//the following line is executed in case the timer1 has overflowed between the rising and falling edges
				channel[3].commandUs = 65536 - channel[3].risingEdgeTime + channel[3].fallingEdgeTime;
			}
			else
			{
				channel[3].commandUs = channel[3].fallingEdgeTime - channel[3].risingEdgeTime;
			}
			//disable CAP1.3 falling edge interrupt, enable rising edge interrupt
			T1CCR &= 0xFFFFFBFF;
			T1CCR |= 0x200;
		}
		T1IR = 128;
	}
	else
	{
		errorCause = Error_unnasigned_TCAP_interrupt;
	}
	VICIntEnable = 0x20;			//enable Timer1 interrupts
	VICVectAddr =0;
}

void initSoftware()
{
	int i=0;
	indexChart = 0; /*Configure the data output set*/
	startLoop = 0;
	errorCause = No_error;
	startCond = 0;
	CLOCK_VAR = 0;
	MCU_LOAD = 0;
  SAVE_TC = 0; 
	GyroQueueIndex = 0;
	AccQueueIndex = 0;
	GyroRawQueueIndex = 0;
	nextMagReadLoop = 15;
	STATUS_ARMED = 0;
	for(i=0;i<4;i++)
	{
		channel[i].nextEdge = Rising;
		channel[i].risingEdgeTime = 0;
		channel[i].fallingEdgeTime = 0;
		channel[i].newDataAvail = 0;
		channel[i].currentEdge = 0;
		channel[i].commandUs = 0;
		channel[i].commandPercent = 0;
		channel[i].calibrationDone = 0;
		channel[i].lowRange = 0xFFFF;
		channel[i].hiRange = 0;
		channel[i].noDataCounter = 0;
	}
	PID_Init();
}

void initInputs()
{
	APBDIV = 2;						//PCLK is 1/2 CCLK
//timer1 initialization
	T1CTCR = 0;						//timer mode selected for Timer1, TC is incremented on every rising PCLK
	T1CCR = 0xB6D;					//enable Timer1 interrupt and capture on rising edges for pins CAP1[0,1,2] = 4 capture inputs!
	T1PR = 29;						//because the PCLK is 30Mhz, this prescaler value will cause Timer1 to use 1Mhz frequency = 1us resolution is enough
	T1TCR = 1;						//this enables the Timer1, this should be the last instruction from this function

//timer3 init
	T3CTCR = 0;					//timer mode
	T3MCR = 0x600;			//Interrupt on match MR3, reset on interrupt
	T3PR = 29;
	T3MR0 = 1000;
	T3MR1 = 1000;
	T3MR2 = 1000;
	T3MR3 = 5000;
	T3PWMCON = 0x0F;
	T3TCR = 1;
	
//timer2 init
	T2CTCR = 0;		//timer mode
	T2MCR = 0x400; //reset Timer on match of MR3
	T2PR = 29;
	T2MR0 = 1000;
	T2MR3 = 5000;
	T2PWMCON = 1;
	T2TCR = 1;

//Interrupt init	
	VICIntSelect = 0x0; 				//all interrupts are IRQ
	VICVectCntl0 = 0x25;				//vector 0 used for timer1 interrupts
	VICVectCntl1 = 0x3B;				//vector 1 used for timer3 interrupts
	VICVectCntl2 = 0x29;				//vector 2 used for I2C 0 communication
	VICVectAddr0 = (unsigned)&IRQ_Capture;		//address to the ISR
	VICVectAddr1 = (unsigned)&IRQ_Loop;		//address to the ISR
	VICVectAddr2 = (unsigned)&ISR_IIC_COMM;
	VICIntEnable = 0x8000220;			//enable Timer1,3, I2C0 interrupts
	VICVectAddr = 0;
	
	reqDioFunction(21,2); //P0.21 = MAT3.0
	reqDioFunction(7,2); //P0.7 = MAT2.0
	reqDioFunction(0,2); //P0.0 = MAT3.1
	reqDioFunction(1,2); //P0.1 = MAT3.2
	reqDioFunction(2,1); //P0.2 = SCL0
	reqDioFunction(3,1); //P0.3 = SDA0
	reqDioFunction(10,2); //P0.10 = CAP1.0
	reqDioFunction(11,2); //P0.11 = CAP1.1
	reqDioFunction(17,2); //P0.17 = CAP1.2
	reqDioFunction(18,2); //P0.18 = CAP1.3
	reqDioFunction(23,0); //P0.23 = GPIO (Status LED)
	reqDioDirection(23,0);//P0.23 = output
	
	//IODIR = 0x0C;
	//IOSET = 0x0C;

	/*=== Init UART ===*/
#if(SEND_GYRO_DATA_UART)
	serialInit();
#endif
	
	/*=== Init i2c (aka iic) ===*/
	IIC_Init(400);
	
	/*=== GYRO INIT ===*/
	IIC_COMM_REQ((iic_comm_t*)&GYRO_INIT_STRUCT_FIFO);
	IIC_COMM_REQ((iic_comm_t*)&GYRO_INIT_STRUCT);
	IIC_COMM_REQ((iic_comm_t*)&GYRO_INIT_READ_STRUCT);


	/*=== ACC INIT ===*/
	IIC_COMM_REQ((iic_comm_t*)&ACC_FIFO_INIT_STRUCT);
	IIC_COMM_REQ((iic_comm_t*)&ACC_INIT_STRUCT);

	/*=== MAG INIT ===*/
	IIC_COMM_REQ((iic_comm_t*)&MAG_INIT_STRUCT);
	
	/*=== SPI INIT ===*/
	SPI_Init();
	
	/*=== RFM12B telemetry module init send function ===*/
#if(TELEMETRY_CFG)
	SPI_Send(0x80D8);	/* enable register,433MHz,12.5pF */
	Wait_time();
	SPI_Send(0x8208);	/* Turn on crystal,!PA */
	Wait_time();
	SPI_Send(0xA640);
	Wait_time();
	SPI_Send(0xC647);
	Wait_time();
	SPI_Send(0XCC77);
	Wait_time();
	SPI_Send(0x94A0);
	Wait_time();
	SPI_Send(0xC2AC);
	Wait_time();
	SPI_Send(0xCA80);
	Wait_time();
	SPI_Send(0xCA83);
	Wait_time();
	SPI_Send(0xC49B);
	Wait_time();
	SPI_Send(0x9850);
	Wait_time();
	SPI_Send(0xE000);
	Wait_time();
	SPI_Send(0xC80E);
	Wait_time();
	SPI_Send(0xC000);
	Wait_time();
	SPI_Send(0x8228); //OPEN PA
	Wait_time();
	SPI_Send(0x8238);
#endif
}

void setCommandtoAct(unsigned char channelNumber, unsigned int command)
{
	switch(channelNumber)
	{
		case 1:
		{
			T3MR1 = command;
			break;
		}
		case 2:
		{
			T3MR0 = command;
			break;
		}
		case 3:
		{
			T3MR2 = command;
			break;
		}
		case 4:
		{
			T2MR0 = command;
			break;
		}
		case 99:
		{
			T3MR0 = command;
			T3MR1 = command;
			T3MR2 = command;
			T2MR0 = command;
			/* to command all channels at once! */
			break;
		}
		default:
		{
			break;
		}
	}
}

void calculateInputs()
{
	int i = 0;
	
	for(i=0;i<4;i++)
	{
		if(channel[i].newDataAvail == True)
		{
			//check if pulse is too long or too short, make small adjustments to the pulse
			if(channel[i].commandUs > 2200)							//SAFETY FEATURE 
			{
				errorCause = Error_too_long_input_pulse;
			}
			else if(channel[i].commandUs > 2000)
			{
				channel[i].commandUs = 2000;
			}
			if(channel[i].commandUs < 800)							//SAFETY FEATURE 
			{
				errorCause = Error_too_short_input_pulse;
			}
			else if(channel[i].commandUs < 1000)
			{
				channel[i].commandUs =1000;
			}

			//now calculate the percentage of command
			//percentage is commandPercent / 10
			//commandPercent is in the range 0 - 1000!!!!!!!
			channel[i].commandPercent = channel[i].commandUs - 1000;
			if(channel[i].noDataCounter>0)
			{
				channel[i].noDataCounter = 0;
			}
		}
		else
		{
		 	channel[i].noDataCounter++;					//SAFETY FEATURE counts loops where no input data was available
			// TO DO: counter should be incremented only when consecutive data is missing
		}
		/* Use standard defined interfaces to pass information to upper layers */
		INPUT_COMM.THROTTLE_STICK = channel[0].commandPercent;
		INPUT_COMM.ROLL_STICK = channel[1].commandPercent;
		INPUT_COMM.RUDDER_STICK = channel[3].commandPercent;
		INPUT_COMM.PITCH_STICK = channel[2].commandPercent;
		if(channel[0].noDataCounter == 0)
		{
			INPUT_COMM.THROTTLE_OK = 1;
		}
		else
		{
			INPUT_COMM.THROTTLE_OK = 0;
		}
		if(channel[1].noDataCounter == 0)
		{
			INPUT_COMM.ROLL_OK = 1;
		}
		else
		{
			INPUT_COMM.ROLL_OK = 0;
		}
		if(channel[3].noDataCounter == 0)
		{
			INPUT_COMM.RUDDER_OK = 1;
		}
		else
		{
			INPUT_COMM.RUDDER_OK = 0;
		}
		if(channel[2].noDataCounter == 0)
		{
			INPUT_COMM.PITCH_OK = 1;
		}
		else
		{
			INPUT_COMM.PITCH_OK = 0;
		}
	}

/*=============================*/
/*==== Now get the IMU data ===*/
/*=============================*/
	IIC_COMM_REQ((iic_comm_t*)&GYRO_READ_STRUCT);
	IIC_COMM_REQ((iic_comm_t*)&ACC_READ_STRUCT);
	if(nextMagReadLoop == CLOCK_VAR)
	{ 
		IIC_COMM_REQ((iic_comm_t*)&MAG_READ_STRUCT);
	}
}

int main()
{
initDone = 0;
initSoftware();
initInputs();
initDone = 1;
for(;;)
{
	if(startLoop)					/* Mainloop start, call all function from here */
	{
		unsigned char index; 
		/*Get the Rx and IMU data*/
		calculateInputs(); 
		/*Calculate the angles based on the gyro data*/
		gyroCalculateData();
		/*Calculate the angles based on the acc data*/
		accCalculateData();
		/*Calculate magnetometer data*/
		if(nextMagReadLoop == CLOCK_VAR)
		{
			nextMagReadLoop += 14;
			magCalculateData();
		}
		/* Array of pointers to functions to be called */
		for(index=0; index<NumberOfFunctioCalls; index++ )
		{
			(*functioPointerArray[index])();
		}
		SAVE_TC = T3TC;
		startLoop = 0;
	}
}
}
