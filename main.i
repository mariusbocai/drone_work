#line 1 "main.c"
#line 1 "E:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "E:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"


  
  typedef unsigned int size_t;    








 
 

 
  typedef struct __va_list __va_list;





   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 138 "E:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 957 "E:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"



 

#line 2 "main.c"
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

 


 


 


 





#line 3 "main.c"
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



#line 4 "main.c"
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

#line 5 "main.c"
#line 1 "sensors_i2c.h"




#line 6 "sensors_i2c.h"





 
extern unsigned char GYRO_INIT_VAL[];
extern const iic_comm_t GYRO_INIT_STRUCT;
extern const iic_comm_t GYRO_READ_STRUCT;
extern const iic_comm_t GYRO_INIT_READ_STRUCT;
extern unsigned char GYRO_FIFO_INIT_VAL;
extern const iic_comm_t GYRO_INIT_STRUCT_FIFO;
extern const iic_comm_t ACC_INIT_STRUCT;
extern const iic_comm_t ACC_FIFO_INIT_STRUCT;
extern const iic_comm_t ACC_READ_STRUCT;
extern const iic_comm_t MAG_INIT_STRUCT;
extern const iic_comm_t MAG_READ_STRUCT;
extern unsigned long CLOCK_VAR;

 
typedef struct gyro_input_struct_tag
{
	unsigned char OUT_TEMP;
	unsigned char STATUS_REG;
	unsigned char OUT_X_L;
	unsigned char OUT_X_H;
	unsigned char OUT_Y_L;
	unsigned char OUT_Y_H;
	unsigned char OUT_Z_L;
	unsigned char OUT_Z_H;
} gyro_input_struct_t;

 
typedef struct gyro_angular_data_tag
{
	float GYRO_RAW_X_ANGLE_VEL;
	float GYRO_RAW_Y_ANGLE_VEL;
	float GYRO_RAW_Z_ANGLE_VEL;
	float GYRO_X_ANGLE_TO_EARTH;
	float GYRO_Y_ANGLE_TO_EARTH;
	float GYRO_Z_ANGLE_TO_EARTH;
	unsigned char GYRO_NEW_DATA_AVAILABLE;
} gyro_angular_data_t;

typedef struct acc_input_struct_tag
{
	unsigned char OUT_X_L;
	unsigned char OUT_X_H;
	unsigned char OUT_Y_L;
	unsigned char OUT_Y_H;
	unsigned char OUT_Z_L;
	unsigned char OUT_Z_H;
} acc_input_struct_t;

typedef struct acc_angular_data_tag
{
	float ACC_RAW_X_ANGLE_VEL;
	float ACC_RAW_Y_ANGLE_VEL;
	float ACC_RAW_Z_ANGLE_VEL;
	float ACC_X_ANGLE_TO_EARTH;
	float ACC_Y_ANGLE_TO_EARTH;
	float ACC_Z_ANGLE_TO_EARTH;
	unsigned char ACC_NEW_DATA_AVAILABLE;
} acc_angular_data_t;

typedef struct mag_input_struct_tag
{
	unsigned char OUT_X_H;
	unsigned char OUT_X_L;
	unsigned char OUT_Z_H;
	unsigned char OUT_Z_L;
	unsigned char OUT_Y_H;
	unsigned char OUT_Y_L;
} mag_input_struct_t;

typedef struct	mag_data_struct_tag
{
	float MILIGAUSS_X_AXIS;
	float MILIGAUSS_Y_AXIS;
	float MILIGAUSS_Z_AXIS;
} mag_data_struct_t;

extern void gyroCalculateData(void);
extern void accCalculateData(void);
extern void magCalculateData(void);

#line 6 "main.c"
#line 1 "uart.h"



extern void serialInit(void);
extern void serialSendChar(double* value, unsigned char newLine);













#line 7 "main.c"
#line 1 "hal.h"



#line 1 "config.h"




 
 
 







#line 5 "hal.h"
#line 6 "hal.h"
#line 7 "hal.h"
#line 1 "dio.h"






extern void reqDioFunction(unsigned char pin_no, unsigned char pin_function);
extern void reqDioDirection(unsigned char pin_no, unsigned char pin_direction);
extern void reqDioOutput(unsigned char pin_no, unsigned char pin_output);
extern unsigned char reqDioInput(unsigned char pin_no, unsigned char pin_input);

#line 8 "hal.h"
#line 1 "spi.h"






typedef struct {
	unsigned char chipSelectPin;
	unsigned char numberOfBytes;
	unsigned char *pTxBuffer;
	unsigned char *pRxBuffer;
} spi_comm_t;

extern void SPI_Init(void);
extern unsigned short SPI_Send(spi_comm_t *dataStruct);

#line 9 "hal.h"
#line 1 "pid.h"



typedef struct
{
	double dState; 
	double iState; 
	double iMax, iMin; 
	double iGain, pGain, dGain; 
} SPid;

extern double actualRollAngle, actualPitchAngle, actualRawYawRate;
extern double rollAngleError, pitchAngleError, yawRateError;
extern double MCU_LOAD;

extern void PID_Init(void);
extern double UpdatePID(SPid * pid, double error, double position);
extern unsigned int calcAndLimitCommand(unsigned int inputThrottle, signed int commDelta);
extern unsigned int limitCommand(signed int commDelta);

#line 10 "hal.h"
#line 1 "output.h"



 




 
extern double rollPIDResult;
extern double pitchPIDResult;
extern double yawPIDResult;

extern SPid rollAxis;
extern SPid pitchAxis;
extern SPid yawAxis;

extern unsigned char selectPidCurrentLoop;

 
extern void PID_Main(void);
extern void resetStateMachine(void);

 
extern char indexChart;



#line 11 "hal.h"
#line 12 "hal.h"

 

 
enum {
	E_NOK = 0,	 
	E_OK 		 
};

 






typedef unsigned char ret_val_t;

 
typedef struct {
	float ANGLE_RELATIVE_TO_EARTH_X[5]; 	 
	float ANGLE_RELATIVE_TO_EARTH_Y[5];	 
	float ANGLE_RELATIVE_TO_EARTH_Z[5];	 
} gyro_data_t;

typedef struct {
	float GET_GYRO_RAW_X_ANGLE_VEL[5];
	float GET_GYRO_RAW_Y_ANGLE_VEL[5];
	float GET_GYRO_RAW_Z_ANGLE_VEL[5];
}gyro_raw_data_t;

typedef struct {
	float ACCELERATIO_AXIS_X[5];	 
	float ACCELERATIO_AXIS_Y[5];	 
	float ACCELERATIO_AXIS_Z[5];	 
}acc_data_t;

 
typedef struct {
	unsigned int THROTTLE_STICK;	 
	unsigned char THROTTLE_OK;		 
	unsigned int RUDDER_STICK;
	unsigned char RUDDER_OK;
	unsigned int ROLL_STICK;
	unsigned char ROLL_OK;
	unsigned int PITCH_STICK;
	unsigned char PITCH_OK;
} stick_data_t;

extern stick_data_t INPUT_COMM;
extern unsigned char STATUS_ARMED;
extern unsigned char GyroQueueIndex;
extern unsigned char AccQueueIndex;
extern unsigned char GyroRawQueueIndex;
extern unsigned char STATE;
extern unsigned char LEAVE_STATE;
extern unsigned long CLOCK_VAR;

enum {
	stateInit = 0,
	stateCalibration,
	stateRunning
};



 
enum {
	StartConditionCheck = 0,
	calcStateMachineEnum,
	DesiredAngleModel,
	ActualAngleModel,
	WirelessTelemetry,
	PIDMain,
	LedMain,
	NumberOfFunctioCalls
};

 
extern void checkSafetyTriggerCondition(void);
extern void calcDesiredAngle(void);
extern void calcActualAngle(void);
extern void telemetryMain(void);
extern void calcStateMachine(void);
extern void PID_Main(void);
extern void LED_Main(void);

 
 
 
 
 
 

#line 116 "hal.h"

 
extern ret_val_t HAL_GET_GYRO_DATA_IF(gyro_data_t* Buffer);
extern ret_val_t HAL_GET_ACC_DATA_IF(acc_data_t* Buffer);
extern ret_val_t HAL_GET_GYRO_RAW_DATA_IF(gyro_raw_data_t* Buffer);
																
 
extern void setCommandtoAct(unsigned char channelNumber, unsigned int command);

 
extern void STORE_GYRO_INIT_VALUES(float accXAngle_loc, float accYAngle_loc);

 
extern mag_data_struct_t MAG_DATA_STRUCT;
																
 
extern void LCDClear(void);
extern void LCDBitmap(char my_array[]);
extern void LCDString(char *characters);
extern void LCDInit(void);

 
#line 8 "main.c"
#line 9 "main.c"
#line 10 "main.c"

 


















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
double MCU_LOAD;   

 
void (*functioPointerArray[NumberOfFunctioCalls])() = { checkSafetyTriggerCondition, calcStateMachine, calcDesiredAngle, calcActualAngle, telemetryMain, PID_Main, LED_Main, };

 
void IRQ_Loop (void) __irq
{
	int i;
	CLOCK_VAR++;
	i = (*((volatile unsigned char *) 0xE0074000));
	(*((volatile unsigned char *) 0xE0074000)) = 8;
	i=i;
	if(startLoop == 0)
	{
		MCU_LOAD = SAVE_TC/50;
	}
	else
	{
		errorCause = 5;
	}
	if(initDone == 1)  
	{
		startLoop = 1;
	}
	(*((volatile unsigned long *) 0xFFFFF030)) =0;
} 

 
void IRQ_Capture (void) __irq 
{
	int intSource = 0;
	(*((volatile unsigned long *) 0xFFFFF014)) = 0x20;			
	intSource = (*((volatile unsigned char *) 0xE0008000));			
	if((intSource&16)==16)
	{
		if(channel[0].nextEdge == 1)
		{
			channel[0].risingEdgeTime = (*((volatile unsigned long *) 0xE000802C));
			channel[0].nextEdge = 2;
			channel[0].newDataAvail = 0;
			
			(*((volatile unsigned short*) 0xE0008028)) &= 0xFFFFFFFE;
			(*((volatile unsigned short*) 0xE0008028)) |= 2;
		}
		else
		{
			channel[0].fallingEdgeTime = (*((volatile unsigned long *) 0xE000802C));
			channel[0].nextEdge = 1;
			channel[0].newDataAvail = 1;
			
			if(channel[0].fallingEdgeTime < channel[0].risingEdgeTime)
			{
				
				channel[0].commandUs = 65536 - channel[0].risingEdgeTime + channel[0].fallingEdgeTime;
			}
			else
			{
				channel[0].commandUs = channel[0].fallingEdgeTime - channel[0].risingEdgeTime;
			}
			
			(*((volatile unsigned short*) 0xE0008028)) &= 0xFFFFFFFD;
			(*((volatile unsigned short*) 0xE0008028)) |= 1;	
		}
		(*((volatile unsigned char *) 0xE0008000)) = 16;
	}
	else if((intSource&32)==32)
	{
		if(channel[1].nextEdge == 1)
		{
			channel[1].risingEdgeTime = (*((volatile unsigned long *) 0xE0008030));
			channel[1].nextEdge = 2;
			channel[1].newDataAvail = 0;
			
			(*((volatile unsigned short*) 0xE0008028)) &= 0xFFFFFFF7;
			(*((volatile unsigned short*) 0xE0008028)) |= 0x10;
		}
		else
		{
			channel[1].fallingEdgeTime = (*((volatile unsigned long *) 0xE0008030));
			channel[1].nextEdge = 1;
			channel[1].newDataAvail = 1;
			
			if(channel[1].fallingEdgeTime < channel[1].risingEdgeTime)
			{
				
				channel[1].commandUs = 65536 - channel[1].risingEdgeTime + channel[1].fallingEdgeTime;
			}
			else
			{
				channel[1].commandUs = channel[1].fallingEdgeTime - channel[1].risingEdgeTime;
			}
			
			(*((volatile unsigned short*) 0xE0008028)) &= 0xFFFFFFEF;
			(*((volatile unsigned short*) 0xE0008028)) |= 0x8;	
		}
		(*((volatile unsigned char *) 0xE0008000)) = 32;
	}
	else if((intSource&64)==64)
	{
		if(channel[2].nextEdge == 1)
		{
			channel[2].risingEdgeTime = (*((volatile unsigned long *) 0xE0008034));
			channel[2].nextEdge = 2;
			channel[2].newDataAvail = 0;
			
			(*((volatile unsigned short*) 0xE0008028)) &= 0xFFFFFFBF;
			(*((volatile unsigned short*) 0xE0008028)) |= 0x80;
		}
		else
		{
			channel[2].fallingEdgeTime = (*((volatile unsigned long *) 0xE0008034));
			channel[2].nextEdge = 1;
			channel[2].newDataAvail = 1;
			
			if(channel[2].fallingEdgeTime < channel[2].risingEdgeTime)
			{
				
				channel[2].commandUs = 65536 - channel[2].risingEdgeTime + channel[2].fallingEdgeTime;
			}
			else
			{
				channel[2].commandUs = channel[2].fallingEdgeTime - channel[2].risingEdgeTime;
			}
			
			(*((volatile unsigned short*) 0xE0008028)) &= 0xFFFFFF7F;
			(*((volatile unsigned short*) 0xE0008028)) |= 0x40;		
		}
		(*((volatile unsigned char *) 0xE0008000)) = 64;
	}
	else if((intSource&128)==128)
	{
		if(channel[3].nextEdge == 1)
		{
			channel[3].risingEdgeTime = (*((volatile unsigned long *) 0xE0008038));
			channel[3].nextEdge = 2;
			channel[3].newDataAvail = 0;
			
			(*((volatile unsigned short*) 0xE0008028)) &= 0xFFFFFDFF;
			(*((volatile unsigned short*) 0xE0008028)) |= 0x400;
		}
		else
		{
			channel[3].fallingEdgeTime = (*((volatile unsigned long *) 0xE0008038));
			channel[3].nextEdge = 1;
			channel[3].newDataAvail = 1;
			
			if(channel[3].fallingEdgeTime < channel[3].risingEdgeTime)
			{
				
				channel[3].commandUs = 65536 - channel[3].risingEdgeTime + channel[3].fallingEdgeTime;
			}
			else
			{
				channel[3].commandUs = channel[3].fallingEdgeTime - channel[3].risingEdgeTime;
			}
			
			(*((volatile unsigned short*) 0xE0008028)) &= 0xFFFFFBFF;
			(*((volatile unsigned short*) 0xE0008028)) |= 0x200;
		}
		(*((volatile unsigned char *) 0xE0008000)) = 128;
	}
	else
	{
		errorCause = 1;
	}
	(*((volatile unsigned long *) 0xFFFFF010)) = 0x20;			
	(*((volatile unsigned long *) 0xFFFFF030)) =0;
}

void initSoftware()
{
	int i=0;
	indexChart = 0;  
	startLoop = 0;
	errorCause = 0;
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
		channel[i].nextEdge = 1;
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
	(*((volatile unsigned char *) 0xE01FC100)) = 2;						

	(*((volatile unsigned char *) 0xE0008070)) = 0;						
	(*((volatile unsigned short*) 0xE0008028)) = 0xB6D;					
	(*((volatile unsigned long *) 0xE000800C)) = 29;						
	(*((volatile unsigned char *) 0xE0008004)) = 1;						


	(*((volatile unsigned char *) 0xE0074070)) = 0;					
	(*((volatile unsigned short*) 0xE0074014)) = 0x600;			
	(*((volatile unsigned long *) 0xE007400C)) = 29;
	(*((volatile unsigned long *) 0xE0074018)) = 1000;
	(*((volatile unsigned long *) 0xE007401C)) = 1000;
	(*((volatile unsigned long *) 0xE0074020)) = 1000;
	(*((volatile unsigned long *) 0xE0074024)) = 5000;
	(*((volatile unsigned long *) 0xE0074074)) = 0x0F;
	(*((volatile unsigned char *) 0xE0074004)) = 1;
	

	(*((volatile unsigned char *) 0xE0070070)) = 0;		
	(*((volatile unsigned short*) 0xE0070014)) = 0x400; 
	(*((volatile unsigned long *) 0xE007000C)) = 29;
	(*((volatile unsigned long *) 0xE0070018)) = 1000;
	(*((volatile unsigned long *) 0xE0070024)) = 5000;
	(*((volatile unsigned long *) 0xE0070074)) = 1;
	(*((volatile unsigned char *) 0xE0070004)) = 1;


	(*((volatile unsigned long *) 0xFFFFF00C)) = 0x0; 				
	(*((volatile unsigned long *) 0xFFFFF200)) = 0x25;				
	(*((volatile unsigned long *) 0xFFFFF204)) = 0x3B;				
	(*((volatile unsigned long *) 0xFFFFF208)) = 0x29;				
	(*((volatile unsigned long *) 0xFFFFF100)) = (unsigned)&IRQ_Capture;		
	(*((volatile unsigned long *) 0xFFFFF104)) = (unsigned)&IRQ_Loop;		
	(*((volatile unsigned long *) 0xFFFFF108)) = (unsigned)&ISR_IIC_COMM;
	(*((volatile unsigned long *) 0xFFFFF010)) = 0x8000220;			
	(*((volatile unsigned long *) 0xFFFFF030)) = 0;
	
	reqDioFunction(21,2); 
	reqDioFunction(7,2); 
	reqDioFunction(0,2); 
	reqDioFunction(1,2); 
	reqDioFunction(2,1); 
	reqDioFunction(3,1); 
	reqDioFunction(10,2); 
	reqDioFunction(11,2); 
	reqDioFunction(17,2); 
	reqDioFunction(18,2); 
	reqDioFunction(23,0); 
	reqDioDirection(23,0);
	
	
	

	 

	serialInit();

	
	 
	IIC_Init(400);
	
	 
	IIC_COMM_REQ((iic_comm_t*)&GYRO_INIT_STRUCT_FIFO);
	IIC_COMM_REQ((iic_comm_t*)&GYRO_INIT_STRUCT);
	IIC_COMM_REQ((iic_comm_t*)&GYRO_INIT_READ_STRUCT);


	 
	IIC_COMM_REQ((iic_comm_t*)&ACC_FIFO_INIT_STRUCT);
	IIC_COMM_REQ((iic_comm_t*)&ACC_INIT_STRUCT);

	 
	IIC_COMM_REQ((iic_comm_t*)&MAG_INIT_STRUCT);
	
	 
	SPI_Init();
	
	 
#line 360 "main.c"
}

void setCommandtoAct(unsigned char channelNumber, unsigned int command)
{
	switch(channelNumber)
	{
		case 1:
		{
			(*((volatile unsigned long *) 0xE007401C)) = command;
			break;
		}
		case 2:
		{
			(*((volatile unsigned long *) 0xE0074018)) = command;
			break;
		}
		case 3:
		{
			(*((volatile unsigned long *) 0xE0074020)) = command;
			break;
		}
		case 4:
		{
			(*((volatile unsigned long *) 0xE0070018)) = command;
			break;
		}
		case 99:
		{
			(*((volatile unsigned long *) 0xE0074018)) = command;
			(*((volatile unsigned long *) 0xE007401C)) = command;
			(*((volatile unsigned long *) 0xE0074020)) = command;
			(*((volatile unsigned long *) 0xE0070018)) = command;
			 
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
		if(channel[i].newDataAvail == 1)
		{
			
			if(channel[i].commandUs > 2200)							
			{
				errorCause = 3;
			}
			else if(channel[i].commandUs > 2000)
			{
				channel[i].commandUs = 2000;
			}
			if(channel[i].commandUs < 800)							
			{
				errorCause = 4;
			}
			else if(channel[i].commandUs < 1000)
			{
				channel[i].commandUs =1000;
			}

			
			
			
			channel[i].commandPercent = channel[i].commandUs - 1000;
			if(channel[i].noDataCounter>0)
			{
				channel[i].noDataCounter = 0;
			}
		}
		else
		{
		 	channel[i].noDataCounter++;					
			
		}
		 
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
	if(startLoop)					 
	{
		unsigned char index; 
		 
		calculateInputs(); 
		 
		gyroCalculateData();
		 
		accCalculateData();
		 
		if(nextMagReadLoop == CLOCK_VAR)
		{
			nextMagReadLoop += 14;
			magCalculateData();
		}
		 
		for(index=0; index<NumberOfFunctioCalls; index++ )
		{
			(*functioPointerArray[index])();
		}
		SAVE_TC = (*((volatile unsigned long *) 0xE0074008));
		startLoop = 0;
	}
}
}
