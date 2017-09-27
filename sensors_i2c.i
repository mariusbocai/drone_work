#line 1 "sensors_i2c.c"
#line 1 "config.h"




 
 
 







#line 2 "sensors_i2c.c"
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

#line 3 "sensors_i2c.c"
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

#line 4 "sensors_i2c.c"
#line 1 "hal.h"



#line 5 "hal.h"
#line 1 "LPC2103.h"
 
 
 
 
 
 
 
 
 
 




 
#line 59 "LPC2103.h"

 



 





 






 




 





 


 



 





 
#line 120 "LPC2103.h"

 
#line 140 "LPC2103.h"

 
#line 156 "LPC2103.h"


 
#line 174 "LPC2103.h"

 
#line 183 "LPC2103.h"

 






 
#line 218 "LPC2103.h"

 
#line 232 "LPC2103.h"

 
#line 241 "LPC2103.h"

 
#line 252 "LPC2103.h"

 
#line 271 "LPC2103.h"

 
#line 290 "LPC2103.h"

 


 


 


 





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
#line 1 "uart.h"



extern void serialInit(void);
extern void serialSendChar(double* value, unsigned char newLine);













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

 
#line 5 "sensors_i2c.c"
#line 6 "sensors_i2c.c"

 
gyro_input_struct_t GYRO_RAW_INPUT_STRUCT;
acc_input_struct_t ACC_RAW_INPUT_STRUCT;
mag_input_struct_t MAG_RAW_INPUT_STRUCT;
unsigned char GYRO_INIT_INPUT_STRUCT[5];

unsigned char GyroQueueIndex;
unsigned char GyroRawQueueIndex;
unsigned char AccQueueIndex;

#line 24 "sensors_i2c.c"

 
 
 
 
unsigned char GYRO_INIT_VAL[] = {0x4F, 0x21, 0x0, 0x0, 0x0};
const iic_comm_t GYRO_INIT_STRUCT = {2, 0x69, 0xA0, GYRO_INIT_VAL ,5, 0};

unsigned char GYRO_FIFO_INIT_VAL = 0x0;	 
const iic_comm_t GYRO_INIT_STRUCT_FIFO = {2, 0x69, 0xAE, &GYRO_FIFO_INIT_VAL ,1, 0};

 
const iic_comm_t GYRO_READ_STRUCT = {1, 0x69, 0xA6, (unsigned char*)&GYRO_RAW_INPUT_STRUCT, 8, 0};
const iic_comm_t GYRO_INIT_READ_STRUCT = {1, 0x69, 0xA0, (unsigned char*)&GYRO_INIT_INPUT_STRUCT, 5, 0};

 
gyro_angular_data_t GYRO_DATA_STRUCT;

 
 
 

unsigned char ACC_INIT_VAL = 0x8;
const iic_comm_t ACC_INIT_STRUCT = {2, 0x53, 0x2D, &ACC_INIT_VAL ,1, 0};

unsigned char ACC_FIFO_INIT_VAL = 0xC0;
const iic_comm_t ACC_FIFO_INIT_STRUCT = {2, 0x53, 0x38, &ACC_FIFO_INIT_VAL ,1, 0};

const iic_comm_t ACC_READ_STRUCT = {1, 0x53, 0x32, (unsigned char*)&ACC_RAW_INPUT_STRUCT, 6, 0};

acc_angular_data_t ACC_DATA_STRUCT;

 
 
 
unsigned char MAG_INIT_VAL[] = {0x0};
const iic_comm_t MAG_INIT_STRUCT = {2, 0x1E, 0x02, MAG_INIT_VAL ,1, 0};
const iic_comm_t MAG_READ_STRUCT = {1, 0x1E, 0x3, (unsigned char*)&MAG_RAW_INPUT_STRUCT, 6, 0};

mag_data_struct_t MAG_DATA_STRUCT;

 
 
 
unsigned char BARO_INIT_VAL[] = {0x0, 0x0, 0x0, 0x0, 0x0};
const iic_comm_t BARO_INIT_STRUCT = {2, 0x69, 0xA0, BARO_INIT_VAL ,5, 0};

 
void gyroCalculateData()
{
	signed short tempVar;




	if(CLOCK_VAR == 1)
	{



		GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL = 0;
		GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH = 0;
		GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL = 0;
		GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH = 0;
		GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL = 0;
		GYRO_DATA_STRUCT.GYRO_Z_ANGLE_TO_EARTH = 0;
#line 100 "sensors_i2c.c"
	}
#line 107 "sensors_i2c.c"

	 
	if(GYRO_RAW_INPUT_STRUCT.STATUS_REG & 0x1)
	{
		tempVar = (signed short)((GYRO_RAW_INPUT_STRUCT.OUT_X_H<<8) | (GYRO_RAW_INPUT_STRUCT.OUT_X_L));
		GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL = (float)(((tempVar)*9)); 
		GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH += (GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL/200000);
#line 120 "sensors_i2c.c"
	}
	if(GYRO_RAW_INPUT_STRUCT.STATUS_REG & 0x2)
	{
		tempVar = (signed short)((GYRO_RAW_INPUT_STRUCT.OUT_Y_H<<8) | (GYRO_RAW_INPUT_STRUCT.OUT_Y_L));
		GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL = (float)(((tempVar)*9));
		GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH += GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL/200000;
	}
	if(GYRO_RAW_INPUT_STRUCT.STATUS_REG & 0x4)
	{
		tempVar = (signed short)((GYRO_RAW_INPUT_STRUCT.OUT_Z_H<<8) | (GYRO_RAW_INPUT_STRUCT.OUT_Z_L));
		GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL = (float)(((tempVar)*9));
		GYRO_DATA_STRUCT.GYRO_Z_ANGLE_TO_EARTH += GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL/200000;
	}
}

void accCalculateData()
{
	signed short tempVar;
	
	tempVar = (signed short)((ACC_RAW_INPUT_STRUCT.OUT_X_H<<8) | (ACC_RAW_INPUT_STRUCT.OUT_X_L));
	ACC_DATA_STRUCT.ACC_RAW_X_ANGLE_VEL = (double)((((double)tempVar)/256));  
	
	tempVar = (signed short)((ACC_RAW_INPUT_STRUCT.OUT_Y_H<<8) | (ACC_RAW_INPUT_STRUCT.OUT_Y_L));
	ACC_DATA_STRUCT.ACC_RAW_Y_ANGLE_VEL = (float)((((float)tempVar)/256));  
	
	tempVar = (signed short)((ACC_RAW_INPUT_STRUCT.OUT_Z_H<<8) | (ACC_RAW_INPUT_STRUCT.OUT_Z_L));
	ACC_DATA_STRUCT.ACC_RAW_Z_ANGLE_VEL = (float)((((float)tempVar)/256));  	
}

void magCalculateData()
{
	signed short tempVar;
	tempVar = (signed short)((MAG_RAW_INPUT_STRUCT.OUT_X_H<<8)|(MAG_RAW_INPUT_STRUCT.OUT_X_L));
	MAG_DATA_STRUCT.MILIGAUSS_X_AXIS = (float)(((float)tempVar)*0.92);

	tempVar = (signed short)((MAG_RAW_INPUT_STRUCT.OUT_Y_H<<8)|(MAG_RAW_INPUT_STRUCT.OUT_Y_L));
	MAG_DATA_STRUCT.MILIGAUSS_Y_AXIS = (float)(((float)tempVar)*0.92);

	tempVar = (signed short)((MAG_RAW_INPUT_STRUCT.OUT_Z_H<<8)|(MAG_RAW_INPUT_STRUCT.OUT_Z_L));
	MAG_DATA_STRUCT.MILIGAUSS_Z_AXIS = (float)(((float)tempVar)*0.92);
}

 
void STORE_GYRO_INIT_VALUES(float accXAngle_loc, float accYAngle_loc)
{
	GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH = accXAngle_loc;
	GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH = accYAngle_loc;
}

 
ret_val_t HAL_GET_GYRO_RAW_DATA_IF(gyro_raw_data_t* Buffer)
{
	ret_val_t retValue;
#line 195 "sensors_i2c.c"
	Buffer->GET_GYRO_RAW_X_ANGLE_VEL[0] = GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL;
	Buffer->GET_GYRO_RAW_Y_ANGLE_VEL[0] = GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL;
	Buffer->GET_GYRO_RAW_Z_ANGLE_VEL[0] = GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL;

	retValue = E_OK;
	return retValue;
}

 
ret_val_t HAL_GET_GYRO_DATA_IF(gyro_data_t* Buffer)
{
	ret_val_t retValue;



	if((GYRO_RAW_INPUT_STRUCT.STATUS_REG & 0x1) && (GYRO_RAW_INPUT_STRUCT.STATUS_REG & 0x2) && (GYRO_RAW_INPUT_STRUCT.STATUS_REG & 0x4) )
	{
#line 234 "sensors_i2c.c"
		Buffer->ANGLE_RELATIVE_TO_EARTH_X[0] = GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH;
		Buffer->ANGLE_RELATIVE_TO_EARTH_Y[0] = GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH;
		Buffer->ANGLE_RELATIVE_TO_EARTH_Z[0] = GYRO_DATA_STRUCT.GYRO_Z_ANGLE_TO_EARTH;

	}
	else retValue = E_NOK;
	return retValue;
}

ret_val_t HAL_GET_ACC_DATA_IF(acc_data_t* Buffer)
{
#line 267 "sensors_i2c.c"
		Buffer->ACCELERATIO_AXIS_X[0] = ACC_DATA_STRUCT.ACC_RAW_X_ANGLE_VEL;
		Buffer->ACCELERATIO_AXIS_Y[0] = ACC_DATA_STRUCT.ACC_RAW_Y_ANGLE_VEL;
		Buffer->ACCELERATIO_AXIS_Z[0] = ACC_DATA_STRUCT.ACC_RAW_Z_ANGLE_VEL;

	return E_OK;
}

